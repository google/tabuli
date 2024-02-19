// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <functional>
#include <future>  // NOLINT
#include <sndfile.hh>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/check.h"

ABSL_FLAG(int, output_channels, 120, "number of output channels");
ABSL_FLAG(double, distance_to_interval_ratio, 40,
          "ratio of (distance between microphone and source array) / (distance "
          "between each source); default = 40cm / 10cm = 4");


namespace {

constexpr int kSubSourcePrecision = 1000;

float SquaredNorm(const std::complex<double> c) { return std::norm(c); }

double MicrophoneResponse(const double angle) {
  return 0.5f * (1.25f + std::cos(angle));
}

double ExpectedLeftToRightRatio(const double angle) {
  return (1e-3 + MicrophoneResponse(angle + M_PI / 4)) /
         (1e-3 + MicrophoneResponse(angle - M_PI / 4));
}

float ActualLeftToRightRatio(const std::complex<double> left,
                             const std::complex<double> right) {
  return std::sqrt((1e-3 + SquaredNorm(left)) / (1e-3 + SquaredNorm(right)));
}

double FindMedian3xLeaker(double window) {
  return -2.32/log(window);  // Faster approximate function.
}

constexpr int64_t kNumRotators = 128;

struct Rotator {
  std::complex<double> rot[4] = {{1, 0}, 0};
  double window = std::pow(0.9996, 128.0/kNumRotators);  // at 40 Hz.
  double windowM1 = 1 - window;
  std::complex<double> exp_mia;
  int advance = 0;

  Rotator(double frequency, const double sample_rate) {
    window = pow(window, std::max(1.0, frequency / 40.0));
    advance = 65000 - FindMedian3xLeaker(window);
    if (advance < 1) {
      advance = 1;
    }
    if (advance >= 0xfff0) {
      advance = 0xfff0;
    }
    windowM1 = 1.0 - window;
    frequency *= 2 * M_PI / sample_rate;
    exp_mia = {std::cos(frequency), -std::sin(frequency)};
  }

  void Increment(double audio) {
    rot[0] *= exp_mia;
    rot[1] *= window;
    rot[2] *= window;
    rot[3] *= window;

    audio *= 0.1;
    rot[1] += windowM1 * audio * rot[0];
    rot[2] += windowM1 * rot[1];
    rot[3] += windowM1 * rot[2];
  }
  double GetSample() const {
    return rot[0].real() * rot[3].real() + rot[0].imag() * rot[3].imag();
  }
  double SquaredAmplitude() const { return std::norm(rot[3]); }
};

double BarkFreq(double v) {
  constexpr double linlogsplit = 0.1;
  if (v < linlogsplit) {
    return 20.0 + (v / linlogsplit) * 20.0;  // Linear 20-40 Hz.
  } else {
    float normalized_v = (v - linlogsplit) * (1.0 / (1.0 - linlogsplit));
    return 40.0 * pow(500.0, normalized_v);  // Logarithmic 40-20000 Hz.
  }
}


static constexpr int64_t kBlockSize = 32768;
static const int kHistorySize = (1 << 18);
static const int kHistoryMask = kHistorySize - 1;

class TaskExecutor {
  std::vector<float> speaker_to_ratio_table;

 public:
  TaskExecutor(size_t num_threads, size_t output_channels)
      : thread_outputs_(num_threads) {
    output_channels_ = output_channels;
    for (std::vector<double>& output : thread_outputs_) {
      output.resize(output_channels * kBlockSize, 0.f);
    }
    const double distance_to_interval_ratio =
        absl::GetFlag(FLAGS_distance_to_interval_ratio);
    speaker_to_ratio_table.reserve(kSubSourcePrecision * (output_channels - 1) +
                                   1);
    for (int i = 0; i < kSubSourcePrecision * (output_channels - 1) + 1; ++i) {
      const float x_div_interval = static_cast<float>(i) / kSubSourcePrecision -
                                   0.5f * (output_channels - 1);
      const float x_div_distance = x_div_interval / distance_to_interval_ratio;
      const float angle = std::atan(x_div_distance);
      speaker_to_ratio_table.push_back(ExpectedLeftToRightRatio(angle));
    }
  }

  void Execute(size_t num_tasks, size_t read, size_t total,
               const double* history, Rotator* rot_left, Rotator* rot_right) {
    read_ = read;
    total_ = total;
    history_ = history;
    rot_left_ = rot_left;
    rot_right_ = rot_right;
    num_tasks_ = num_tasks;
    next_task_ = 0;
    std::vector<std::future<void>> futures;
    size_t num_threads = thread_outputs_.size();
    futures.reserve(num_threads);

    for (size_t i = 0; i < num_threads; ++i) {
      futures.push_back(
          std::async(std::launch::async, &TaskExecutor::Run, this, i));
    }
    for (size_t i = 0; i < num_threads; ++i) futures[i].get();
  }

  void Run(size_t thread) {
    while (true) {
      size_t my_task = next_task_++;
      if (my_task >= num_tasks_) return;
      std::vector<double>& thread_output = thread_outputs_[thread];
      for (int i = 0; i < read_; ++i) {
        int delayed_ix = total_ + i - rot_left_[my_task].advance;
        float delayed_r = history_[2 * (delayed_ix & kHistoryMask) + 0];
        float delayed_g = history_[2 * (delayed_ix & kHistoryMask) + 1];

        rot_left_[my_task].Increment(delayed_r);
        rot_right_[my_task].Increment(delayed_g);

        const float ratio =
            ActualLeftToRightRatio(rot_left_[my_task].rot[3],
                                   rot_right_[my_task].rot[3]);
        const int subspeaker_index =
            std::lower_bound(speaker_to_ratio_table.begin(),
                             speaker_to_ratio_table.end(), ratio,
                             std::greater<>()) -
            speaker_to_ratio_table.begin();


        float distance_from_center = (subspeaker_index - 0.5 * (output_channels_ - 1));
        float assumed_distance_to_line = 0.75 * (output_channels_ - 1);
        float distance_to_virtual = sqrt(distance_from_center * distance_from_center +
                                         assumed_distance_to_line * assumed_distance_to_line);
        float dist_ratio = distance_to_virtual * (1.0f / assumed_distance_to_line);
        float amp = dist_ratio * dist_ratio;

        float index =
            static_cast<float>(subspeaker_index) / kSubSourcePrecision;

        if (index < 0) {
          index = 0;
        }
        if (index >= output_channels_) {
          index = output_channels_ - 1;
        }
        double integral_index_f;
        const double fractional_index = std::modf(index, &integral_index_f);
        const int integral_index = integral_index_f;
        const double sample = .5f * (rot_left_[my_task].GetSample() +
                                     rot_right_[my_task].GetSample());
        thread_output[i * output_channels_ + integral_index] +=
            (1 - fractional_index) * sample;
        if (integral_index < output_channels_) {
          thread_output[i * output_channels_ + integral_index + 1] +=
              fractional_index * sample;
        }
      }
    }
  }

  int64_t read_;
  int64_t total_;
  size_t num_tasks_;
  size_t output_channels_;
  Rotator* rot_left_;
  Rotator* rot_right_;
  const double* history_;
  std::vector<std::vector<double>> thread_outputs_;
  std::atomic<size_t> next_task_{0};
};

template <typename In, typename Out>
void Process(
    const int output_channels, const double distance_to_interval_ratio,
    In& input_stream, Out& output_stream,
    const std::function<void()>& start_progress = [] {},
    const std::function<void(int64_t)>& set_progress = [](int64_t written) {}) {
  std::vector<double> history(input_stream.channels() * kHistorySize);
  std::vector<double> input(input_stream.channels() * kBlockSize);
  std::vector<double> output(output_channels * kBlockSize);

  std::vector<Rotator> rot_left, rot_right;
  constexpr int64_t kNumRotators = 128;
  rot_left.reserve(kNumRotators);
  rot_right.reserve(kNumRotators);
  for (int i = 0; i < kNumRotators; ++i) {
    const double frequency =
        BarkFreq(static_cast<double>(i) / (kNumRotators - 1));
    rot_left.emplace_back(frequency, input_stream.samplerate());
    rot_right.emplace_back(frequency, input_stream.samplerate());
  }

  std::vector<double> speaker_to_ratio_table;
  speaker_to_ratio_table.reserve(kSubSourcePrecision * (output_channels - 1) +
                                 1);
  for (int i = 0; i < kSubSourcePrecision * (output_channels - 1) + 1; ++i) {
    const double x_div_interval = static_cast<double>(i) / kSubSourcePrecision -
                                  0.5f * (output_channels - 1);
    const double x_div_distance = x_div_interval / distance_to_interval_ratio;
    const double angle = std::atan(x_div_distance);
    speaker_to_ratio_table.push_back(ExpectedLeftToRightRatio(angle));
  }

  TaskExecutor pool(40, output_channels);

  start_progress();
  int64_t total = 0;
  for (;;) {
    const int64_t read = input_stream.readf(input.data(), kBlockSize);
    for (int i = 0; i < read; ++i) {
      int input_ix = i + total;
      history[2 * (input_ix & kHistoryMask) + 0] = input[2 * i];
      history[2 * (input_ix & kHistoryMask) + 1] = input[2 * i + 1];
    }
    if (read == 0) break;

    pool.Execute(kNumRotators, read, total, history.data(), rot_left.data(),
                 rot_right.data());

    std::fill(output.begin(), output.end(), 0);
    for (std::vector<double>& thread_output : pool.thread_outputs_) {
      for (int i = 0; i < output.size(); ++i) {
        output[i] += thread_output[i];
        thread_output[i] = 0.f;
      }
    }
    output_stream.writef(output.data(), read);
    total += read;
    set_progress(total);
  }
}

}  // namespace

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);

  const int output_channels = absl::GetFlag(FLAGS_output_channels);
  const double distance_to_interval_ratio =
      absl::GetFlag(FLAGS_distance_to_interval_ratio);

  QCHECK_EQ(argc, 3) << "Usage: " << argv[0] << " <input> <output>";

  SndfileHandle input_file(argv[1]);
  QCHECK(input_file) << input_file.strError();

  QCHECK_EQ(input_file.channels(), 2);

  SndfileHandle output_file(
      argv[2], /*mode=*/SFM_WRITE, /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
      /*channels=*/output_channels, /*samplerate=*/input_file.samplerate());

  Process(
      output_channels, distance_to_interval_ratio, input_file, output_file,
      [] {}, [](const int64_t written) {});
}
