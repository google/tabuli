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
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <future>  // NOLINT
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/check.h"
#include "sndfile.hh"

namespace {

double FindMedian3xLeaker(double window) {
  double half_way_to_total_sum = 1e300;
  for (int k = 0; k < 2; ++k) {
    double sum = 0;
    double v0 = 1;
    double v1 = 0;
    double v2 = 0;
    for (int i = 0; i < 65000; ++i) {
      v1 += v0;
      v2 += v1;
      v0 *= window;
      v1 *= window;
      v2 *= window;

      double amp = v2;
      sum += v2 * amp;
      if (sum >= half_way_to_total_sum) {
        return i;
      }
    }
    half_way_to_total_sum = 0.445 * sum;  // Sounds better when not quite half.
  }
  return 65000;
}

struct Rotator {
  std::complex<double> rot[4] = {{1, 0}, 0};
  double window = 0.9999;  // at 40 Hz.
  double windowM1 = 1 - window;
  double windowD = 0.99995;
  double windowDM1 = 1 - windowD;
  std::complex<double> exp_mia;
  int ix = 0;
  double advance = 0;
  double reverb_ratio = 0;

  Rotator(double frequency, const double sample_rate) {
    window = pow(window, std::max(1.0, frequency / 40.0));
    advance = 40000 - FindMedian3xLeaker(window);
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
    audio *= 0.3;
    rot[0] *= exp_mia;
    rot[1] *= window;
    rot[2] *= window;
    rot[3] *= window;

    rot[1] += windowM1 * audio * rot[0];
    rot[2] += windowM1 * rot[1];
    rot[3] += windowM1 * rot[2];
    ix++;
  }
  double GetSample() const {
    return rot[0].real() * rot[3].real() + rot[0].imag() * rot[3].imag();
  }
};

double BarkFreq(double v) {
  // should be larger for human hearing, around 0.165, but bass quality seems to
  // suffer
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
 public:
  TaskExecutor(size_t num_threads, size_t output_channels)
      : thread_outputs_(num_threads) {
    output_channels_ = output_channels;
    for (std::vector<double>& output : thread_outputs_) {
      output.resize(output_channels * kBlockSize, 0.f);
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
        double left = rot_left_[my_task].GetSample();
        double right = rot_right_[my_task].GetSample();

        thread_output[i * output_channels_ + 0] += left;
        thread_output[i * output_channels_ + 1] += right;
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
  constexpr int64_t kNumRotators = 512;
  rot_left.reserve(kNumRotators);
  rot_right.reserve(kNumRotators);
  for (int i = 0; i < kNumRotators; ++i) {
    const double frequency =
        BarkFreq(static_cast<double>(i) / (kNumRotators - 1));
    rot_left.emplace_back(frequency, input_stream.samplerate());
    rot_right.emplace_back(frequency, input_stream.samplerate());
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

ABSL_FLAG(int, output_channels, 2, "number of output channels");
ABSL_FLAG(double, distance_to_interval_ratio, 4,
          "ratio of (distance between microphone and source array) / (distance "
          "between each source); default = 40cm / 10cm = 4");

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
