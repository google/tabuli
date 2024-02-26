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

int FindMedian3xLeaker(double window) {
  return static_cast<int>(-2.32/log(window));  // Approximate filter delay.
  /*
  double windowM1 = 1.0 - window;
  double half_way_to_total_sum = 1e99;
  for (int k = 0; k < 2; ++k) {
    double sum = 0;
    double rot1 = 1;
    double rot2 = 0;
    double rot3 = 0;
    for (int i = 0; i < 65000; ++i) {
      rot1 *= window;
      rot2 *= window;
      rot3 *= window;
      rot2 += windowM1 * rot1;
      rot3 += windowM1 * rot2;
      sum += rot3 * rot3;
      if (sum >= half_way_to_total_sum) {
        return i;
      }
    }
    half_way_to_total_sum = 0.5 * sum;
  }
  abort();
  */
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

    rot[1] += windowM1 * audio * rot[0];
    rot[2] += windowM1 * rot[1];
    rot[3] += windowM1 * rot[2];
  }
  double GetSample() const {
    return rot[0].real() * rot[3].real() + rot[0].imag() * rot[3].imag();
  }
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
 public:
  TaskExecutor(size_t num_threads, size_t num_channels)
      : thread_outputs_(num_threads) {
    num_channels_ = num_channels;
    for (std::vector<double>& output : thread_outputs_) {
      output.resize(num_channels * kBlockSize, 0.f);
    }
  }

  void Execute(size_t num_tasks, size_t read, size_t total,
               const double* history, Rotator* rot) {
    read_ = read;
    total_ = total;
    history_ = history;
    rot_ = rot;
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
      Rotator* my_rot = &rot_[my_task * num_channels_];
      std::vector<double>& thread_output = thread_outputs_[thread];
      for (int i = 0; i < read_; ++i) {
        int delayed_ix = total_ + i - my_rot->advance;
        size_t histo_ix = num_channels_ * (delayed_ix & kHistoryMask);
        for (size_t c = 0; c < num_channels_; ++c) {
          float delayed = history_[histo_ix + c];
          my_rot[c].Increment(delayed);
          double sample = my_rot[c].GetSample();
          thread_output[i * num_channels_ + c] += sample;
        }
      }
    }
  }

  int64_t read_;
  int64_t total_;
  size_t num_tasks_;
  size_t num_channels_;
  Rotator* rot_;
  const double* history_;
  std::vector<std::vector<double>> thread_outputs_;
  std::atomic<size_t> next_task_{0};
};

template <typename In, typename Out>
void Process(
    In& input_stream, Out& output_stream,
    const std::function<void()>& start_progress = [] {},
    const std::function<void(int64_t)>& set_progress = [](int64_t written) {}) {
  const size_t num_channels = input_stream.channels();
  std::vector<double> history(num_channels * kHistorySize);
  std::vector<double> input(num_channels * kBlockSize);
  std::vector<double> output(num_channels * kBlockSize);

  std::vector<Rotator> rot;
  rot.reserve(num_channels * kNumRotators);
  for (int i = 0; i < kNumRotators; ++i) {
    const double frequency =
        BarkFreq(static_cast<double>(i) / (kNumRotators - 1));
    for (size_t c = 0; c < num_channels; ++c) {
      rot.emplace_back(frequency, input_stream.samplerate());
    }
  }

  TaskExecutor pool(40, num_channels);

  start_progress();
  int64_t total = 0;
  for (;;) {
    const int64_t read = input_stream.readf(input.data(), kBlockSize);
    for (int i = 0; i < read; ++i) {
      int input_ix = i + total;
      size_t histo_ix = num_channels * (input_ix & kHistoryMask);
      for (size_t c = 0; c < num_channels; ++c) {
        history[histo_ix + c] = input[num_channels * i + c];
      }
    }
    if (read == 0) break;

    pool.Execute(kNumRotators, read, total, history.data(), rot.data());

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

  QCHECK_EQ(argc, 3) << "Usage: " << argv[0] << " <input> <output>";

  SndfileHandle input_file(argv[1]);
  QCHECK(input_file) << input_file.strError();

  SndfileHandle output_file(
      argv[2], /*mode=*/SFM_WRITE, /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
      /*channels=*/input_file.channels(),
      /*samplerate=*/input_file.samplerate());

  Process(input_file, output_file, [] {}, [](const int64_t written) {});
}
