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
  TaskExecutor(size_t num_threads)
      : thread_outputs_(num_threads) {
  }
  double error_ = 0;

  void Execute(size_t num_tasks,
               size_t read, size_t total, const double* history, Rotator* rot_left, Rotator* rot_right,
               size_t read2, size_t total2, const double* history2, Rotator* rot_left2, Rotator* rot_right2) {
    read_ = read;
    total_ = total;
    history_ = history;
    rot_left_ = rot_left;
    rot_right_ = rot_right;
    read2_ = read2;
    total2_ = total2;
    history2_ = history2;
    rot_left2_ = rot_left2;
    rot_right2_ = rot_right2;
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
        float delayed_l = history_[2 * (delayed_ix & kHistoryMask) + 1];
        float delayed_r = history_[2 * (delayed_ix & kHistoryMask) + 0];
        float delayed_l2 = history2_[2 * (delayed_ix & kHistoryMask) + 1];
        float delayed_r2 = history2_[2 * (delayed_ix & kHistoryMask) + 0];

        rot_left_[my_task].Increment(delayed_l);
        rot_right_[my_task].Increment(delayed_r);
        rot_left2_[my_task].Increment(delayed_l2);
        rot_right2_[my_task].Increment(delayed_r2);
        double left = rot_left_[my_task].GetSample();
        double right = rot_right_[my_task].GetSample();
        double left2 = rot_left2_[my_task].GetSample();
        double right2 = rot_right2_[my_task].GetSample();

        double pnorm = 1.0;
        error_ += pow(fabs(left - left2), pnorm);
        error_ += pow(fabs(right - right2), pnorm);
      }
    }
  }

  size_t num_tasks_;
  int64_t read_;
  int64_t total_;
  Rotator* rot_left_;
  Rotator* rot_right_;
  const double* history_;
  int64_t read2_;
  int64_t total2_;
  Rotator* rot_left2_;
  Rotator* rot_right2_;
  const double* history2_;
  std::vector<std::vector<double>> thread_outputs_;
  std::atomic<size_t> next_task_{0};
};

template <typename In>
void Process(
    In& input_stream, In& input_stream2,
    double *error) {
  std::vector<double> history(input_stream.channels() * kHistorySize);
  std::vector<double> input(input_stream.channels() * kBlockSize);

  std::vector<Rotator> rot_left, rot_right;
  rot_left.reserve(kNumRotators);
  rot_right.reserve(kNumRotators);
  for (int i = 0; i < kNumRotators; ++i) {
    const double frequency =
        BarkFreq(static_cast<double>(i) / (kNumRotators - 1));
    rot_left.emplace_back(frequency, input_stream.samplerate());
    rot_right.emplace_back(frequency, input_stream.samplerate());
  }
  std::vector<double> history2(input_stream2.channels() * kHistorySize);
  std::vector<double> input2(input_stream2.channels() * kBlockSize);

  std::vector<Rotator> rot_left2, rot_right2;
  rot_left2.reserve(kNumRotators);
  rot_right2.reserve(kNumRotators);
  for (int i = 0; i < kNumRotators; ++i) {
    const double frequency =
        BarkFreq(static_cast<double>(i) / (kNumRotators - 1));
    rot_left2.emplace_back(frequency, input_stream2.samplerate());
    rot_right2.emplace_back(frequency, input_stream2.samplerate());
  }

  TaskExecutor pool(40);

  int64_t total = 0;
  for (;;) {
    const int64_t read = input_stream.readf(input.data(), kBlockSize);
    const int64_t read2 = input_stream2.readf(input2.data(), kBlockSize);
    for (int i = 0; i < read; ++i) {
      int input_ix = i + total;
      history[2 * (input_ix & kHistoryMask) + 0] = input[2 * i];
      history[2 * (input_ix & kHistoryMask) + 1] = input[2 * i + 1];
    }
    for (int i = 0; i < read2; ++i) {
      int input_ix = i + total;
      history2[2 * (input_ix & kHistoryMask) + 0] = input2[2 * i];
      history2[2 * (input_ix & kHistoryMask) + 1] = input2[2 * i + 1];
    }
    if (read == 0) break;
    if (read2 == 0) break;

    pool.Execute(kNumRotators,
                 read, total, history.data(), rot_left.data(), rot_right.data(),
                 read2, total, history2.data(), rot_left2.data(), rot_right2.data());

    total += read;
  }
  *error = pool.error_;
}

}  // namespace

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);

  QCHECK_EQ(argc, 3) << "Usage: " << argv[0] << " <input1> <input2>";

  SndfileHandle input_file1(argv[1]);
  QCHECK(input_file1) << input_file1.strError();

  QCHECK_EQ(input_file1.channels(), 2);
  SndfileHandle input_file2(argv[2]);
  QCHECK(input_file2) << input_file2.strError();

  QCHECK_EQ(input_file2.channels(), 2);

  double error = 0;

  Process(input_file1, input_file2, &error);

  printf("error %g\n", error);
}
