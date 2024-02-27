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
  double window = 0.9996;
  double windowM1 = 1 - window;
  std::complex<double> exp_mia;
  int64_t delay = 0;
  int64_t advance = 0;

  Rotator(int64_t num_rotators, double frequency, const double sample_rate) {
    window = std::pow(window, 128.0 / num_rotators);  // at 40 Hz.
    window = pow(window, std::max(1.0, frequency / 40.0));
    delay = FindMedian3xLeaker(window);
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

static constexpr int64_t kBlockSize = 1 << 15;
static const int kHistorySize = (1 << 18);
static const int kHistoryMask = kHistorySize - 1;

struct RotatorFilterBank {
  RotatorFilterBank(size_t num_rotators, size_t num_channels,
                    size_t samplerate, size_t num_threads) {
    num_rotators_ = num_rotators;
    num_channels_ = num_channels;
    rotators_.reserve(num_channels_ * num_rotators_);
    max_delay_ = 0;
    for (size_t i = 0; i < num_rotators_; ++i) {
      const double frequency =
          BarkFreq(static_cast<double>(i) / (num_rotators_ - 1));
      for (size_t c = 0; c < num_channels; ++c) {
        rotators_.emplace_back(num_rotators_, frequency, samplerate);
        max_delay_ = std::max(max_delay_, rotators_.back().delay);
      }
    }
    QCHECK_LE(max_delay_, kBlockSize);
    fprintf(stderr, "Rotator bank output delay: %zu\n", max_delay_);
    for (auto& r : rotators_) {
      r.advance = max_delay_ - r.delay;
    }
    thread_outputs_.resize(num_threads);
    for (std::vector<double>& output : thread_outputs_) {
      output.resize(num_channels_ * kBlockSize, 0.f);
    }
  }

  void FilterOne(size_t f_idx, const double* history, int64_t total_in,
                 int64_t len, double* output) {
    Rotator* rot = &rotators_[f_idx * num_channels_];
    size_t out_ix = 0;
    for (int64_t i = 0; i < len; ++i) {
      int64_t delayed_ix = total_in + i - rot->advance;
      size_t histo_ix = num_channels_ * (delayed_ix & kHistoryMask);
      for (size_t c = 0; c < num_channels_; ++c) {
        float delayed = history[histo_ix + c];
        rot[c].Increment(delayed);
      }
      if (total_in + i >= max_delay_) {
        for (size_t c = 0; c < num_channels_; ++c) {
          double sample = rot[c].GetSample();
          output[out_ix * num_channels_ + c] += sample;
        }
        ++out_ix;
      }
    }
  }

  int64_t FilterAll(const double* history, int64_t total_in, int64_t len,
                    double* output, size_t output_size) {
    auto run = [&](size_t thread) {
      while (true) {
        size_t my_task = next_task_++;
        if (my_task >= num_rotators_) return;
        FilterOne(my_task, history, total_in, len,
                  thread_outputs_[thread].data());
      }
    };
    next_task_ = 0;
    std::vector<std::future<void>> futures;
    size_t num_threads = thread_outputs_.size();
    futures.reserve(num_threads);
    for (size_t i = 0; i < num_threads; ++i) {
      futures.push_back(std::async(std::launch::async, run, i));
    }
    for (size_t i = 0; i < num_threads; ++i) {
      futures[i].get();
    }
    std::fill(output, output + output_size, 0);
    for (std::vector<double>& thread_output : thread_outputs_) {
      for (int i = 0; i < thread_output.size(); ++i) {
        output[i] += thread_output[i];
        thread_output[i] = 0.f;
      }
    }
    return total_in < max_delay_ ?
        std::max<int64_t>(0, len - (max_delay_ - total_in)) : len;
  }

  size_t num_rotators_;
  size_t num_channels_;
  std::vector<Rotator> rotators_;
  int64_t max_delay_;
  std::vector<std::vector<double>> thread_outputs_;
  std::atomic<size_t> next_task_{0};
};

double SquareError(const double* input_history, const double* output,
                   size_t num_channels, size_t total, size_t output_len) {
  double res = 0.0;
  for (size_t i = 0; i < output_len; ++i) {
    int input_ix = i + total;
    size_t histo_ix = num_channels * (input_ix & kHistoryMask);
    for (size_t c = 0; c < num_channels; ++ c) {
      double in = input_history[histo_ix + c];
      double out = output[num_channels * i + c];
      double diff = in - out;
      res += diff * diff;
    }
  }
  return res;
}

template <typename In, typename Out>
void Process(
    In& input_stream, Out& output_stream,
    const std::function<void()>& start_progress = [] {},
    const std::function<void(int64_t)>& set_progress = [](int64_t written) {}) {
  const size_t num_channels = input_stream.channels();
  std::vector<double> history(num_channels * kHistorySize);
  std::vector<double> input(num_channels * kBlockSize);
  std::vector<double> output(num_channels * kBlockSize);

  RotatorFilterBank rotbank(kNumRotators, num_channels,
                            input_stream.samplerate(), /*num_threads=*/40);

  start_progress();
  int64_t total_in = 0;
  int64_t total_out = 0;
  bool done = false;
  double err = 0.0;
  while (!done) {
    int64_t read = input_stream.readf(input.data(), kBlockSize);
    if (read == 0) {
      done = true;
      read = total_in - total_out;
      std::fill(input.begin(), input.begin() + read, 0);
    }
    for (int i = 0; i < read; ++i) {
      int input_ix = i + total_in;
      size_t histo_ix = num_channels * (input_ix & kHistoryMask);
      for (size_t c = 0; c < num_channels; ++c) {
        history[histo_ix + c] = input[num_channels * i + c];
      }
    }
    int64_t output_len =
        rotbank.FilterAll(history.data(), total_in, read, output.data(),
                          output.size());
    output_stream.writef(output.data(), output_len);
    err += SquareError(history.data(), output.data(), num_channels, total_out,
                       output_len);
    total_in += read;
    total_out += output_len;
    set_progress(total_in);
  }
  err /= total_out;
  double psnr = -10.0 * std::log(err) / std::log(10.0);
  fprintf(stderr, "MSE: %f  PSNR: %f\n", err, psnr);
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
