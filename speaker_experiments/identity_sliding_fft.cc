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
#include <memory>
#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/check.h"
#include "absl/strings/str_split.h"
#include "sndfile.hh"

ABSL_FLAG(bool, plot_input, false, "If set, plots the input signal.");
ABSL_FLAG(bool, plot_output, false, "If set, plots the output signal.");
ABSL_FLAG(bool, plot_fft, false, "If set, plots fft of signal.");
ABSL_FLAG(int, plot_from, -1, "If non-negative, start plot from here.");
ABSL_FLAG(int, plot_to, -1, "If non-negative, end plot here.");

namespace {

template<typename T>
std::vector<std::complex<double>> FFT(const std::vector<T>& x) {
  size_t N = x.size();
  QCHECK((N & (N - 1)) == 0) << "FFT length must be power of two";
  size_t n = 0;
  while (N) {
    n += 1;
    N >>= 1;
  }
  N = x.size();
  --n;
  auto bit_reverse = [&n](size_t x) {
    size_t r = 0;
    for (size_t k = 0; k < n; ++k) {
      if (x & (1 << k)) {
        r += 1 << (n - 1 - k);
      }
    }
    return r;
  };
  auto butterfly = [](const std::complex<double>& m,
                      std::complex<double>& a, std::complex<double>& b) {
    std::complex<double> A = a;
    std::complex<double> B = m * b;
    a = A + B;
    b = A - B;
  };
  std::vector<std::complex<double>> X(N);
  for (size_t i = 0; i < N; ++i) {
    X[bit_reverse(i)] = x[i];
  }
  for (size_t s = 1; s <= n; ++s) {
    size_t m = 1 << s;
    double freq = 2 * M_PI / m;
    std::complex<double> mul = {std::cos(freq), -std::sin(freq)};
    for (size_t k = 0; k < N; k += m) {
      std::complex<double> omega = {1, 0};
      for (size_t j = 0; j < m/ 2; ++j) {
        butterfly(omega, X[k + j], X[k + j + m / 2]);
        omega *= mul;
      }
    }
  }
  return X;
}

bool CheckPosition(int64_t pos) {
  int from = absl::GetFlag(FLAGS_plot_from);
  int to = absl::GetFlag(FLAGS_plot_to);
  if (from >= 0 && pos < from) return false;
  if (to >= 0 && pos > to) return false;
  return true;
}

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

class InputSignal {
 public:
  InputSignal(const std::string& desc) {
    std::vector<std::string> params = absl::StrSplit(desc, ":");
    if (params.size() == 1) {
      input_file_ = std::make_unique<SndfileHandle>(params[0].c_str());
      QCHECK(*input_file_) << input_file_->strError();
      channels_ = input_file_->channels();
      samplerate_ = input_file_->samplerate();
      signal_type_ = SignalType::WAV;
    } else {
      channels_ = 1;
      samplerate_ = 48000;
      for (size_t i = 1; i < params.size(); ++i) {
        signal_args_.push_back(std::stod(params[i]));
      }
      if (params[0] == "impulse") {
        QCHECK(signal_args_.size() >= 3);
        signal_type_ = SignalType::IMPULSE;
      } else if (params[0] == "sine") {
        QCHECK(signal_args_.size() >= 4);
        signal_type_ = SignalType::SINE;
      } else {
        QCHECK(0) << "Unknown signal type " << params[0];
      }
    }
    if (absl::GetFlag(FLAGS_plot_input)) {
      signal_f_ = fopen("/tmp/input_signal.txt", "w");
    }
  }

  ~InputSignal() {
    if (signal_f_) fclose(signal_f_);
  }

  size_t channels() const { return channels_; }
  size_t samplerate() const { return samplerate_; }

  int64_t readf(double* data, size_t nframes) {
    if (input_file_) {
      int64_t read = input_file_->readf(data, nframes);
      if (signal_f_) {
        for (size_t i = 0; i < read; ++i) {
          if (CheckPosition(input_ix_)) {
            fprintf(signal_f_, "%zu %f\n", input_ix_, data[i * channels_]);
          }
          ++input_ix_;
        }
      }
      if (signal_f_) fflush(signal_f_);
      return read;
    }
    int64_t len = signal_args_[0];
    int64_t delay = signal_args_[1];
    double amplitude = signal_args_[2];
    double frequency = signal_type_ == SignalType::SINE ? signal_args_[3] : 0.0;
    double mul = 2 * M_PI * frequency / samplerate_;
    nframes = std::min<int64_t>(len - input_ix_, nframes);
    for (size_t i = 0; i < nframes; ++i) {
      for (size_t c = 0; c < channels_; ++c) {
        if (signal_type_ == SignalType::IMPULSE) {
          data[i * channels_ + c] = (input_ix_ == delay ? amplitude : 0.0);
        } else if (signal_type_ == SignalType::SINE) {
          data[i * channels_ + c] =
              amplitude * std::sin((input_ix_ - delay) * mul);
        }
      }
      if (signal_f_ && CheckPosition(input_ix_)) {
        fprintf(signal_f_, "%zu %f\n", input_ix_, data[i * channels_]);
      }
      ++input_ix_;
    }
    if (signal_f_) fflush(signal_f_);
    return nframes;
  }

 private:
  enum SignalType {
    WAV,
    IMPULSE,
    SINE,
  };
  SignalType signal_type_;
  std::vector<double> signal_args_;
  FILE* signal_f_ = nullptr;
  int64_t input_ix_ = 0;
  size_t channels_;
  size_t samplerate_;
  std::unique_ptr<SndfileHandle> input_file_;
};

class OutputSignal {
 public:
  OutputSignal(size_t channels, size_t samplerate)
      : channels_(channels), samplerate_(samplerate) {
    if (absl::GetFlag(FLAGS_plot_fft)) {
      save_output_ = true;
    } else if (absl::GetFlag(FLAGS_plot_output)) {
      signal_f_ = fopen("/tmp/output_signal.txt", "w");
    }
  }

  ~OutputSignal() {
    if (signal_f_) fclose(signal_f_);
  }

  void writef(const double* data, size_t nframes) {
    if (output_file_) {
      output_file_->writef(data, nframes);
    }
    if (signal_f_) {
      for (size_t i = 0; i < nframes; ++i) {
        if (CheckPosition(output_ix_)) {
          fprintf(signal_f_, "%zu %f\n", output_ix_, data[i * channels_]);
        }
        ++output_ix_;
      }
      fflush(signal_f_);
    }
    if (save_output_) {
      for (size_t i = 0; i < nframes; ++i) {
        output_.push_back(data[i * channels_]);
      }
    }
  }

  void SetWavFile(const char* fn) {
    output_file_ = std::make_unique<SndfileHandle>(
        fn, /*mode=*/SFM_WRITE, /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
        channels_, samplerate_);
  }

  void DumpFFT() {
    if (!absl::GetFlag(FLAGS_plot_fft) || !absl::GetFlag(FLAGS_plot_output)) {
      return;
    }
    size_t N = 1;
    while (N < 2 * output_.size()) N <<= 1;
    output_.resize(N);
    std::vector<std::complex<double>> fft = FFT(output_);
    signal_f_ = fopen("/tmp/output_signal.txt", "w");
    const int from = absl::GetFlag(FLAGS_plot_from);
    const int to = absl::GetFlag(FLAGS_plot_to);
    const size_t start_freq = from == -1 ? 0 : from;
    const size_t end_freq = to == -1 ? 20000 : to;
    const size_t start_i = start_freq * N / samplerate_;
    const size_t end_i = end_freq * N / samplerate_;
    for (size_t i = start_i; i < end_i; ++i) {
      fprintf(signal_f_, "%f  %f  %f\n", i * samplerate_ * 1.0 / N,
              std::abs(fft[i]));
    }
    fclose(signal_f_);
    signal_f_ = nullptr;
  }

 private:
  size_t channels_;
  size_t samplerate_;
  size_t output_ix_ = 0;
  bool save_output_ = false;
  std::vector<double> output_;
  std::unique_ptr<SndfileHandle> output_file_;
  FILE* signal_f_ = nullptr;
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

void CreatePlot() {
  std::vector<std::pair<std::string, std::string> > to_plot;
  if (absl::GetFlag(FLAGS_plot_input)) {
    to_plot.push_back({"/tmp/input_signal.txt", "input"});
  }
  if (absl::GetFlag(FLAGS_plot_output)) {
    to_plot.push_back({"/tmp/output_signal.txt", "output"});
  }
  if (to_plot.empty()) {
    return;
  }
  FILE* f = fopen("/tmp/plot.txt", "w");
  fprintf(f, "set term pngcairo\n");
  fprintf(f, "set output \"plot.png\"\n");
  fprintf(f, "plot ");
  for (size_t i = 0; i < to_plot.size(); ++i) {
    fprintf(f, "\"%s\" with lines title \"%s\"%s",
            to_plot[i].first.c_str(), to_plot[i].second.c_str(),
            i + 1 < to_plot.size() ? ", \\\n     " : "\n");
  }
  fclose(f);
  system("gnuplot /tmp/plot.txt");
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<char*> posargs = absl::ParseCommandLine(argc, argv);
  QCHECK_GE(posargs.size(), 2) << "Usage: " << argv[0] << " <input> [<output>]";
  InputSignal input(posargs[1]);
  OutputSignal output(input.channels(), input.samplerate());
  if (posargs.size() > 2) {
    output.SetWavFile(posargs[2]);
  }
  Process(input, output, [] {}, [](const int64_t written) {});
  output.DumpFFT();
  CreatePlot();
}
