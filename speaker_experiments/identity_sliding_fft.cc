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
#include "fourier_bank.h"
#include "sndfile.hh"

ABSL_FLAG(bool, plot_input, false, "If set, plots the input signal.");
ABSL_FLAG(bool, plot_output, false, "If set, plots the output signal.");
ABSL_FLAG(bool, plot_fft, false, "If set, plots fft of signal.");
ABSL_FLAG(bool, ppm, false, "If set, outputs ppm plot.");
ABSL_FLAG(int, plot_from, -1, "If non-negative, start plot from here.");
ABSL_FLAG(int, plot_to, -1, "If non-negative, end plot here.");
ABSL_FLAG(double, gain, 1.0, "Global volume scaling.");
ABSL_FLAG(std::string, filter_mode, "identity", "Filter mode.");

namespace tabuli {

template <typename T>
std::vector<std::complex<float>> FFT(const std::vector<T>& x) {
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
  auto butterfly = [](const std::complex<float>& m, std::complex<float>& a,
                      std::complex<float>& b) {
    std::complex<float> A = a;
    std::complex<float> B = m * b;
    a = A + B;
    b = A - B;
  };
  std::vector<std::complex<float>> X(N);
  for (size_t i = 0; i < N; ++i) {
    X[bit_reverse(i)] = x[i];
  }
  for (size_t s = 1; s <= n; ++s) {
    size_t m = 1 << s;
    float freq = 2 * M_PI / m;
    std::complex<float> mul = {std::cos(freq), -std::sin(freq)};
    for (size_t k = 0; k < N; k += m) {
      std::complex<float> omega = {1, 0};
      for (size_t j = 0; j < m / 2; ++j) {
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

FilterMode GetFilterMode() {
  std::string desc = absl::GetFlag(FLAGS_filter_mode);
  if (desc == "identity") {
    return IDENTITY;
  } else if (desc == "amplitude") {
    return AMPLITUDE;
  } else if (desc == "phase") {
    return PHASE;
  }
  QCHECK(0);
}

float SquareError(const float* input_history, const float* output,
                  size_t num_channels, size_t total, size_t output_len) {
  float res = 0.0;
  for (size_t i = 0; i < output_len; ++i) {
    int input_ix = i + total;
    size_t histo_ix = num_channels * (input_ix & kHistoryMask);
    for (size_t c = 0; c < num_channels; ++c) {
      float in = input_history[histo_ix + c];
      float out = output[num_channels * i + c];
      float diff = in - out;
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

  int64_t readf(float* data, size_t nframes) {
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
    float amplitude = signal_args_[2];
    float frequency = signal_type_ == SignalType::SINE ? signal_args_[3] : 0.0;
    float mul = 2 * M_PI * frequency / samplerate_;
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
  std::vector<float> signal_args_;
  FILE* signal_f_ = nullptr;
  int64_t input_ix_ = 0;
  size_t channels_;
  size_t samplerate_;
  std::unique_ptr<SndfileHandle> input_file_;
};

class OutputSignal {
 public:
  OutputSignal(size_t channels, size_t freq_channels, size_t samplerate,
               bool save_output)
      : channels_(channels),
        freq_channels_(freq_channels),
        samplerate_(samplerate),
        save_output_(save_output) {}

  void writef(const float* data, size_t nframes) {
    if (output_file_) {
      output_file_->writef(data, nframes);
    }
    if (save_output_) {
      output_.insert(output_.end(), data, data + nframes * frame_size());
    }
  }

  void SetWavFile(const char* fn) {
    output_file_ = std::make_unique<SndfileHandle>(
        fn, /*mode=*/SFM_WRITE, /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
        channels_, samplerate_);
  }

  void DumpSignal(FILE* f) {
    const int from = absl::GetFlag(FLAGS_plot_from);
    const int to = absl::GetFlag(FLAGS_plot_to);
    const size_t start_i = from == -1 ? 0 : from;
    const size_t end_i = to == -1 ? num_frames() : to;
    for (size_t i = start_i; i < end_i; ++i) {
      fprintf(f, "%zu %f\n", i, output_[i * frame_size()]);
    }
  }

  std::vector<std::complex<float>> output_fft() {
    size_t N = 1;
    while (N < 2 * output_.size()) N <<= 1;
    output_.resize(N);
    return FFT(output_);
  }

  void DumpFFT(FILE* f) {
    auto fft = output_fft();
    const int from = absl::GetFlag(FLAGS_plot_from);
    const int to = absl::GetFlag(FLAGS_plot_to);
    const size_t start_freq = from == -1 ? 0 : from;
    const size_t end_freq = to == -1 ? 20000 : to;
    const size_t start_i = start_freq * fft.size() / samplerate_;
    const size_t end_i = end_freq * fft.size() / samplerate_;
    for (size_t i = start_i; i < end_i; ++i) {
      fprintf(f, "%f  %f\n", i * samplerate_ * 1.0 / fft.size(),
              std::abs(fft[i]));
    }
  }

  const std::vector<float>& output() { return output_; }
  size_t channels() const { return channels_; }
  size_t frame_size() const { return channels_ * freq_channels_; }
  size_t num_frames() const { return output_.size() / frame_size(); }

 private:
  size_t channels_;
  size_t freq_channels_;
  size_t samplerate_;
  bool save_output_;
  std::vector<float> output_;
  std::unique_ptr<SndfileHandle> output_file_;
};

template <typename In, typename Out>
void Process(
    In& input_stream, Out& output_stream, FilterMode mode,
    const std::vector<float>& filter_gains = {},
    const std::function<void()>& start_progress = [] {},
    const std::function<void(int64_t)>& set_progress = [](int64_t written) {}) {
  const size_t num_channels = input_stream.channels();
  std::vector<float> history(num_channels * kHistorySize);
  std::vector<float> input(num_channels * kBlockSize);
  std::vector<float> output(output_stream.frame_size() * kBlockSize);

  RotatorFilterBank rotbank(kNumRotators, num_channels,
                            input_stream.samplerate(), /*num_threads=*/1,
                            filter_gains, absl::GetFlag(FLAGS_gain));

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
    int64_t output_len = 0;
    if (mode == IDENTITY) {
      output_len = rotbank.FilterAllSingleThreaded(
          history.data(), total_in, read, mode, output.data(), output.size());
    } else {
      output_len = rotbank.FilterAll(history.data(), total_in, read, mode,
                                     output.data(), output.size());
    }
    output_stream.writef(output.data(), output_len);
    err += SquareError(history.data(), output.data(), num_channels, total_out,
                       output_len);
    total_in += read;
    total_out += output_len;
    set_progress(total_in);
  }
  err /= total_out;
  float psnr = -10.0 * std::log(err) / std::log(10.0);
  fprintf(stdout, "score=%.15g\n", err);
  fprintf(stderr, "MSE: %f  PSNR: %f\n", err, psnr);
}

void RecomputeFilterGains(std::vector<float>& filter_gains) {
  for (int iter = 0; iter < 10000; ++iter) {
    float optsum = 0;
    InputSignal in("impulse:16384:6000:1");
    OutputSignal out(1, 1, 48000, true);
    Process(in, out, IDENTITY, filter_gains);
    auto fft = out.output_fft();
    for (size_t i = 0; i < kNumRotators; ++i) {
      const float frequency =
          BarkFreq(static_cast<float>(i) / (kNumRotators - 1));
      float scaled_f = frequency * fft.size() / 48000;
      size_t f0 = scaled_f;
      size_t f1 = f0 + 1;
      float gain = std::abs(fft[f0]) * (scaled_f - f0) +
                   std::abs(fft[f1]) * (f1 - scaled_f);
      optsum += fabs(gain - 1.0);
      filter_gains[i] /= pow(gain, 0.8 - 0.7 * iter / 10000);
      filter_gains[i] = pow(filter_gains[i], 0.9999);
    }
    std::vector<float> tmp = filter_gains;
    for (size_t i = 0; i < kNumRotators; ++i) {
      if (i >= 1 && i < kNumRotators - 1) {
        filter_gains[i] *= 0.99999;
        filter_gains[i] += 0.000005 * (tmp[i - 1] + tmp[i + 1]);
      }
      fprintf(stderr, " %f,%s", filter_gains[i], i % 4 == 3 ? "\n  " : "");
    }
    fprintf(stderr, "optsum %g\n", optsum);
  }
}

void ValueToRgb(float val, float good_threshold, float bad_threshold,
                float rgb[3]) {
  float heatmap[12][3] = {
      {0, 0, 0},       {0, 0, 1},
      {0, 1, 1},       {0, 1, 0},  // Good level
      {1, 1, 0},       {1, 0, 0},  // Bad level
      {1, 0, 1},       {0.5, 0.5, 1.0},
      {1.0, 0.5, 0.5},  // Pastel colors for the very bad quality range.
      {1.0, 1.0, 0.5}, {1, 1, 1},
      {1, 1, 1},  // Last color repeated to have a solid range of white.
  };
  if (val < good_threshold) {
    val = (val / good_threshold) * 0.3;
  } else if (val < bad_threshold) {
    val =
        0.3 + (val - good_threshold) / (bad_threshold - good_threshold) * 0.15;
  } else {
    val = 0.45 + (val - bad_threshold) / (bad_threshold * 12) * 0.5;
  }
  static const int kTableSize = sizeof(heatmap) / sizeof(heatmap[0]);
  val = std::min<float>(std::max<float>(val * (kTableSize - 1), 0.0),
                        kTableSize - 2);
  int ix = static_cast<int>(val);
  ix = std::min(std::max(0, ix), kTableSize - 2);  // Handle NaN
  float mix = val - ix;
  for (int i = 0; i < 3; ++i) {
    float v = mix * heatmap[ix + 1][i] + (1 - mix) * heatmap[ix][i];
    rgb[i] = pow(v, 0.5);
  }
}

void PhaseToRgb(float phase, float rgb[3]) {
  phase /= M_PI;
  phase += 1;
  if (phase < 1.0 / 3) {
    rgb[0] = 3 * phase;
    rgb[1] = 1 - 3 * phase;
  } else if (phase < 2.0 / 3) {
    phase -= 1.0 / 3;
    rgb[1] = 3 * phase;
    rgb[2] = 1 - 3 * phase;
  } else {
    phase -= 2.0 / 3;
    rgb[2] = 3 * phase;
    rgb[0] = 1 - 3 * phase;
  }
}

void GetPixelValue(float sample, FilterMode mode, uint8_t* out) {
  float rgb[3] = {};
  if (mode == AMPLITUDE) {
    ValueToRgb(sample, 0.01, 0.05, rgb);
  } else if (mode == PHASE) {
    PhaseToRgb(sample, rgb);
  }
  for (size_t i = 0; i < 3; ++i) {
    out[i] = std::min<int>(255, std::max<int>(0, std::round(rgb[i] * 255)));
  }
}

void CreatePlot(InputSignal& input, OutputSignal& output, FilterMode mode) {
  if (absl::GetFlag(FLAGS_ppm)) {
    const std::vector<float>& out = output.output();
    FILE* f = fopen("/tmp/result.ppm", "wb");
    size_t xsize = std::min<size_t>(output.num_frames(), 1 << 14);
    size_t ysize = output.frame_size();
    fprintf(f, "P6\n%zu %zu\n255\n", xsize, ysize);
    for (size_t y = 0; y < ysize; ++y) {
      std::vector<uint8_t> line(3 * xsize);
      for (size_t x = 0; x < xsize; ++x) {
        float sample = out[x * ysize + y];
        GetPixelValue(sample, mode, &line[3 * x]);
      }
      fwrite(line.data(), xsize, 3, f);
    }
    fclose(f);
    return;
  }
  std::vector<std::pair<std::string, std::string>> to_plot;
  if (absl::GetFlag(FLAGS_plot_input)) {
    to_plot.push_back({"/tmp/input_signal.txt", "input"});
  }
  if (absl::GetFlag(FLAGS_plot_output)) {
    const std::string& fn = "/tmp/output_signal.txt";
    FILE* f = fopen(fn.c_str(), "w");
    if (absl::GetFlag(FLAGS_plot_fft)) {
      output.DumpFFT(f);
    } else {
      output.DumpSignal(f);
    }
    fclose(f);
    to_plot.push_back({fn, "output"});
  }
  if (to_plot.empty()) {
    return;
  }
  FILE* f = fopen("/tmp/plot.txt", "w");
  fprintf(f, "set term pngcairo\n");
  fprintf(f, "set output \"plot.png\"\n");
  fprintf(f, "plot ");
  for (size_t i = 0; i < to_plot.size(); ++i) {
    fprintf(f, "\"%s\" with lines title \"%s\"%s", to_plot[i].first.c_str(),
            to_plot[i].second.c_str(),
            i + 1 < to_plot.size() ? ", \\\n     " : "\n");
  }
  fclose(f);
  system("gnuplot /tmp/plot.txt");
}

}  // namespace tabuli

using namespace tabuli;

int main(int argc, char** argv) {
  std::vector<char*> posargs = absl::ParseCommandLine(argc, argv);
  QCHECK_GE(posargs.size(), 2) << "Usage: " << argv[0] << " <input> [<output>]";
  FilterMode mode = GetFilterMode();
  InputSignal input(posargs[1]);
  size_t freq_channels = mode == IDENTITY ? 1 : kNumRotators;
  OutputSignal output(input.channels(), freq_channels, input.samplerate(),
                      absl::GetFlag(FLAGS_plot_output));
  if (posargs.size() > 2) {
    output.SetWavFile(posargs[2]);
  }

  std::vector<float> filter_gains;
  for (int i = 0; i < kNumRotators; ++i) {
    filter_gains.push_back(GetRotatorGains(i));
  }
  //  RecomputeFilterGains(filter_gains);

  Process(input, output, mode, filter_gains);
  CreatePlot(input, output, mode);
}
