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
ABSL_FLAG(bool, ppm, false, "If set, outputs ppm plot.");
ABSL_FLAG(int, plot_from, -1, "If non-negative, start plot from here.");
ABSL_FLAG(int, plot_to, -1, "If non-negative, end plot here.");
ABSL_FLAG(int, select_rot, -1, "If set, use only one rotator.");
ABSL_FLAG(double, gain, 1.0, "Global volume scaling.");
ABSL_FLAG(std::string, filter_mode, "identity", "Filter mode.");

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

constexpr int64_t kNumRotators = 128;

enum FilterMode {
  IDENTITY,
  AMPLITUDE,
  PHASE,
};

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

static const float kRotatorGains[kNumRotators] = {
  2.912138, 1.701489, 2.744206, 4.447661,
  0.494643, 3.636012, 0.203908, 0.045490,
  0.034070, 0.180307, 1.513761, 1.994389,
  1.598270, 1.158646, 0.183195, 0.167145,
  2.011196, 0.704368, 0.193714, 1.212243,
  1.268576, 0.363077, 0.925369, 1.113430,
  0.717405, 0.714828, 1.122345, 0.643973,
  0.938022, 0.861511, 0.758117, 0.986843,
  0.723425, 0.920137, 0.796517, 0.853722,
  0.893164, 0.838417, 0.802576, 0.883825,
  0.861366, 0.843076, 0.835036, 0.854449,
  0.847942, 0.829981, 0.857752, 0.829870,
  0.855910, 0.871936, 0.811830, 0.874936,
  0.829745, 0.870881, 0.826383, 0.851275,
  0.855121, 0.848116, 0.845903, 0.853037,
  0.847783, 0.843479, 0.858642, 0.848820,
  0.850666, 0.852596, 0.849117, 0.857270,
  0.843934, 0.861162, 0.856501, 0.836743,
  0.883791, 0.853619, 0.836916, 0.874427,
  0.853898, 0.848716, 0.850426, 0.864535,
  0.877663, 0.834846, 0.836647, 0.911410,
  0.851662, 0.786024, 0.930903, 0.867310,
  0.797635, 0.864158, 0.911078, 0.849298,
  0.816952, 0.870522, 0.899609, 0.841014,
  0.795122, 0.923079, 0.864671, 0.852164,
  0.834744, 0.963428, 0.881483, 0.776253,
  0.942036, 0.859765, 0.857380, 0.915745,
  0.784842, 0.969661, 0.919484, 0.879202,
  0.966006, 0.760651, 0.972971, 0.964370,
  0.967375, 1.366477, 0.861335, 0.394892,
  1.398265, 1.497949, 0.436235, 1.496855,
  0.705432, 1.205676, 1.073779, 1.451679,
};

struct Rotators {
  // Five arrays of rotators.
  // [0..1] is for rotation speed
  // [2..3] is for a unitary rotator
  // [4..5] is for 1st leaking accumulation
  // [6..7] is for 2nd leaking accumulation
  // [8..9] is for 3rd leaking accumulation
  float rot[10][kNumRotators] = { 0 };
  float window[kNumRotators];
  float gain[kNumRotators];
  int32_t delay[kNumRotators] = { 0 };
  int32_t advance[kNumRotators] = { 0 };
  int32_t max_delay_ = 0;

  int FindMedian3xLeaker(double window) {
    // Approximate filter delay. TODO: optimize this value along with gain values.
    return static_cast<int>(-2.206/log(window));
  }

  Rotators() { }
  Rotators(std::vector<double> frequency, const double sample_rate) {
    for (int i = 0; i < kNumRotators; ++i) {
      // The parameter relates to the frequency shape overlap and window length
      // of triple leaking integrator.
      float kWindow = 0.9996;
      float w40Hz = std::pow(kWindow, 128.0 / kNumRotators);  // at 40 Hz.
      window[i] = pow(w40Hz, std::max(1.0, frequency[i] / 40.0));
      delay[i] = FindMedian3xLeaker(window[i]);
      float windowM1 = 1.0f - window[i];
      max_delay_ = std::max(max_delay_, delay[i]);
      float f = frequency[i] * 2.0f * M_PI / sample_rate;
      gain[i] = absl::GetFlag(FLAGS_gain) * kRotatorGains[i] * pow(windowM1, 3.0);
      rot[0][i] = float(std::cos(f));
      rot[1][i] = float(-std::sin(f));
      rot[2][i] = sqrt(gain[i]);
      rot[3][i] = 0.0f;
    }
    for (size_t i = 0; i < kNumRotators; ++i) {
      advance[i] = max_delay_ - delay[i];
    }
  }

  void Increment(int i, float audio) {
    float tr = rot[0][i] * rot[2][i] - rot[1][i] * rot[3][i];
    float tc = rot[0][i] * rot[3][i] + rot[1][i] * rot[2][i];
    rot[2][i] = tr;
    rot[3][i] = tc;
    rot[4][i] *= window[i];
    rot[5][i] *= window[i];
    rot[6][i] *= window[i];
    rot[7][i] *= window[i];
    rot[8][i] *= window[i];
    rot[9][i] *= window[i];

    rot[4][i] += rot[2][i] * audio;
    rot[5][i] += rot[3][i] * audio;
    rot[6][i] += rot[4][i];
    rot[7][i] += rot[5][i];
    rot[8][i] += rot[6][i];
    rot[9][i] += rot[7][i];

    // TODO(jyrki): (rot[2],rot[3]) length should be maintained at 1.0
  }


  void AddAudio(int i, float audio) {
    rot[4][i] += rot[2][i] * audio;
    rot[5][i] += rot[3][i] * audio;
  }
  void OccasionallyRenormalize() {
    for (int i = 0; i < kNumRotators; ++i) {
      float norm = sqrt(gain[i] / (rot[2][i] * rot[2][i] + rot[3][i] * rot[3][i]));
      rot[2][i] *= norm;
      rot[3][i] *= norm;
    }
  }
  void IncrementAll() {
    for (int i = 0; i < kNumRotators; i++) {
      const float tr = rot[0][i] * rot[2][i] - rot[1][i] * rot[3][i];
      const float tc = rot[0][i] * rot[3][i] + rot[1][i] * rot[2][i];
      rot[2][i] = tr;
      rot[3][i] = tc;
      const float w = window[i];
      rot[4][i] *= w;
      rot[5][i] *= w;
      rot[6][i] *= w;
      rot[7][i] *= w;
      rot[8][i] *= w;
      rot[9][i] *= w;
      rot[6][i] += rot[4][i];
      rot[7][i] += rot[5][i];
      rot[8][i] += rot[6][i];
      rot[9][i] += rot[7][i];
    }
  }
  float GetSampleAll() {
    float retval = 0;
    for (int i = 0; i < kNumRotators; ++i) {
      retval += (rot[2][i] * rot[8][i] + rot[3][i] * rot[9][i]);
    }
    return retval;
  }

  double GetSample(int i, FilterMode mode = IDENTITY) const {
    return (mode == IDENTITY ?
            (rot[2][i] * rot[8][i] + rot[3][i] * rot[9][i]) :
            mode == AMPLITUDE ? std::sqrt(gain[i] * (rot[8][i] * rot[8][i] + rot[9][i] * rot[9][i])) :
            std::atan2(rot[8][i], rot[9][i]));
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
                    size_t samplerate, size_t num_threads,
                    const std::vector<double>& filter_gains) {
    num_rotators_ = num_rotators;
    num_channels_ = num_channels;
    num_threads_ = num_threads;
    rotators_.reserve(num_channels_);
    std::vector<double> freqs(num_rotators);
    for (size_t i = 0; i < num_rotators_; ++i) {
      freqs[i] = BarkFreq(static_cast<double>(i) / (num_rotators_ - 1));
    }
    if (rotators_.size() != num_channels) {
      rotators_.clear();
      for (size_t c = 0; c < num_channels; ++c) {
        rotators_.push_back(Rotators(freqs, samplerate));
      }
    }
    max_delay_ = rotators_[0].max_delay_;
    QCHECK_LE(max_delay_, kBlockSize);
    fprintf(stderr, "Rotator bank output delay: %zu\n", max_delay_);
    filter_outputs_.resize(num_rotators);
    for (std::vector<double>& output : filter_outputs_) {
      output.resize(num_channels_ * kBlockSize, 0.f);
    }
  }

  // TODO(jyrki): filter all at once in the generic case, filtering one
  // is not memory friendly in this memory tabulation.
  void FilterOne(size_t f_ix, const double* history, int64_t total_in,
                 int64_t len, FilterMode mode, double* output) {
    size_t out_ix = 0;
    for (int64_t i = 0; i < len; ++i) {
      int64_t delayed_ix = total_in + i - rotators_[0].advance[f_ix];
      size_t histo_ix = num_channels_ * (delayed_ix & kHistoryMask);
      for (size_t c = 0; c < num_channels_; ++c) {
        float delayed = history[histo_ix + c];
        rotators_[c].Increment(f_ix, delayed);
      }
      if (total_in + i >= max_delay_) {
        for (size_t c = 0; c < num_channels_; ++c) {
          output[out_ix * num_channels_ + c] = rotators_[c].GetSample(f_ix, mode);
        }
        ++out_ix;
      }
    }
  }

  int64_t FilterAllSingleThreaded(const double* history, int64_t total_in, int64_t len,
                                  FilterMode mode, double* output, size_t output_size) {
    size_t out_ix = 0;
    for (size_t c = 0; c < num_channels_; ++c) {
      rotators_[c].OccasionallyRenormalize();
    }
    for (int64_t i = 0; i < len; ++i) {
      for (size_t c = 0; c < num_channels_; ++c) {
        for (int k = 0; k < kNumRotators; ++k) {
          int64_t delayed_ix = total_in + i - rotators_[c].advance[k];
          size_t histo_ix = num_channels_ * (delayed_ix & kHistoryMask);
          float delayed = history[histo_ix + c];
          rotators_[c].AddAudio(k, delayed);
        }
        rotators_[c].IncrementAll();
      }
      if (total_in + i >= max_delay_) {
        for (size_t c = 0; c < num_channels_; ++c) {
          output[out_ix * num_channels_ + c] = rotators_[c].GetSampleAll();
        }
        ++out_ix;
      }
    }
    size_t out_len = total_in < max_delay_ ?
                     std::max<int64_t>(0, len - (max_delay_ - total_in)) : len;
    return out_len;
  }

  int64_t FilterAll(const double* history, int64_t total_in, int64_t len,
                    FilterMode mode, double* output, size_t output_size) {
    int select_rot = absl::GetFlag(FLAGS_select_rot);
    auto run = [&](size_t thread) {
      while (true) {
        size_t my_task = next_task_++;
        if (my_task >= num_rotators_) return;
        if (select_rot >= 0 && my_task != select_rot) {
          continue;
        }
        FilterOne(my_task, history, total_in, len, mode,
                  filter_outputs_[my_task].data());
      }
    };
    next_task_ = 0;
    std::vector<std::future<void>> futures;
    futures.reserve(num_threads_);
    for (size_t i = 0; i < num_threads_; ++i) {
      futures.push_back(std::async(std::launch::async, run, i));
    }
    for (size_t i = 0; i < num_threads_; ++i) {
      futures[i].get();
    }
    size_t out_len = total_in < max_delay_ ?
                     std::max<int64_t>(0, len - (max_delay_ - total_in)) : len;
    if (mode == IDENTITY || select_rot >= 0) {
      std::fill(output, output + output_size, 0);
      for (std::vector<double>& filter_output : filter_outputs_) {
        for (int i = 0; i < filter_output.size(); ++i) {
          output[i] += filter_output[i];
          filter_output[i] = 0.f;
        }
      }
    } else {
      for (size_t i = 0; i < out_len; ++i) {
        for (size_t j = 0; j < num_rotators_; ++j) {
          for (size_t c = 0; c < num_channels_; ++c) {
            size_t out_idx = (i * num_rotators_ + j) * num_channels_ + c;
            output[out_idx] =
                filter_outputs_[j][i * num_channels_ + c];
          }
        }
      }
    }
    return out_len;
  }

  size_t num_rotators_;
  size_t num_channels_;
  size_t num_threads_;
  std::vector<Rotators> rotators_;
  int64_t max_delay_;
  std::vector<std::vector<double>> filter_outputs_;
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
  OutputSignal(size_t channels, size_t freq_channels, size_t samplerate,
               bool save_output)
      : channels_(channels), freq_channels_(freq_channels),
        samplerate_(samplerate), save_output_(save_output) {
  }

  void writef(const double* data, size_t nframes) {
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

  std::vector<std::complex<double>> output_fft() {
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

  const std::vector<double>& output() { return output_; }
  size_t channels() const { return channels_; }
  size_t frame_size() const { return channels_ * freq_channels_; }
  size_t num_frames() const { return output_.size() / frame_size(); }

 private:
  size_t channels_;
  size_t freq_channels_;
  size_t samplerate_;
  bool save_output_;
  std::vector<double> output_;
  std::unique_ptr<SndfileHandle> output_file_;
};

template <typename In, typename Out>
void Process(
    In& input_stream, Out& output_stream, FilterMode mode,
    const std::vector<double>& filter_gains = {},
    const std::function<void()>& start_progress = [] {},
    const std::function<void(int64_t)>& set_progress = [](int64_t written) {}) {
  const size_t num_channels = input_stream.channels();
  std::vector<double> history(num_channels * kHistorySize);
  std::vector<double> input(num_channels * kBlockSize);
  std::vector<double> output(output_stream.frame_size() * kBlockSize);

  RotatorFilterBank rotbank(kNumRotators, num_channels,
                            input_stream.samplerate(), /*num_threads=*/1,
                            filter_gains);

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
      output_len = rotbank.FilterAllSingleThreaded(history.data(), total_in, read, mode,
                                                   output.data(), output.size());
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
  double psnr = -10.0 * std::log(err) / std::log(10.0);
  fprintf(stderr, "MSE: %f  PSNR: %f\n", err, psnr);
}

void RecomputeFiltrGains(std::vector<double>& filter_gains) {
  for (int iter = 0; iter < 20; ++iter) {
    InputSignal in("impulse:10000:6000:1");
    OutputSignal out(1, 1, 48000, true);
    Process(in, out, IDENTITY, filter_gains);
    auto fft = out.output_fft();
    for (size_t i = 0; i < kNumRotators; ++i) {
      const double frequency =
          BarkFreq(static_cast<double>(i) / (kNumRotators - 1));
      double scaled_f = frequency * fft.size() / 48000;
      size_t f0 = scaled_f;
      size_t f1 = f0 + 1;
      double gain = std::abs(fft[f0]) * (scaled_f - f0) +
                    std::abs(fft[f1]) * (f1 - scaled_f);
      filter_gains[i] /= gain;
      fprintf(stderr, " %f,%s", filter_gains[i], i % 4 == 3 ? "\n  " : "");
    }
  }
}

void ValueToRgb(double val, double good_threshold, double bad_threshold,
                float rgb[3]) {
  double heatmap[12][3] = {
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
    val = 0.3 +
            (val - good_threshold) / (bad_threshold - good_threshold) * 0.15;
  } else {
    val = 0.45 + (val - bad_threshold) / (bad_threshold * 12) * 0.5;
  }
  static const int kTableSize = sizeof(heatmap) / sizeof(heatmap[0]);
  val = std::min<double>(std::max<double>(val * (kTableSize - 1), 0.0),
                           kTableSize - 2);
  int ix = static_cast<int>(val);
  ix = std::min(std::max(0, ix), kTableSize - 2);  // Handle NaN
  double mix = val - ix;
  for (int i = 0; i < 3; ++i) {
    double v = mix * heatmap[ix + 1][i] + (1 - mix) * heatmap[ix][i];
    rgb[i] = pow(v, 0.5);
  }
}

void PhaseToRgb(double phase, float rgb[3]) {
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

void GetPixelValue(double sample, FilterMode mode, uint8_t* out) {
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
    const std::vector<double>& out = output.output();
    FILE* f = fopen("/tmp/result.ppm", "wb");
    size_t xsize = std::min<size_t>(output.num_frames(), 1 << 14);
    size_t ysize = output.frame_size();
    fprintf(f, "P6\n%zu %zu\n255\n", xsize, ysize);
    for (size_t y = 0; y < ysize; ++y) {
      std::vector<uint8_t> line(3 * xsize);
      for (size_t x = 0; x < xsize; ++x) {
        double sample = out[x * ysize + y];
        GetPixelValue(sample, mode, &line[3 * x]);
      }
      fwrite(line.data(), xsize, 3, f);
    }
    fclose(f);
    return;
  }
  std::vector<std::pair<std::string, std::string> > to_plot;
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
    to_plot.push_back({fn,"output"});
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
  FilterMode mode = GetFilterMode();
  InputSignal input(posargs[1]);
  size_t freq_channels =
      mode == IDENTITY || absl::GetFlag(FLAGS_select_rot) >= 0 ?
      1 : kNumRotators;
  OutputSignal output(input.channels(), freq_channels, input.samplerate(),
                      absl::GetFlag(FLAGS_plot_output));
  if (posargs.size() > 2) {
    output.SetWavFile(posargs[2]);
  }

  std::vector<double> filter_gains(kRotatorGains, kRotatorGains + kNumRotators);
  Process(input, output, mode, filter_gains);
  CreatePlot(input, output, mode);
}
