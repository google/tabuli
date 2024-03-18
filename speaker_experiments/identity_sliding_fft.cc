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
  auto butterfly = [](const std::complex<float>& m,
                      std::complex<float>& a, std::complex<float>& b) {
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

constexpr int64_t kNumRotators = 128;

float GetRotatorGains(int i) {
  static const float kRotatorGains[kNumRotators] = {
    1.050645, 1.948438, 3.050339, 3.967913,
    4.818584, 5.303335, 5.560281, 5.490826,
    5.156689, 4.547374, 3.691308, 2.666868,
    1.539254, 0.656948, 0.345893, 0.327111,
    0.985318, 1.223506, 0.447645, 0.830961,
    1.075181, 0.613335, 0.902695, 0.855391,
    0.817774, 0.823359, 0.841483, 0.838562,
    0.831912, 0.808731, 0.865214, 0.808036,
    0.850837, 0.821305, 0.839458, 0.829195,
    0.836373, 0.827271, 0.836018, 0.834514,
    0.825624, 0.836999, 0.833990, 0.832992,
    0.830897, 0.832593, 0.846116, 0.824796,
    0.829331, 0.844509, 0.838830, 0.821733,
    0.840738, 0.841735, 0.827570, 0.838581,
    0.837742, 0.834965, 0.842970, 0.832145,
    0.847596, 0.840942, 0.830891, 0.850632,
    0.841468, 0.838383, 0.841493, 0.855118,
    0.826750, 0.848000, 0.874356, 0.812177,
    0.849037, 0.893550, 0.832527, 0.827986,
    0.877198, 0.851760, 0.846317, 0.883044,
    0.843178, 0.856925, 0.857045, 0.860695,
    0.894345, 0.870391, 0.839519, 0.870541,
    0.870573, 0.902951, 0.871798, 0.818328,
    0.871413, 0.921101, 0.863915, 0.793014,
    0.936519, 0.888107, 0.856968, 0.821018,
    0.987345, 0.904846, 0.783447, 0.973613,
    0.903628, 0.875688, 0.931024, 0.992087,
    0.806914, 1.050332, 0.942569, 0.800870,
    1.210426, 0.916555, 0.817352, 1.126946,
    0.985119, 0.922530, 0.994633, 0.959602,
    0.381419, 1.879201, 2.078451, 0.475196,
    0.952731, 1.709305, 1.383894, 1.557669,
  };
  return kRotatorGains[i];
}

struct PerChannel {
  // [0..1] is for real and imag of 1st leaking accumulation
  // [2..3] is for real and imag of 2nd leaking accumulation
  // [4..5] is for real and imag of 3rd leaking accumulation
  float accu[6][kNumRotators] = { 0 };
};

struct Rotators {
  // Four arrays of rotators.
  // [0..1] is real and imag for rotation speed
  // [2..3] is real and image for a frequency rotator of length sqrt(gain[i])
  // Values inserted into the rotators are multiplied with this rotator in both input
  // and output, leading to a total gain multiplication if the length is at sqrt(gain).
  float rot[4][kNumRotators] = { 0 };
  std::vector<PerChannel> channel;
  // Accu has the channel related data, everything else the same between channels.
  float window[kNumRotators];
  float gain[kNumRotators];
  int16_t delay[kNumRotators] = { 0 };
  int16_t advance[kNumRotators] = { 0 };
  int16_t max_delay_ = 0;

  int FindMedian3xLeaker(float window) {
    // Approximate filter delay. TODO: optimize this value along with gain values.
    // Recordings can sound better with -2.32 as it pushes the bass signals a bit earlier
    // and likely compensates human hearing's deficiency for temporal separation.
    const float kMagic = -2.2028003503591482;
    const float kAlmostHalfForRounding = 0.4687;
    return static_cast<int>(kMagic/log(window) + kAlmostHalfForRounding);
  }

  Rotators() { }
  Rotators(int num_channels, std::vector<float> frequency,
           std::vector<float> filter_gains, const float sample_rate) {
    channel.resize(num_channels);
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
      gain[i] = filter_gains[i] * absl::GetFlag(FLAGS_gain) * pow(windowM1, 3.0);
      rot[0][i] = float(std::cos(f));
      rot[1][i] = float(-std::sin(f));
      rot[2][i] = sqrt(gain[i]);
      rot[3][i] = 0.0f;
    }
    for (size_t i = 0; i < kNumRotators; ++i) {
      advance[i] = max_delay_ - delay[i];
    }
  }

  void Increment(int c, int i, float audio) {
    if (c == 0) {
      float tr = rot[0][i] * rot[2][i] - rot[1][i] * rot[3][i];
      float tc = rot[0][i] * rot[3][i] + rot[1][i] * rot[2][i];
      rot[2][i] = tr;
      rot[3][i] = tc;
    }
    channel[c].accu[0][i] *= window[i];
    channel[c].accu[1][i] *= window[i];
    channel[c].accu[2][i] *= window[i];
    channel[c].accu[3][i] *= window[i];
    channel[c].accu[4][i] *= window[i];
    channel[c].accu[5][i] *= window[i];
    channel[c].accu[0][i] += rot[2][i] * audio;
    channel[c].accu[1][i] += rot[3][i] * audio;
    channel[c].accu[2][i] += channel[c].accu[0][i];
    channel[c].accu[3][i] += channel[c].accu[1][i];
    channel[c].accu[4][i] += channel[c].accu[2][i];
    channel[c].accu[5][i] += channel[c].accu[3][i];
  }

  void AddAudio(int c, int i, float audio) {
    channel[c].accu[0][i] += rot[2][i] * audio;
    channel[c].accu[1][i] += rot[3][i] * audio;
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
    }
    for (int c = 0; c < channel.size(); ++c) {
      for (int i = 0; i < kNumRotators; i++) {
        const float w = window[i];
        channel[c].accu[0][i] *= w;
        channel[c].accu[1][i] *= w;
        channel[c].accu[2][i] *= w;
        channel[c].accu[3][i] *= w;
        channel[c].accu[4][i] *= w;
        channel[c].accu[5][i] *= w;
        channel[c].accu[2][i] += channel[c].accu[0][i];
        channel[c].accu[3][i] += channel[c].accu[1][i];
        channel[c].accu[4][i] += channel[c].accu[2][i];
        channel[c].accu[5][i] += channel[c].accu[3][i];
      }
    }
  }
  float GetSampleAll(int c) {
    float retval = 0;
    for (int i = 0; i < kNumRotators; ++i) {
      retval += (rot[2][i] * channel[c].accu[4][i] + rot[3][i] * channel[c].accu[5][i]);
    }
    return retval;
  }
  float GetSample(int c, int i, FilterMode mode = IDENTITY) const {
    return (mode == IDENTITY ?
            (rot[2][i] * channel[c].accu[4][i] + rot[3][i] * channel[c].accu[5][i]) :
            mode == AMPLITUDE ? std::sqrt(gain[i] * (channel[c].accu[4][i] * channel[c].accu[4][i] +
                                                     channel[c].accu[5][i] * channel[c].accu[5][i])) :
            std::atan2(channel[c].accu[4][i], channel[c].accu[5][i]));
  }
};

float BarkFreq(float v) {
  constexpr float linlogsplit = 0.1;
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

float HardClip(float v) {
  return std::max(-1.0f, std::min(1.0f, v));
}

struct RotatorFilterBank {
  RotatorFilterBank(size_t num_rotators, size_t num_channels,
                    size_t samplerate, size_t num_threads,
                    const std::vector<float>& filter_gains) {
    num_rotators_ = num_rotators;
    num_channels_ = num_channels;
    num_threads_ = num_threads;
    std::vector<float> freqs(num_rotators);
    for (size_t i = 0; i < num_rotators_; ++i) {
      freqs[i] = BarkFreq(static_cast<float>(i) / (num_rotators_ - 1));
      // printf("%d %g\n", i, freqs[i]);
    }
    rotators_ = new Rotators(num_channels, freqs, filter_gains, samplerate);

    max_delay_ = rotators_[0].max_delay_;
    QCHECK_LE(max_delay_, kBlockSize);
    fprintf(stderr, "Rotator bank output delay: %zu\n", max_delay_);
    filter_outputs_.resize(num_rotators);
    for (std::vector<float>& output : filter_outputs_) {
      output.resize(num_channels_ * kBlockSize, 0.f);
    }
  }
  ~RotatorFilterBank() {
    delete rotators_;
  }

  // TODO(jyrki): filter all at once in the generic case, filtering one
  // is not memory friendly in this memory tabulation.
  void FilterOne(size_t f_ix, const float* history, int64_t total_in,
                 int64_t len, FilterMode mode, float* output) {
    size_t out_ix = 0;
    for (int64_t i = 0; i < len; ++i) {
      int64_t delayed_ix = total_in + i - rotators_->advance[f_ix];
      size_t histo_ix = num_channels_ * (delayed_ix & kHistoryMask);
      for (size_t c = 0; c < num_channels_; ++c) {
        float delayed = history[histo_ix + c];
        rotators_->Increment(c, f_ix, delayed);
      }
      if (total_in + i >= max_delay_) {
        for (size_t c = 0; c < num_channels_; ++c) {
          output[out_ix * num_channels_ + c] = rotators_->GetSample(c, f_ix, mode);
        }
        ++out_ix;
      }
    }
  }

  int64_t FilterAllSingleThreaded(const float* history, int64_t total_in, int64_t len,
                                  FilterMode mode, float* output, size_t output_size) {
    size_t out_ix = 0;
    for (size_t c = 0; c < num_channels_; ++c) {
      rotators_->OccasionallyRenormalize();
    }
    for (int64_t i = 0; i < len; ++i) {
      for (size_t c = 0; c < num_channels_; ++c) {
        for (int k = 0; k < kNumRotators; ++k) {
          int64_t delayed_ix = total_in + i - rotators_->advance[k];
          size_t histo_ix = num_channels_ * (delayed_ix & kHistoryMask);
          float delayed = history[histo_ix + c];
          rotators_->AddAudio(c, k, delayed);
        }
      }
      rotators_->IncrementAll();
      if (total_in + i >= max_delay_) {
        for (size_t c = 0; c < num_channels_; ++c) {
          output[out_ix * num_channels_ + c] = HardClip(rotators_->GetSampleAll(c));
        }
        ++out_ix;
      }
    }
    size_t out_len = total_in < max_delay_ ?
                     std::max<int64_t>(0, len - (max_delay_ - total_in)) : len;
    return out_len;
  }

  int64_t FilterAll(const float* history, int64_t total_in, int64_t len,
                    FilterMode mode, float* output, size_t output_size) {
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
      for (std::vector<float>& filter_output : filter_outputs_) {
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
  Rotators *rotators_;
  int64_t max_delay_;
  std::vector<std::vector<float>> filter_outputs_;
  std::atomic<size_t> next_task_{0};
};

float SquareError(const float* input_history, const float* output,
                   size_t num_channels, size_t total, size_t output_len) {
  float res = 0.0;
  for (size_t i = 0; i < output_len; ++i) {
    int input_ix = i + total;
    size_t histo_ix = num_channels * (input_ix & kHistoryMask);
    for (size_t c = 0; c < num_channels; ++ c) {
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
      : channels_(channels), freq_channels_(freq_channels),
        samplerate_(samplerate), save_output_(save_output) {
  }

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
    val = 0.3 +
            (val - good_threshold) / (bad_threshold - good_threshold) * 0.15;
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

  std::vector<float> filter_gains;
  for (int i = 0; i < kNumRotators; ++i) {
    filter_gains.push_back(GetRotatorGains(i));
  }
  //  RecomputeFilterGains(filter_gains);

  Process(input, output, mode, filter_gains);
  CreatePlot(input, output, mode);
}
