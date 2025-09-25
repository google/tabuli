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
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "absl/log/check.h"
#include "sndfile.hh"

#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <memory>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

static const int kHistorySize = (1 << 18);
static const int kHistoryMask = kHistorySize - 1;
static constexpr int64_t kBlockSize = 1 << 15;

namespace tabuli {

int FindMedian3xLeaker(float window) {
  // Approximate filter delay. TODO: optimize this value along with gain values.
  // Recordings can sound better with -2.32 as it pushes the bass signals a bit
  // earlier and likely compensates human hearing's deficiency for temporal
  // separation.
  const float kMagic = -12.5;
  const float kAlmostHalfForRounding = 0.4687;
  return static_cast<int>(kMagic / log(window) + kAlmostHalfForRounding);
}

// Expected signal sample rate.
constexpr float kSampleRate = 48000;
constexpr int64_t kNumRotators = 128;

// Computes dot product of two 32-element float arrays.
// Optimized for SIMD vectorization with -ffast-math.
inline float Dot32(const float* a, const float* b) {
  // -ffast-math is helpful here, and clang can simdify this.
  float sum = 0;
  for (int i = 0; i < 32; ++i) sum += a[i] * b[i];
  return sum;
}

// Returns the center frequency in Hz for filter bank channel i.
// The 128 channels are spaced to match human auditory perception,
// with finer resolution at lower frequencies.
float Freq(int i) {
  // Center frequencies of the filter bank, plus one frequency in both ends.
  static const float kFreq[130] = {
      17.858,  24.349,  33.199,  42.359,  51.839,  61.651,  71.805,  82.315,
      93.192,  104.449, 116.099, 128.157, 140.636, 153.552, 166.919, 180.754,
      195.072, 209.890, 225.227, 241.099, 257.527, 274.528, 292.124, 310.336,
      329.183, 348.690, 368.879, 389.773, 411.398, 433.778, 456.941, 480.914,
      505.725, 531.403, 557.979, 585.484, 613.950, 643.411, 673.902, 705.459,
      738.119, 771.921, 806.905, 843.111, 880.584, 919.366, 959.503, 1001.04,
      1044.03, 1088.53, 1134.58, 1182.24, 1231.57, 1282.62, 1335.46, 1390.14,
      1446.73, 1505.31, 1565.93, 1628.67, 1693.60, 1760.80, 1830.35, 1902.34,
      1976.84, 2053.94, 2133.74, 2216.33, 2301.81, 2390.27, 2481.83, 2576.58,
      2674.65, 2776.15, 2881.19, 2989.91, 3102.43, 3218.88, 3339.40, 3464.14,
      3593.23, 3726.84, 3865.12, 4008.23, 4156.35, 4309.64, 4468.30, 4632.49,
      4802.43, 4978.31, 5160.34, 5348.72, 5543.70, 5745.49, 5954.34, 6170.48,
      6394.18, 6625.70, 6865.32, 7113.31, 7369.97, 7635.61, 7910.53, 8195.06,
      8489.53, 8794.30, 9109.73, 9436.18, 9774.04, 10123.7, 10485.6, 10860.1,
      11247.8, 11648.9, 12064.2, 12493.9, 12938.7, 13399.0, 13875.3, 14368.4,
      14878.7, 15406.8, 15953.4, 16519.1, 17104.5, 17710.4, 18337.6, 18986.6,
      19658.3, 20352.7,
  };
  return kFreq[i + 1];
}

float FreqAve(int i) {
  return sqrt(Freq(i - 1) * Freq(i + 1));
}

// Calculates the effective bandwidth in Hz for filter bank channel i.
// Uses geometric mean spacing between adjacent channels.
double CalculateBandwidthInHz(int i) {
  return 0.5 * (Freq(i + 1) - std::sqrt(Freq(i - 1)));
  //  return std::sqrt(Freq(i + 1) * Freq(i)) - std::sqrt(Freq(i - 1) * Freq(i));
}

struct PerChannel {
  // [0..1] is for real and imag of 1st leaking accumulation
  // [2..3] is for real and imag of 2nd leaking accumulation
  // [4..5] is for real and imag of 3rd leaking accumulation
  float accu[20][kNumRotators] = {0};
};

// Core signal processing engine using rotating phasors (Goertzel-like
// algorithm) for efficient frequency analysis. Implements the Zimtohrli/Tabuli
// filterbank.
class Rotators {
 private:
  // Four arrays of rotators, with memory layout for up to 128-way
  // simd-parallel. [0..1] is real and imag for rotation speed [2..3] is real
  // and image for a frequency rotator of length sqrt(gain[i])
  float rot[4][kNumRotators];
  // [0..1] is for real and imag of 1st leaking accumulation
  // [2..3] is for real and imag of 2nd leaking accumulation
  // [4..5] is for real and imag of 3rd leaking accumulation
  std::vector<PerChannel> channel;
  float window[kNumRotators];
  float gain[kNumRotators];
  int16_t delay[kNumRotators] = {0};
  int16_t advance[kNumRotators] = {0};
  int16_t max_delay_ = 0;
  int num_channels;

  // Renormalizes the rotating phasors to prevent numerical drift.
  // Called periodically during signal processing.
  void OccasionallyRenormalize() {
    for (int i = 0; i < kNumRotators; ++i) {
      float norm =
          gain[i] / sqrt(rot[2][i] * rot[2][i] + rot[3][i] * rot[3][i]);
      rot[2][i] *= norm;
      rot[3][i] *= norm;
    }
  }
  void AddAudio(int c, int i, float audio) {
    channel[c].accu[0][i] += rot[2][i] * audio;
    channel[c].accu[1][i] += rot[3][i] * audio;
  }
  // Updates all rotators and accumulators with a new signal sample.
  // Applies windowing, rotates phasors, and accumulates energy.
  void IncrementAll() {
    for (int c = 0; c < channel.size(); ++c) {
      for (int i = 0; i < kNumRotators; i++) {  // clang simdifies this.
	const float w = window[i];
	for (int k = 0; k < 20; ++k) channel[c].accu[k][i] *= w;
	for (int k = 2; k < 20; ++k) {
	  channel[c].accu[k][i] += channel[c].accu[k - 2][i];
	}
	const float a = rot[2][i], b = rot[3][i];
	rot[2][i] = rot[0][i] * a - rot[1][i] * b;
	rot[3][i] = rot[0][i] * b + rot[1][i] * a;
      }
    }
  }

 public:
  // Main signal processing function that converts time-domain audio to a
  // perceptual spectrogram. Applies resonator filtering, frequency analysis
  // via rotating phasors, and downsampling.
  // in: input audio samples
  // in_size: number of input samples
  // out: output spectrogram buffer
  // out_shape0: number of time steps in output
  // out_stride: stride between time steps in output buffer
  // downsample: downsampling factor
  void Init(int channels) {
    channel.resize(channels);
    num_channels = channels;
    static int downsample = 480;
    static const float kSampleRate = 48000.0;
    static const float kHzToRad = 2.0f * M_PI / kSampleRate;
    static const double kWindow = 0.9996073584827937;
    // A big value for normalization.
    static const double kScale = 2e2;
    const float gainer = sqrt(kScale / downsample);
    for (int i = 0; i < kNumRotators; ++i) {
      float bandwidth = CalculateBandwidthInHz(i);  // bandwidth per bucket.
      window[i] = std::pow(kWindow, bandwidth * 0.82);
      delay[i] = FindMedian3xLeaker(window[i]);
      max_delay_ = std::max(max_delay_, delay[i]);
      float windowM1 = 1.0f - window[i];
      const float f = FreqAve(i) * kHzToRad;
      gain[i] = gainer * pow(windowM1, 5.0);
      rot[0][i] = float(std::cos(f));
      rot[1][i] = float(-std::sin(f));
      rot[2][i] = gain[i];
      rot[3][i] = 0.0f;
    }
    for (size_t i = 0; i < kNumRotators; ++i) {
      advance[i] = max_delay_ - delay[i];
    }
    std::vector<float> downsample_window(downsample);
    for (int i = 0; i < downsample; ++i) {
      downsample_window[i] =
          1.0 / (1.0 + exp(8.0246040186567118 * ((2.0 / downsample) * (i + 0.5) - 1)));
    }
  }
  float HardClip(float v) { return std::max(-1.0f, std::min(1.0f, v)); }
  float GetSampleAll(int c) {
    float retval = 0;
    for (int i = 0; i < 128; ++i) {
      float a0 = channel[c].accu[18][i];
      float a1 = channel[c].accu[19][i];
      retval += (rot[2][i] * a0 + rot[3][i] * a1);
    }
    return retval;
  }

  int64_t FilterAllSingleThreaded(const float *history,
				  int64_t total_in,
				  int64_t len,
				  float *output,
				  size_t output_size) {
    size_t out_ix = 0;
    OccasionallyRenormalize();
    for (int64_t i = 0; i < len; ++i) {
      for (size_t c = 0; c < channel.size(); ++c) {
	for (int k = 0; k < kNumRotators; ++k) {
	  int64_t delayed_ix = total_in + i - advance[k];
	  size_t histo_ix = num_channels * (delayed_ix & kHistoryMask);
	  float delayed = history[histo_ix + c];
	  AddAudio(c, k, delayed);
	}
      }
      IncrementAll();
      if (total_in + i >= max_delay_) {
	for (size_t c = 0; c < num_channels; ++c) {
	  output[out_ix * num_channels + c] =
	      HardClip(GetSampleAll(c));
	}
	++out_ix;
      }
    }
    size_t out_len = total_in < max_delay_
      ? std::max<int64_t>(0, len - (max_delay_ - total_in))
      : len;
    return out_len;
  }
};



class InputSignal {
 public:
  InputSignal(const char *path) {
    input_file_ = std::make_unique<SndfileHandle>(path);
    if (!*input_file_) {
      fprintf(stderr, "cannot open input path: %s\n", path);
      exit(2);
    }
    channels_ = input_file_->channels();
    samplerate_ = input_file_->samplerate();
  }

  size_t channels() const { return channels_; }
  size_t samplerate() const { return samplerate_; }

  int64_t readf(float* data, size_t nframes) {
    int64_t read = input_file_->readf(data, nframes);
    return read;
  }

 private:
  size_t channels_;
  size_t samplerate_;
  std::unique_ptr<SndfileHandle> input_file_;
};

class OutputSignal {
 public:
  OutputSignal(size_t channels, size_t freq_channels, size_t samplerate)
    : channels_(channels),
      freq_channels_(freq_channels),
      samplerate_(samplerate) { }

  void writef(const float* data, size_t nframes) {
    if (output_file_) {
      output_file_->writef(data, nframes);
    }
  }

  void SetWavFile(const char* fn) {
    output_file_ = std::make_unique<SndfileHandle>(
        fn, /*mode=*/SFM_WRITE, /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
        channels_, samplerate_);
  }
  
  size_t channels() const { return channels_; }
  size_t frame_size() const { return channels_ * freq_channels_; }

 private:
  size_t channels_;
  size_t freq_channels_;
  size_t samplerate_;
  std::unique_ptr<SndfileHandle> output_file_;
};

template <typename In, typename Out>
void Process(
    In& input_stream, Out& output_stream,
    const std::vector<float>& filter_gains = {},
    const std::function<void()>& start_progress = [] {},
    const std::function<void(int64_t)>& set_progress = [](int64_t written) {}) {
  const size_t num_channels = input_stream.channels();
  std::vector<float> history(num_channels * kHistorySize);
  std::vector<float> input(num_channels * kBlockSize);
  std::vector<float> output(output_stream.frame_size() * kBlockSize);
  
  Rotators rotbank;
  rotbank.Init(num_channels);
  
  start_progress();
  int64_t total_in = 0;
  int64_t total_out = 0;
  bool done = false;
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
    output_len = rotbank.FilterAllSingleThreaded(history.data(), total_in, read, output.data(), output.size());
    output_stream.writef(output.data(), output_len);
    total_in += read;
    total_out += output_len;
    set_progress(total_in);
  }
};

}  // namespace tabuli

using namespace tabuli;

int main(int argc, char** argv) {
  if(argc < 3) {
    fprintf(stderr, "Usage: %s in.wav out.wav\n", argv[0]);
    exit(1);
  }
  InputSignal input(argv[1]);
  OutputSignal output(input.channels(), kNumRotators, input.samplerate());
  output.SetWavFile(argv[2]);
  std::vector<float> filter_gains;
  //  for (int i = 0; i < kNumRotators; ++i) {
  //    filter_gains.push_back(GetRotatorGains(i));
  //  }
  Process(input, output, filter_gains);
}
