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

static constexpr int64_t kBlockSize = 1 << 18;
static constexpr int64_t kHistorySize = 1 << 19; // needs to bigger than kBlockSize
static constexpr int64_t kHistoryMask = kHistorySize - 1;

namespace tabuli {

double FindMedian3xLeaker(double window) {
  // Approximate filter delay.
  static const double kMagic = 5.46;
  return kMagic / window;
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

    18.949270513107507, 20.0, 21.10899201757206, 22.279477199896046, 23.51486531841422, 24.818755215034393, 26.194945286011798, 27.647444547157992, 29.180484312611213, 30.798530521189843, 32.50629674623729, 34.30875792685758, 36.21116486054244, 38.219059499408814, 40.33829109460661, 42.57503323592745, 44.93580183625289, 47.4274741132331, 50.05730862349216, 52.83296640772185, 55.76253330826266, 58.85454352418564, 62.1180044724941, 65.56242302786917, 69.19783321739865, 73.03482545096755, 77.08457737246212, 81.35888642166094, 85.87020420166964, 90.63167275201636, 95.65716283307582, 100.9613143333494, 106.55957891731282, 112.46826503806983, 118.70458544593973, 125.2867073313771, 132.2338052482963, 139.56611697197326, 147.3050024542456, 155.47300604775518, 164.09392218049982, 173.1928646720131, 182.79633989309812, 192.93232398223984, 203.63034434363638, 214.92156566426348, 226.83888070055136, 239.41700609914596, 252.6925835308937, 266.7042864326648, 281.49293266796866, 297.10160343455505, 313.57576876539406, 330.9634199886363, 349.31520953242364, 368.68459848182215, 389.1280123177271, 410.7050052914292, 433.4784339136835, 457.5146400636791, 482.88364425132784, 509.65934959586934, 537.9197571150087, 567.747192951751, 599.2285482008728, 632.4555320336758, 667.5249388584077, 704.5389302946203, 743.605332782895, 784.8379516969072, 828.356902872881, 874.2889625222178, 922.7679365466429, 973.9350503317262, 1027.9393601543027, 1084.9381874022652, 1145.0975768716758, 1208.5927804762655, 1275.6087677784355, 1346.3407648289965, 1420.9948228853573, 1499.7884186649117, 1582.951087882232, 1670.7250939156525, 1763.3661335511417, 1861.1440818593978, 1964.3437783760758, 2073.265856875396, 2188.2276211543713, 2309.563969378916, 2437.6283696845794, 2572.793889873949, 2715.454284210368, 2866.0251404739256, 3024.945090621247, 3192.677088575884, 3369.7097588716774, 3556.5588200778457, 3753.7685871524372, 3961.9135571006773, 4181.6000825574365, 4413.46813816918, 4658.193184921091, 4916.488137840395, 5189.105442808031, 5476.839268528722, 5780.5278200449, 6101.055780534051, 6439.356888502758, 6796.416657885119, 7173.275248969538, 7571.030498517261, 7990.841117899773, 8433.930068571646, 8901.58812471199, 9395.177633412985, 9916.136483369311, 10465.982293629893, 11046.316834614197, 11658.830694272148, 12305.308202980745, 12987.632631524228, 13707.791677300165, 14467.883254733495, 15270.12160676669, 16116.843755229636, 17010.516308879927, 17953.742648946285, 18949.27051310751, 20000.0, 
/*
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
    */
  };
  return kFreq[i + 1];
}

float CalcPhaseDiff(float delay_diff, float freq) {
  float cycles = delay_diff * freq / 48000.0;
  return 2 * M_PI * cycles;
}


float FreqAve(int i) {
  return sqrt(Freq(i - 1) * Freq(i + 1));
}

// Calculates the effective bandwidth in Hz for filter bank channel i.
// Uses geometric mean spacing between adjacent channels.
double CalculateBandwidthInHz(int i) {
  return 0.5 * (Freq(i + 1) - Freq(i - 1));
  //  return std::sqrt(Freq(i + 1) * Freq(i)) - std::sqrt(Freq(i - 1) * Freq(i));
}

struct PerChannel {
  // [0..1] is for real and imag of 1st leaking accumulation
  // [2..3] is for real and imag of 2nd leaking accumulation
  // [4..5] is for real and imag of 3rd leaking accumulation
  double accu[16][kNumRotators] = {0};
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
	for (int k = 2; k < 16; ++k) {
	  channel[c].accu[k][i] += channel[c].accu[k - 2][i];
	}
	const float a = rot[2][i], b = rot[3][i];
	rot[2][i] = rot[0][i] * a - rot[1][i] * b;
	rot[3][i] = rot[0][i] * b + rot[1][i] * a;
	for (int k = 0; k < 16; ++k) channel[c].accu[k][i] *= w;
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
    static const float kSampleRate = 48000.0;
    static const float kHzToRad = 2.0f * M_PI / kSampleRate;
    static const double kWindow = 0.99964;
    // A big value for normalization.
    float glut = 1.0;
    for (int i = 0; i < kNumRotators; ++i) {
      float bandwidth = CalculateBandwidthInHz(i);  // bandwidth per bucket.
      window[i] = std::pow(kWindow, bandwidth);
      delay[i] = FindMedian3xLeaker(1-window[i]);

      max_delay_ = std::max(max_delay_, delay[i]);

      const float f = FreqAve(i) * kHzToRad;
      float windowM1 = 1.0f - window[i];
      gain[i] = pow(windowM1, 4.0) * glut;
      rot[0][i] = float(std::cos(f));
      rot[1][i] = float(-std::sin(f));
      rot[2][i] = gain[i];
      rot[3][i] = 0.0f;
    }
    for (size_t i = 0; i < kNumRotators; ++i) {
      advance[i] = 1 + max_delay_ - delay[i];
      advance[i] = std::round(advance[i]);
    }
    /*
    const ind downsample = 480;
    std::vector<float> downsample_window(downsample);
    for (int i = 0; i < downsample; ++i) {
      downsample_window[i] =
          1.0 / (1.0 + exp(8.0246040186567118 * ((2.0 / downsample) * (i + 0.5) - 1)));
    }
    */
  }
  float HardClip(float v) { return std::max(-1.0f, std::min(1.0f, v)); }
  float GetSampleAll(int c) {
    float retval = 0;
    for (int i = 0; i < 128; ++i) {
      float a0 = channel[c].accu[14][i];
      float a1 = channel[c].accu[15][i];
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
    return out_ix;
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
  OutputSignal(size_t channels, size_t samplerate)
    : channels_(channels),
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
  size_t frame_size() const { return channels_; }

 private:
  size_t channels_;
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
    printf("reading %d -> %d\n", kBlockSize, read);
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
  OutputSignal output(input.channels(), input.samplerate());
  output.SetWavFile(argv[2]);
  std::vector<float> filter_gains;
  Process(input, output, filter_gains);
}
