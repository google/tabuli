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
#include <cstdlib>
#include <functional>
#include <future>  // NOLINT
#include <sndfile.hh>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/check.h"

ABSL_FLAG(int, output_channels, 16, "number of output channels");
ABSL_FLAG(double, distance_to_interval_ratio, 8,
          "ratio of (distance between microphone and source array) / (distance "
          "between each source); default = 40cm / 10cm = 4");


namespace {

constexpr int kSubSourcePrecision = 1000;

float MicrophoneResponse(const float angle) {
  return 0.5f * (1.25f + std::cos(angle));
}

float ExpectedLeftToRightRatio(const float angle) {
  return (1e-3 + MicrophoneResponse(angle + M_PI / 4)) /
         (1e-3 + MicrophoneResponse(angle - M_PI / 4));
}

float ActualLeftToRightRatio(float left, float right) {
  return std::sqrt((1e-13 + left) / (1e-13 + right));
}

// Fast approximate function for temporal coherence of 3x exp leakers.
float FindMedian3xLeaker(float window) {
  return -2.32/log(window);
}

constexpr int64_t kNumRotators = 128;

float GetFilterGains(int i) {
  static const float kFilterGains[kNumRotators] = {
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
  return kFilterGains[i];
}

struct PerChannel {
  // [0..1] is for real and imag of 1st leaking accumulation
  // [2..3] is for real and imag of 2nd leaking accumulation
  // [4..5] is for real and imag of 3rd leaking accumulation
  float accu[6][kNumRotators] = { 0 };
  float LenSqr(size_t i) {
    return accu[4][i] * accu[4][i] + accu[5][i] * accu[5][i];
  }
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
      gain[i] = filter_gains[i] * pow(windowM1, 3.0);
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
    audio *= 0.03;
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
  float GetSample(int c, int i) const {
    return (rot[2][i] * channel[c].accu[4][i] + rot[3][i] * channel[c].accu[5][i]);
  }

  void GetTriplet(float left_to_right_ratio,
                  int rot_ix,
                  float rightr,
                  float righti,
                  float leftr,
                  float lefti,
                  float &right,
                  float &center,
                  float &left) {
    float aver = 0.5 * (rightr + leftr);
    float avei = 0.5 * (righti + lefti);
    center = rot[2][rot_ix] * aver + rot[3][rot_ix] * avei;

    rightr -= left_to_right_ratio * aver;
    righti -= left_to_right_ratio * avei;
    leftr -= (1.0 - left_to_right_ratio) * aver;
    lefti -= (1.0 - left_to_right_ratio) * avei;

    right = rot[2][rot_ix] * rightr + rot[3][rot_ix] * righti;
    left = rot[2][rot_ix] * leftr + rot[3][rot_ix] * lefti;
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
                    size_t samplerate,
                    const std::vector<float>& filter_gains) {
    num_rotators_ = num_rotators;
    num_channels_ = num_channels;
    std::vector<float> freqs(num_rotators);
    for (size_t i = 0; i < num_rotators_; ++i) {
      freqs[i] = BarkFreq(static_cast<float>(i) / (num_rotators_ - 1));
    }
    rotators_ = new Rotators(num_channels, freqs, filter_gains, samplerate);

    max_delay_ = rotators_[0].max_delay_;
    QCHECK_LE(max_delay_, kBlockSize);
    fprintf(stderr, "Rotator bank output delay: %zu\n", max_delay_);
  }
  ~RotatorFilterBank() {
    delete rotators_;
  }

  int64_t FilterAllSingleThreaded(const float* history, int64_t total_in, int64_t len,
                                  float* output, size_t output_size) {
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

  size_t num_rotators_;
  size_t num_channels_;
  Rotators *rotators_;
  int64_t max_delay_;
};


float AngleEffect(float dy, float distance) {
  float dist2 = sqrt(dy * dy + distance * distance);
  float cos_angle = distance / dist2;
  cos_angle = cos_angle * cos_angle * cos_angle;
  return cos_angle;
}

struct MultiChannelDriverModel {
  std::vector<float> ave;   // For high pass filtering of input voltage (~20 Hz or so)
  std::vector<float> pos;   // Position of the driver membrane.
  std::vector<float> dpos;  // Velocity of the driver membrane.
  void Initialize(size_t n) {
    ave.resize(n);
    pos.resize(n);
    dpos.resize(n);
  }
  void Convert(float *p, size_t n) {
    // This number relates to the resonance frequence of the speakers. I suspect I have around ~100 Hz.
    // It is an ad hoc formula.
    const float kResonance = 100.0;
    // Funny constant -- perhaps from 1.0 / (2 * pi * samplerate),
    // didn't analyze yet why this works, but it does.
    const float kFunnyConstant = 0.0000039;
    const float kSuspension = kFunnyConstant * kResonance;

    // damping reduces the speed of the membrane passively as it
    // emits energy or converts it to heat in the suspension deformations
    const float damping = 0.99999;
    const float kSomewhatRandomNonPhysicalPositionRegularization = 0.99999;

    const float kInputMul = 0.3;

    for (int k = 0; k < n; ++k) {
      float kAve = 0.9999;
      ave[k] *= kAve;
      ave[k] += (1.0 - kAve) * p[k];
      float v = kInputMul * (p[k] - ave[k]);
      dpos[k] *= damping;
      dpos[k] += v;
      pos[k] += dpos[k];
      v += kSuspension * pos[k];
      pos[k] *= kSomewhatRandomNonPhysicalPositionRegularization;
      p[k] = v;
    }
  }
};

template <typename In, typename Out>
void Process(
    const int output_channels, const double distance_to_interval_ratio,
    In& input_stream, Out& output_stream) {
  std::vector<float> history(input_stream.channels() * kHistorySize);
  std::vector<float> input(input_stream.channels() * kBlockSize);
  std::vector<float> output(output_channels * kBlockSize);

  MultiChannelDriverModel dm;
  dm.Initialize(output_channels);
  constexpr int64_t kNumRotators = 128;
  std::vector<float> freqs(kNumRotators);
  for (size_t i = 0; i < kNumRotators; ++i) {
    freqs[i] = BarkFreq(static_cast<float>(i) / (kNumRotators - 1));
  }
  std::vector<float> filter_gains(kNumRotators);
  for (size_t i = 0; i < kNumRotators; ++i) {
    filter_gains[i] = GetFilterGains(i);
  }

  RotatorFilterBank rfb(kNumRotators, input_stream.channels(),
                        input_stream.samplerate(), filter_gains);

  std::vector<float> speaker_to_ratio_table;
  speaker_to_ratio_table.reserve(kSubSourcePrecision * (output_channels - 1) +
                                 1);
  for (int i = 0; i < kSubSourcePrecision * (output_channels - 1) + 1; ++i) {
    const float x_div_interval = static_cast<float>(i) / kSubSourcePrecision -
                                  0.5f * (output_channels - 1);
    const float x_div_distance = x_div_interval / distance_to_interval_ratio;
    const float angle = std::atan(x_div_distance);
    speaker_to_ratio_table.push_back(ExpectedLeftToRightRatio(angle));
  }

  int64_t total_in = 0;
  for (;;) {
    int64_t out_ix = 0;
    const int64_t read = input_stream.readf(input.data(), kBlockSize);
    for (int i = 0; i < read; ++i) {
      int input_ix = i + total_in;
      history[2 * (input_ix & kHistoryMask) + 0] = input[2 * i];
      history[2 * (input_ix & kHistoryMask) + 1] = input[2 * i + 1];
    }
    printf("read %d\n", int(read));
    if (read == 0) {
      break;
    }
    for (size_t c = 0; c < kNumRotators; ++c) {
      rfb.rotators_->OccasionallyRenormalize();
    }
    for (int i = 0; i < read; ++i) {
      for (int rot = 0; rot < kNumRotators; ++rot) {
        for (size_t c = 0; c < 2; ++c) {
          int64_t delayed_ix = total_in + i - rfb.rotators_->advance[rot];
          size_t histo_ix = 2 * (delayed_ix & kHistoryMask);
          float delayed = history[histo_ix + c];
          rfb.rotators_->AddAudio(c, rot, delayed);
        }
      }
      rfb.rotators_->IncrementAll();
      for (int rot = 0; rot < kNumRotators; ++rot) {
        const float ratio =
            ActualLeftToRightRatio(rfb.rotators_->channel[0].LenSqr(rot),
                                   rfb.rotators_->channel[1].LenSqr(rot));
        const float subspeaker_index =
            (std::lower_bound(speaker_to_ratio_table.begin(),
                              speaker_to_ratio_table.end(), ratio,
                              std::greater<>()) -
             speaker_to_ratio_table.begin()) * (1.0 / kSubSourcePrecision);
        float stage_size = 1.3; // meters
        float distance_from_center = stage_size * (subspeaker_index - 0.5 * (output_channels - 1)) / (output_channels - 1);
        float assumed_distance_to_line = stage_size * 1.6;
        float distance_to_virtual = sqrt(distance_from_center * distance_from_center +
                                         assumed_distance_to_line * assumed_distance_to_line);
        float dist_ratio = distance_to_virtual * (1.0f / assumed_distance_to_line);
        float index = static_cast<float>(subspeaker_index);
        float right, center, left;
        rfb.rotators_->GetTriplet(subspeaker_index / (output_channels - 1),
                                  rot,
                                  rfb.rotators_->channel[0].accu[4][rot],
                                  rfb.rotators_->channel[0].accu[5][rot],
                                  rfb.rotators_->channel[1].accu[4][rot],
                                  rfb.rotators_->channel[1].accu[5][rot],
                                  right, center, left);
        float speaker_offset_left = (2 - 7.5) * 0.1;
        float speaker_offset_right = (13 - 7.5) * 0.1;
        if (total_in + i >= rfb.max_delay_) {
          for (int kk = 0; kk < output_channels; ++kk) {
            float speaker_offset = (kk - 7.5) * 0.1;
            output[out_ix * output_channels + kk] +=
                AngleEffect(speaker_offset + distance_from_center, assumed_distance_to_line) * center;
            output[out_ix * output_channels + kk] +=
                AngleEffect(speaker_offset - speaker_offset_right, assumed_distance_to_line) * right;
            output[out_ix * output_channels + kk] +=
                AngleEffect(speaker_offset - speaker_offset_left, assumed_distance_to_line) * left;
          }
          if (rot == kNumRotators - 1) {
            ++out_ix;
          }
        }
      }
      dm.Convert(&output[out_ix * output_channels], output_channels);
    }
    output_stream.writef(output.data(), out_ix);
    total_in += read;
    std::fill(output.begin(), output.end(), 0.0);
  }
};

//          output[out_ix * kNumRotators + c] = HardClip(rotators_->GetSampleAll(c));


}  // namespace

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);

  const int output_channels = absl::GetFlag(FLAGS_output_channels);
  const float distance_to_interval_ratio =
      absl::GetFlag(FLAGS_distance_to_interval_ratio);

  QCHECK_EQ(argc, 3) << "Usage: " << argv[0] << " <input> <output>";

  SndfileHandle input_file(argv[1]);
  QCHECK(input_file) << input_file.strError();

  QCHECK_EQ(input_file.channels(), 2);

  SndfileHandle output_file(
      argv[2], /*mode=*/SFM_WRITE, /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
      /*channels=*/output_channels, /*samplerate=*/input_file.samplerate());

  Process(output_channels, distance_to_interval_ratio, input_file, output_file);
}
