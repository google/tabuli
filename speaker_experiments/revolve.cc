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

ABSL_FLAG(int, output_channels, 11, "number of output channels");

// optimize this?!
ABSL_FLAG(double, distance_to_interval_ratio, 4,
          "ratio of (distance between microphone and source array) / (distance "
          "between each source); default = 40cm / 10cm = 4");

namespace {

constexpr int kSubSourcePrecision = 4000;

float SquaredMicrophoneResponse(const float angle) {
  float ret = 0.5f * (1.0f + std::cos(angle));
  return ret * ret;
}

float ExpectedLeftToRightRatio(const float angle) {
  return (1e-14 + SquaredMicrophoneResponse(angle + M_PI / 4)) /
         (1e-14 + SquaredMicrophoneResponse(angle - M_PI / 4));
}

float ActualLeftToRightRatio(float left, float right) {
  return (1e-14 + left) / (1e-14 + right);
}

constexpr int64_t kNumRotators = 128;

float AngleEffect(float dy, float distance, int rot) {
  float dist2 = dy * dy + distance * distance;
  float cos_angle = distance * distance / dist2; // 2nd
  cos_angle *= cos_angle;  // 4th
  if (rot > 30) {
    cos_angle *= cos_angle;  // 8th
  }
  if (rot > 50) {
    cos_angle *= cos_angle;  // 16th
  }
  if (rot >= 75) {
    cos_angle *= cos_angle;  // 32nd
  }
  if (rot >= 105) {
    cos_angle *= cos_angle;  // 64th
  }
  return cos_angle;
}

float Freq(int i) {
  // Center frequencies of the filter bank, plus one frequency in both ends.
  static const float kFreq[130] = {
    18.949270513107507, 20.0, 21.10899201757206, 22.279477199896046, 23.51486531841422, 24.818755215034393, 26.194945286011798, 27.647444547157992, 29.180484312611213, 30.798530521189843, 32.50629674623729, 34.30875792685758, 36.21116486054244, 38.219059499408814, 40.33829109460661, 42.57503323592745, 44.93580183625289, 47.4274741132331, 50.05730862349216, 52.83296640772185, 55.76253330826266, 58.85454352418564, 62.1180044724941, 65.56242302786917, 69.19783321739865, 73.03482545096755, 77.08457737246212, 81.35888642166094, 85.87020420166964, 90.63167275201636, 95.65716283307582, 100.9613143333494, 106.55957891731282, 112.46826503806983, 118.70458544593973, 125.2867073313771, 132.2338052482963, 139.56611697197326, 147.3050024542456, 155.47300604775518, 164.09392218049982, 173.1928646720131, 182.79633989309812, 192.93232398223984, 203.63034434363638, 214.92156566426348, 226.83888070055136, 239.41700609914596, 252.6925835308937, 266.7042864326648, 281.49293266796866, 297.10160343455505, 313.57576876539406, 330.9634199886363, 349.31520953242364, 368.68459848182215, 389.1280123177271, 410.7050052914292, 433.4784339136835, 457.5146400636791, 482.88364425132784, 509.65934959586934, 537.9197571150087, 567.747192951751, 599.2285482008728, 632.4555320336758, 667.5249388584077, 704.5389302946203, 743.605332782895, 784.8379516969072, 828.356902872881, 874.2889625222178, 922.7679365466429, 973.9350503317262, 1027.9393601543027, 1084.9381874022652, 1145.0975768716758, 1208.5927804762655, 1275.6087677784355, 1346.3407648289965, 1420.9948228853573, 1499.7884186649117, 1582.951087882232, 1670.7250939156525, 1763.3661335511417, 1861.1440818593978, 1964.3437783760758, 2073.265856875396, 2188.2276211543713, 2309.563969378916, 2437.6283696845794, 2572.793889873949, 2715.454284210368, 2866.0251404739256, 3024.945090621247, 3192.677088575884, 3369.7097588716774, 3556.5588200778457, 3753.7685871524372, 3961.9135571006773, 4181.6000825574365, 4413.46813816918, 4658.193184921091, 4916.488137840395, 5189.105442808031, 5476.839268528722, 5780.5278200449, 6101.055780534051, 6439.356888502758, 6796.416657885119, 7173.275248969538, 7571.030498517261, 7990.841117899773, 8433.930068571646, 8901.58812471199, 9395.177633412985, 9916.136483369311, 10465.982293629893, 11046.316834614197, 11658.830694272148, 12305.308202980745, 12987.632631524228, 13707.791677300165, 14467.883254733495, 15270.12160676669, 16116.843755229636, 17010.516308879927, 17953.742648946285, 18949.27051310751, 20000.0, 

  };
  return kFreq[i + 1];
}

float FreqAve(int i) {
  return sqrt(Freq(i - 1) * Freq(i + 1));
}

double CalculateBandwidthInHz(int i) {
  return sqrt ((Freq(i + 1) - Freq(i)) * (Freq(i) - Freq(i - 1)));
}

struct PerChannel {
  // [0..1] is for real and imag of 1st leaking accumulation
  // [2..3] is for real and imag of 2nd leaking accumulation
  // [4..5] is for real and imag of 3rd leaking accumulation
  double accu[16][kNumRotators] = {0};
  float LenSqr(size_t i) {
    // prev rotators will be integrated to cur, i.e., they represent the derivative.
    // if derivative is stronger than the cur value, let's emphasize that so that
    // beginnings of the sounds are more emphasized in spatialization.
    //    float prev = (accu[12][i] * accu[12][i] + accu[13][i] * accu[13][i]);
    float cur = (accu[8][i] * accu[8][i] + accu[9][i] * accu[9][i]);
    return cur; // 0.1 * prev + cur;
  }
};

struct Rotators {
  // Four arrays of rotators.
  // [0..1] is real and imag for rotation speed
  // [2..3] is real and image for a frequency rotator of length sqrt(gain[i])
  // Values inserted into the rotators are multiplied with this rotator in both
  // input and output, leading to a total gain multiplication if the length is
  // at sqrt(gain).
  float rot[4][kNumRotators] = {0};
  std::vector<PerChannel> channel;
  // Accu has the channel related data, everything else the same between
  // channels.
  float window[kNumRotators];
  float gain[kNumRotators];
  int16_t delay[kNumRotators] = {0};
  int16_t advance[kNumRotators] = {0};
  int16_t max_delay_ = 0;


  int FindMedian3xLeaker(float window) {
    // Approximate filter delay. TODO: optimize this value along with gain
    // values. Recordings can sound better with -2.32 as it pushes the bass
    // signals a bit earlier and likely compensates human hearing's deficiency
    // for temporal separation.
    static const double kMagic = 5.46;
    return kMagic / window;
  }

  Rotators() {}
  Rotators(int num_channels, const float sample_rate) {
    channel.resize(num_channels);
    static const float kSampleRate = 48000.0;
    static const float kHzToRad = 2.0f * M_PI / kSampleRate;
    static const double kWindow = 0.99964;
    for (int i = 0; i < kNumRotators; ++i) {
      float bandwidth = CalculateBandwidthInHz(i);  // bandwidth per bucket.
      window[i] = std::pow(kWindow, bandwidth);
      delay[i] = FindMedian3xLeaker(1.0 - window[i]);
      float windowM1 = 1.0f - window[i];
      max_delay_ = std::max(max_delay_, delay[i]);
      const float f = FreqAve(i) * kHzToRad;
      gain[i] = pow(windowM1, 4.0);
      rot[0][i] = float(std::cos(f));
      rot[1][i] = float(-std::sin(f));
      rot[2][i] = gain[i];
      rot[3][i] = 0.0f;
    }
    for (size_t i = 0; i < kNumRotators; ++i) {
      advance[i] = 1 + max_delay_ - delay[i];
    }
  }
  void AddAudio(int c, int i, float audio) {
    channel[c].accu[0][i] += rot[2][i] * audio;
    channel[c].accu[1][i] += rot[3][i] * audio;
  }
  void OccasionallyRenormalize() {
    for (int i = 0; i < kNumRotators; ++i) {
      float norm =
	gain[i] / sqrt(rot[2][i] * rot[2][i] + rot[3][i] * rot[3][i]);
      rot[2][i] *= norm;
      rot[3][i] *= norm;
    }
  }
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
  void GetTriplet(float left_to_right_ratio, int rot_ix,
		  float *out_of_phase_val,
		  float leftr, float lefti, 
		  float rightr, float righti,
		  float prev_leftr, float prev_lefti, 
		  float prev_rightr, float prev_righti,
		  float window,
		  float &left, float &center,  float &right,
		  float &out_of_phase_left,
		  float &out_of_phase_right) {
    float out_of_phase = 0;
    float q = prev_rightr * prev_leftr + prev_righti * prev_lefti;
    if (q < 0 && rot_ix >= 30 && rot_ix < 128 ) {
      float mul = 2.0 * (rot_ix - 44) * (1.0 / 24.0);
      if (mul > 1) {
	mul = 1;
      }
      if (mul < 0) {
	mul = 0;
      }
      out_of_phase = mul * -q / (0.5 * (((prev_rightr * prev_rightr + prev_righti * prev_righti) + (prev_leftr * prev_leftr + prev_lefti * prev_lefti))));
      out_of_phase *= (1.0 / window);
      out_of_phase *= (1.0 / window);
      out_of_phase *= (1.0 / window);
    }
    out_of_phase_val[0] *= 1 - window;
    out_of_phase_val[0] += window * out_of_phase;
    out_of_phase_val[1] *= 1 - window;
    out_of_phase_val[1] += window * out_of_phase_val[0];
    out_of_phase_val[2] *= 1 - window;
    out_of_phase_val[2] += window * out_of_phase_val[1];
    out_of_phase_left = out_of_phase_val[2] * (rot[2][rot_ix] * rightr + rot[3][rot_ix] * righti);
    out_of_phase_right = out_of_phase_val[2] * (rot[2][rot_ix] * leftr + rot[3][rot_ix] * lefti);
    leftr *= 1.0 - out_of_phase_val[2];
    lefti *= 1.0 - out_of_phase_val[2];
    rightr *= 1.0 - out_of_phase_val[2];
    righti *= 1.0 - out_of_phase_val[2];

    float rlen = (rightr * rightr + righti * righti) + 1e-20;
    float llen = (leftr * leftr + lefti * lefti) + 1e-20;
    float invsumlen = 1.0 / (rlen + llen);
    rlen *= invsumlen;
    llen *= invsumlen;

    float aver = (rlen * rightr + llen * leftr);
    float avei = (rlen * righti + llen * lefti);

    center = rot[2][rot_ix] * aver + rot[3][rot_ix] * avei;

    rightr -= rlen * aver;
    righti -= rlen * avei;
    leftr -= llen * aver;
    lefti -= llen * avei;

    right = rot[2][rot_ix] * rightr + rot[3][rot_ix] * righti;
    left = rot[2][rot_ix] * leftr + rot[3][rot_ix] * lefti;
    // center = 0;
    // center = left = right = 0;
  }
};

static constexpr int64_t kBlockSize = 1 << 15;
static const int kHistorySize = (1 << 18);
static const int kHistoryMask = kHistorySize - 1;

struct RotatorFilterBank {
  RotatorFilterBank(size_t num_channels, size_t samplerate) {
    rotators_ = new Rotators(num_channels, samplerate);

    max_delay_ = rotators_->max_delay_;
    QCHECK_LE(max_delay_, kBlockSize);
    fprintf(stderr, "Rotator bank output delay: %zu\n", max_delay_);
  }
  ~RotatorFilterBank() { delete rotators_; }
  Rotators *rotators_;
  int64_t max_delay_;
};

float HardClip(float v) { return std::max(-1.0f, std::min(1.0f, v)); }

struct MultiChannelDriverModel {
  std::vector<float>
      ave;  // For high pass filtering of input voltage (~20 Hz or so)
  std::vector<float> pos;   // Position of the driver membrane.
  std::vector<float> dpos;  // Velocity of the driver membrane.
  void Initialize(size_t n) {
    ave.resize(n);
    pos.resize(n);
    dpos.resize(n);
  }
  void Convert(float *p, size_t n) {
    // This number relates to the resonance frequence of the speakers.
    // I suspect I have around ~100 Hz.
    // It is an ad hoc formula.
    const float kResonance = 100.0;
    // Funny constant -- perhaps from 1.0 / (2 * pi * samplerate),
    // didn't analyze yet why this works, but it does.
    const float kFunnyConstant = 0.0000035;
    const float kSuspension = kFunnyConstant * kResonance;

    // damping reduces the speed of the membrane passively as it
    // emits energy or converts it to heat in the suspension deformations
    const float damping = 0.99999;
    const float kSomewhatRandomNonPhysicalPositionRegularization = 0.99998;

    const float kInputMul = 0.1;

    for (int k = 0; k < n; ++k) {
      float kAve = 0.9995;
      ave[k] *= kAve;
      ave[k] += (1.0 - kAve) * p[k];
      float v = kInputMul * (p[k] - ave[k]);
      dpos[k] *= damping;
      dpos[k] += v;
      pos[k] += dpos[k];
      v += kSuspension * pos[k];
      pos[k] *= kSomewhatRandomNonPhysicalPositionRegularization;
      p[k] = HardClip(v);
    }
  }
};

// Control delays for binaural experience.
struct BinauralModel {
  size_t index = 0;
  float channel[2][4096] = {0};
  void GetAndAdvance(float *left_arg, float *right_arg) {
    *left_arg = HardClip(channel[0][index & 0xfff]);
    *right_arg = HardClip(channel[1][index & 0xfff]);
    /*
    channel[1][(index + 27) & 0xfff] += 0.01 * channel[0][index & 0xfff];
    channel[0][(index + 27) & 0xfff] += 0.01 * channel[1][index & 0xfff];
    */
    channel[0][index & 0xfff] = 0.0;
    channel[1][index & 0xfff] = 0.0;
    ++index;
  }
  void Emit(float *p) { GetAndAdvance(p, p + 1); }
  void WriteWithDelay(size_t c, size_t delay, float v) {
    channel[c][(index + delay) & 0xfff] += v;
  }
  void WriteWithFloatDelay(int c, float float_delay, float v) {
    int delay = floor(float_delay);
    float frac = float_delay - delay;
    WriteWithDelay(c, delay, v * (1.0 - frac));
    WriteWithDelay(c, delay + 1, v * frac);
  }
};

float *GetBinauralTable() {
  static const float binau[16] = {
      1.4, 1.3, 1.2, 1.1,  1.0, 0.9,  0.8, 0.7,

      0.6, 0.5, 0.4, 0.35, 0.3, 0.25, 0.2, 0.15,
  };
  static float table[kNumRotators * 16];
  for (int i = 0; i < kNumRotators; ++i) {
    for (int k = 0; k < 16; ++k) {
      table[i * 16 + k] = pow(binau[k], i / 128.0);
    }
  }
  return table;
}

template <typename In, typename Out>
void Process(const int output_channels_arg, const double distance_to_interval_ratio,
	     bool two_reverb_channels,
             In &input_stream, Out &output_stream,
             Out &binaural_output_stream) {
  std::vector<float> history(input_stream.channels() * kHistorySize);
  std::vector<float> input(input_stream.channels() * kBlockSize);

  int output_channels = two_reverb_channels ? output_channels_arg + 2 : output_channels_arg;
  std::vector<float> output(output_channels * kBlockSize);
  std::vector<float> binaural_output(2 * kBlockSize);
  float midpoint = 0.5 * (output_channels - 1);
  float speaker_offset_left = (2 - midpoint) * 0.1;
  float speaker_offset_right = (9 - midpoint) * 0.1;

  MultiChannelDriverModel dm;
  dm.Initialize(output_channels);
  BinauralModel binaural;
  float *btable = GetBinauralTable();
  constexpr int64_t kNumRotators = 128;
  std::vector<float> freqs(kNumRotators);
  for (size_t i = 0; i < kNumRotators; ++i) {
    freqs[i] = FreqAve(i);
  }
  std::vector<float> filter_gains(kNumRotators);
  for (size_t i = 0; i < kNumRotators; ++i) {
    filter_gains[i] = 1.0;
  }

  RotatorFilterBank rfb(input_stream.channels(),
                        input_stream.samplerate());

  std::vector<float> speaker_to_ratio_table;
  speaker_to_ratio_table.reserve(kSubSourcePrecision * (output_channels_arg - 1) +
                                 1);
  for (int i = 0; i < kSubSourcePrecision * (output_channels_arg - 1) + 1; ++i) {
    const float x_div_interval = static_cast<float>(i) / kSubSourcePrecision -
                                 0.5f * (output_channels_arg - 1);
    const float x_div_distance = x_div_interval / distance_to_interval_ratio;
    const float angle = std::atan(x_div_distance);
    speaker_to_ratio_table.push_back(ExpectedLeftToRightRatio(angle));
  }

  int64_t total_in = 0;
  bool extend_the_end = true;
  float speaker_index_array[kNumRotators][3];
  for (int i = 0; i < 128; ++i) {
    speaker_index_array[i][0] = 0.5 * (output_channels_arg - 1);
    speaker_index_array[i][1] = 0.5 * (output_channels_arg - 1);
    speaker_index_array[i][2] = 0.5 * (output_channels_arg - 1);
  }
  float out_of_phase_array[kNumRotators][3] = {{0}};
  for (;;) {
    int64_t out_ix = 0;
    int64_t read = input_stream.readf(input.data(), kBlockSize);
    for (int i = 0; i < read; ++i) {
      int input_ix = i + total_in;
      history[2 * (input_ix & kHistoryMask) + 0] = input[2 * i];
      history[2 * (input_ix & kHistoryMask) + 1] = input[2 * i + 1];
    }
    printf("read %d\n", int(read));
    if (read == 0) {
      if (extend_the_end) {
        // Empty the filters and the history.
        extend_the_end = false;
        read = rfb.max_delay_;
        printf("empty the history buffer %d\n", int(read));
        for (int i = 0; i < read; ++i) {
          int input_ix = i + total_in;
          history[2 * (input_ix & kHistoryMask) + 0] = 0;
          history[2 * (input_ix & kHistoryMask) + 1] = 0;
        }
      } else {
        break;
      }
    }
    rfb.rotators_->OccasionallyRenormalize();
    for (int i = 0; i < read; ++i) {
      for (int rot = 0; rot < kNumRotators; ++rot) {
        for (size_t c = 0; c < 2; ++c) {
          int64_t delayed_ix = total_in + i - rfb.rotators_->advance[rot];
          size_t histo_ix = 2 * (delayed_ix & kHistoryMask);
          float delayed = history[histo_ix + c];
	  // Mathematically should be less than 0.5 because two channels can add up.
	  // Imperfect phase corrections etc. can lead to further rendering problems,
	  // thus better to correct more than 0.5, i.e., for example 0.1 and normalize
	  // the output using external tools (such as sox).
	  float kNormalizationToReduceAmplitude = 0.1;
	  delayed *= kNormalizationToReduceAmplitude;
          rfb.rotators_->AddAudio(c, rot, delayed);
        }
      }
      rfb.rotators_->IncrementAll();
      for (int rot = kNumRotators - 1; rot >= 0; --rot) {
        const float ratio =
	  ActualLeftToRightRatio(rfb.rotators_->channel[1].LenSqr(rot),
				 rfb.rotators_->channel[0].LenSqr(rot));
        float subspeaker_index =
            (std::lower_bound(speaker_to_ratio_table.begin(),
                              speaker_to_ratio_table.end(), ratio,
                              std::greater<>()) -
             speaker_to_ratio_table.begin()) *
            (1.0 / kSubSourcePrecision);
	speaker_index_array[rot][0] *= 1.0 - rfb.rotators_->window[rot];
	speaker_index_array[rot][0] += rfb.rotators_->window[rot] * subspeaker_index;
	speaker_index_array[rot][1] *= 1.0 - rfb.rotators_->window[rot];
	speaker_index_array[rot][1] += rfb.rotators_->window[rot] * speaker_index_array[rot][0];
	speaker_index_array[rot][2] *= 1.0 - rfb.rotators_->window[rot];
	speaker_index_array[rot][2] += rfb.rotators_->window[rot] * speaker_index_array[rot][1];
	subspeaker_index = speaker_index_array[rot][2];
	/*
        if (subspeaker_index < 1.0) {
          subspeaker_index = 1.0;
        }
        if (subspeaker_index >= 11.0) {
          subspeaker_index = 11.0;
        }
	*/
        float stage_size = 1.3;  // meters
        float distance_from_center =
            stage_size * (subspeaker_index - 0.5 * (output_channels_arg - 1)) /
            (output_channels_arg - 1);
        float assumed_distance_to_line = stage_size * 2.0;
        float right = 0, center = 0, left = 0;
	float out_of_phase_right = 0;
	float out_of_phase_left = 0;
	float ratio_expected = subspeaker_index / (output_channels_arg - 1);
        rfb.rotators_->GetTriplet(ratio_expected,
			          rot,
				  &out_of_phase_array[rot][0],
                                  rfb.rotators_->channel[0].accu[14][rot],
                                  rfb.rotators_->channel[0].accu[15][rot],
                                  rfb.rotators_->channel[1].accu[14][rot],
                                  rfb.rotators_->channel[1].accu[15][rot],
                                  rfb.rotators_->channel[0].accu[8][rot],
                                  rfb.rotators_->channel[0].accu[9][rot],
                                  rfb.rotators_->channel[1].accu[8][rot],
                                  rfb.rotators_->channel[1].accu[9][rot],
				  rfb.rotators_->window[rot],
				  left,
                                  center,
				  right,
				  out_of_phase_left,
				  out_of_phase_right);
        if (total_in + i >= rfb.max_delay_) {
#define BINAURAL
#ifdef BINAURAL
          // left and right.
          {
            // left *= 2.0;
            // right *= 2.0;
            //  Some hacks here: 27 samples is roughly 19 cm which I use
            //  as an approximate for the added delay needed for this
            //  kind of left-right 'residual sound'. perhaps it is too
            //  much. perhaps having more than one delay would make sense.
            //
            //  This computation being left-right and with delay accross
            //  causes a relaxed feeling of space to emerge.
            float lbin = left * 2;
            float rbin = right * 2;
            size_t delay = 0;
            for (int i = 0; i < 5; ++i) {
              binaural.WriteWithDelay(0, delay, lbin);
              binaural.WriteWithDelay(1, delay, rbin);

              float lt = btable[16 * rot + 15] * lbin;
              float rt = btable[16 * rot + 15] * rbin;

              // swap:
              lbin = rt;
              rbin = lt;

              if (i == 0) {
                delay += 17;
              } else {
                delay += 27;
              }
            }
          }
          {
            // center.
            int speaker = static_cast<int>(floor(subspeaker_index));
            float off = subspeaker_index - speaker;
            float right_gain_0 = btable[16 * rot + speaker];
            float right_gain_1 = btable[16 * rot + speaker + 1];
            float right_gain = (1.0 - off) * right_gain_0 + off * right_gain_1;
            float left_gain_0 = btable[16 * rot + 15 - speaker];
            float left_gain_1 = btable[16 * rot + 15 - speaker - 1];
            float left_gain = (1.0 - off) * left_gain_0 + off * left_gain_1;
            float kDelayMul = 0.15;
            float delay_p = 0;
            {
              // Making delay diffs less in the center maintains
              // a smaller acoustic picture of the singer.
              // This relates to Euclidian distances and makes
              // physical sense, too.
              float len = (output_channels_arg - 1);
              float dx = subspeaker_index - 0.5 * len;
              float dist = sqrt(dx * dx + len * len) - len;
              if (dx < 0) {
                dist = -dist;
              }
              dist += 0.5 * len;
              delay_p = dist;
            }

            float delay_l = 1 + kDelayMul * delay_p;
            float delay_r = 1 + kDelayMul * ((output_channels_arg - 1) - delay_p);
            binaural.WriteWithFloatDelay(0, delay_l, center * left_gain);
            binaural.WriteWithFloatDelay(1, delay_r, center * right_gain);
          }
#endif
	  
          // for (int kk = rot & 1; kk < output_channels_arg; kk += 1 + (rot < 70 && rot > 8)) {  // distribute alternating frequencies to every 2nd speaker
          //for (int kk = rot & 3; kk < output_channels_arg; kk += 4) {  // the same in groups of 4
	  
	  int speaker = static_cast<int>(floor(subspeaker_index));
	  float off = subspeaker_index - speaker;

	  if (rot < 25) {
	    output[out_ix * output_channels + 0] += 0.5 * right;
	    output[out_ix * output_channels + 1] += 0.5 * right;
	    output[out_ix * output_channels + output_channels_arg - 2] += 0.5 * left;
	    output[out_ix * output_channels + output_channels_arg - 1] += 0.5 * left;
	    float v = center * (1.0f / output_channels_arg);
	    for (int kk = 0; kk < output_channels_arg; kk++) {
	      output[out_ix * output_channels + kk] += v;
	    }
	    float one_per_output_channels_arg = 0.5;
	    if (two_reverb_channels) {
	      output[(out_ix + 1) * output_channels - 2 ] += left * one_per_output_channels_arg;
	      output[(out_ix + 1) * output_channels - 1 ] += right * one_per_output_channels_arg;
	      output[(out_ix + 1) * output_channels - 2 ] += out_of_phase_left * 0.5;
	      output[(out_ix + 1) * output_channels - 1 ] += out_of_phase_right * 0.5;
	    }
	  } else {
	    std::vector<float> w(output_channels_arg);
	    {
	      float sum = 1e-20;
	      for (int kk = 0; kk < output_channels_arg; kk++) {
		float speaker_offset = (kk - midpoint) * 0.1;
		w[kk] = AngleEffect(speaker_offset + distance_from_center,
				    assumed_distance_to_line, rot);
		sum += w[kk];
	      }
	      float scale = 1.0f / sum;
	      for (int kk = 0; kk < output_channels_arg; kk++) {
		output[out_ix * output_channels + kk] += w[kk] * center * scale;
	      }
	    }
	    float sidew[4] = { 0.06, 0.06, 0.05, 0.03 };
	    for (int zz = 0; zz < 4; ++zz) {
	      output[out_ix * output_channels + zz] += sidew[zz] * left;
	      output[out_ix * output_channels + output_channels_arg - 1 - zz] += sidew[zz] * right;
	    }
	    float one_per_output_channels_arg = 0.5 * (1.0 - sidew[0] - sidew[1] - sidew[2] - sidew[3]);
	    if (two_reverb_channels) {
	      // two last channels reserved for reverb signal
	      // add hock normalization
	      output[(out_ix + 1) * output_channels - 2 ] += left * one_per_output_channels_arg;
	      output[(out_ix + 1) * output_channels - 1 ] += right * one_per_output_channels_arg;
	      output[(out_ix + 1) * output_channels - 2 ] += out_of_phase_left * 0.5;
	      output[(out_ix + 1) * output_channels - 1 ] += out_of_phase_right * 0.5;
	    }
	  }
        }
      }
      if (total_in + i >= rfb.max_delay_) {
#ifdef BINAURAL
        binaural.Emit(&binaural_output[out_ix * 2]);
#endif
        dm.Convert(&output[out_ix * output_channels], output_channels);
        ++out_ix;
      }
    }
    output_stream.writef(output.data(), out_ix);
    binaural_output_stream.writef(binaural_output.data(), out_ix);
    total_in += read;
    std::fill(output.begin(), output.end(), 0.0);
    std::fill(binaural_output.begin(), binaural_output.end(), 0.0);
  }
};

}  // namespace

int main(int argc, char **argv) {
  const std::vector<char*> positional_args = absl::ParseCommandLine(argc, argv);

  const int output_channels = absl::GetFlag(FLAGS_output_channels);
  const float distance_to_interval_ratio =
      absl::GetFlag(FLAGS_distance_to_interval_ratio);

  QCHECK_EQ(positional_args.size(), 4)
      << "Usage: " << argv[0]
      << " <input> <multichannel-output> <binaural-headphone-output>";

  SndfileHandle input_file(positional_args[1]);
  QCHECK(input_file) << input_file.strError();

  QCHECK_EQ(input_file.channels(), 2);
  
  bool two_reverb_channels = true;

  SndfileHandle output_file(
      positional_args[2], /*mode=*/SFM_WRITE, /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
      /*channels=*/output_channels + 2 * two_reverb_channels, /*samplerate=*/input_file.samplerate());

  SndfileHandle binaural_output_file(
      positional_args[3], /*mode=*/SFM_WRITE, /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
      /*channels=*/2, /*samplerate=*/input_file.samplerate());

  Process(output_channels, distance_to_interval_ratio, 
	  two_reverb_channels,
	  input_file, output_file,
          binaural_output_file);
}
