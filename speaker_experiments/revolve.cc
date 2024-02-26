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

float SquaredNorm(const std::complex<double> c) { return std::norm(c); }

double MicrophoneResponse(const double angle) {
  return 0.5f * (1.25f + std::cos(angle));
}

double ExpectedLeftToRightRatio(const double angle) {
  return (1e-3 + MicrophoneResponse(angle + M_PI / 4)) /
         (1e-3 + MicrophoneResponse(angle - M_PI / 4));
}

float ActualLeftToRightRatio(const std::complex<double> left,
                             const std::complex<double> right) {
  return std::sqrt((1e-13 + SquaredNorm(left)) / (1e-13 + SquaredNorm(right)));
}

// Fast approximate function for temporal coherence of 3x exp leakers.
double FindMedian3xLeaker(double window) {
  return -2.32/log(window);
}

constexpr int64_t kNumRotators = 128;

// for fourier-like transform
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

    audio *= 0.1;
    rot[1] += windowM1 * audio * rot[0];
    rot[2] += windowM1 * rot[1];
    rot[3] += windowM1 * rot[2];
  }
  void GetTriplet(std::complex<double> right_rot,
                  std::complex<double> left_rot,
                  double &right,
                  double &center,
                  double &left) {
    auto ave = 0.5 * (right_rot + left_rot);
    center = rot[0].real() * ave.real() + rot[0].imag() * ave.imag();

    right_rot -= ave;
    left_rot -= ave;
    right = rot[0].real() * right_rot.real() + rot[0].imag() * right_rot.imag();
    left = rot[0].real() * left_rot.real() + rot[0].imag() * left_rot.imag();
  }
  double GetSample() const {
    return rot[0].real() * rot[3].real() + rot[0].imag() * rot[3].imag();
  }
  double SquaredAmplitude() const { return std::norm(rot[3]); }
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

double AngleEffect(float dy, float distance) {
  float dist2 = sqrt(dy * dy + distance * distance);
  float cos_angle = distance / dist2;
  cos_angle = cos_angle * cos_angle * cos_angle;
  return cos_angle;
}

static constexpr int64_t kBlockSize = 32768;
static const int kHistorySize = (1 << 18);
static const int kHistoryMask = kHistorySize - 1;


template <typename In, typename Out>
void Process(
    const int output_channels, const double distance_to_interval_ratio,
    In& input_stream, Out& output_stream) {
  std::vector<double> history(input_stream.channels() * kHistorySize);
  std::vector<double> input(input_stream.channels() * kBlockSize);
  std::vector<double> output(output_channels * kBlockSize);

  std::vector<Rotator> rot_left, rot_right;
  constexpr int64_t kNumRotators = 128;
  rot_left.reserve(kNumRotators);
  rot_right.reserve(kNumRotators);
  for (int i = 0; i < kNumRotators; ++i) {
    const double frequency =
        BarkFreq(static_cast<double>(i) / (kNumRotators - 1));
    rot_left.emplace_back(frequency, input_stream.samplerate());
    rot_right.emplace_back(frequency, input_stream.samplerate());
  }

  std::vector<double> speaker_to_ratio_table;
  speaker_to_ratio_table.reserve(kSubSourcePrecision * (output_channels - 1) +
                                 1);
  for (int i = 0; i < kSubSourcePrecision * (output_channels - 1) + 1; ++i) {
    const double x_div_interval = static_cast<double>(i) / kSubSourcePrecision -
                                  0.5f * (output_channels - 1);
    const double x_div_distance = x_div_interval / distance_to_interval_ratio;
    const double angle = std::atan(x_div_distance);
    speaker_to_ratio_table.push_back(ExpectedLeftToRightRatio(angle));
  }

  int64_t total = 0;
  for (;;) {
    const int64_t read = input_stream.readf(input.data(), kBlockSize);
    for (int i = 0; i < read; ++i) {
      int input_ix = i + total;
      history[2 * (input_ix & kHistoryMask) + 0] = input[2 * i];
      history[2 * (input_ix & kHistoryMask) + 1] = input[2 * i + 1];
    }
    printf("read %d\n", int(read));
    if (read == 0) {
      break;
    }
    for (int i = 0; i < read; ++i) {
      for (int rot = 0; rot < kNumRotators; ++rot) {
        int delayed_ix = total + i - rot_left[rot].advance;
        float delayed_r = history[2 * (delayed_ix & kHistoryMask) + 0];
        float delayed_l = history[2 * (delayed_ix & kHistoryMask) + 1];
        rot_left[rot].Increment(delayed_r);
        rot_right[rot].Increment(delayed_l);
        const float ratio =
            ActualLeftToRightRatio(rot_left[rot].rot[3],
                                   rot_right[rot].rot[3]);
        const double subspeaker_index =
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
        double right, center, left;
        rot_left[rot].GetTriplet(rot_right[rot].rot[3], rot_left[rot].rot[3], right, center, left);
        float speaker_offset_left = (2 - 7.5) * 0.1;
        float speaker_offset_right = (13 - 7.5) * 0.1;
        for (int kk = 0; kk < 16; ++kk) {
          float speaker_offset = (kk - 7.5) * 0.1;
          output[i * output_channels + kk] +=
              AngleEffect(speaker_offset + distance_from_center, assumed_distance_to_line) * center;
          output[i * output_channels + kk] +=
              AngleEffect(speaker_offset - speaker_offset_right, assumed_distance_to_line) * right;
          output[i * output_channels + kk] +=
              AngleEffect(speaker_offset - speaker_offset_left, assumed_distance_to_line) * left;
        }
      }
    }
    output_stream.writef(output.data(), read);
    total += read;
    std::fill(output.begin(), output.end(), 0.0);
  }
};


}  // namespace

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);

  const int output_channels = absl::GetFlag(FLAGS_output_channels);
  const double distance_to_interval_ratio =
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
