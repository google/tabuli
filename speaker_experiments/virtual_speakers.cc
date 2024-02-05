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
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "absl/algorithm/container.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/check.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "sndfile.hh"

ABSL_FLAG(std::string, input_file, "",
          "Path to an audio file to be played back on the virtual speakers");
ABSL_FLAG(std::string, output_file, "", "Where to write the output to");
ABSL_FLAG(
    std::string, virtual_speaker_positions, "-0.5,0.5;0.5,0.5",
    "List of positions for the virtual speakers, where the middle of the array "
    "is assumed to be at (0, 0), positive y is in front (towards the "
    "listener), and positive x goes to the right when facing the array");
ABSL_FLAG(float, speaker_separation, 0.1,
          "Distance between each physical speaker of the array");
ABSL_FLAG(int, num_speakers, 12, "Number of physical speakers in the array");
ABSL_FLAG(float, speed_of_sound, 343.f,
          "Speed of sound in distance units per second");

namespace {

constexpr int64_t kBufferSize = 4096;

}  // namespace

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);

  const std::string input_file = absl::GetFlag(FLAGS_input_file);
  QCHECK(!input_file.empty());

  const std::string output_file = absl::GetFlag(FLAGS_output_file);
  QCHECK(!output_file.empty());

  const int num_speakers = absl::GetFlag(FLAGS_num_speakers);
  const float speed_of_sound = absl::GetFlag(FLAGS_speed_of_sound);

  SndfileHandle sound_file(input_file);
  QCHECK(sound_file) << sound_file.strError();

  const float speaker_separation = absl::GetFlag(FLAGS_speaker_separation);
  const auto SpeakerPosition = [num_speakers, speaker_separation](const int i) {
    return std::make_pair((num_speakers - 1.f) * speaker_separation *
                              (i / (num_speakers - 1.f) - .5f),
                          0.f);
  };

  std::vector<std::pair<float, float>> virtual_speaker_positions;
  for (absl::string_view microphone :
       absl::StrSplit(absl::GetFlag(FLAGS_virtual_speaker_positions), ';')) {
    const std::vector<absl::string_view> coordinates =
        absl::StrSplit(microphone, ',');
    QCHECK_EQ(coordinates.size(), 2);
    float x, y;
    QCHECK(absl::SimpleAtof(coordinates[0], &x));
    QCHECK(absl::SimpleAtof(coordinates[1], &y));
    virtual_speaker_positions.emplace_back(x, y);
  }

  QCHECK_EQ(sound_file.channels(), virtual_speaker_positions.size());

  const float samples_per_distance = sound_file.samplerate() / speed_of_sound;

  Eigen::ArrayXXi delays(num_speakers, virtual_speaker_positions.size());
  Eigen::ArrayXXf multipliers(num_speakers, virtual_speaker_positions.size());
  for (int c = 0; c < num_speakers; ++c) {
    const auto [x, y] = SpeakerPosition(c);
    for (int s = 0; s < virtual_speaker_positions.size(); ++s) {
      const auto [vx, vy] = virtual_speaker_positions[s];
      const float dx = x - vx;
      const float dy = y - vy;
      const float distance = std::hypot(dx, dy);
      if (vy <= y) {
        // Behind
        const float cos_angle = dy / distance;
        delays(c, s) = std::lround(samples_per_distance * distance);
        multipliers(c, s) = cos_angle / distance;
      } else {
        // In front
        const float cos_angle = dy / distance;
        delays(c, s) = std::lround(-samples_per_distance * distance);
        // This happens to be equivalent to just `dy` but expressing it like
        // this is perhaps clearer and makes it potentially easier to experiment
        // with other possibilities.
        // TODO(sboukortt): Notably, would cos_angle^2 possibly be better? Let's
        // investigate this.
        multipliers(c, s) = distance * cos_angle;
      }
    }
  }

  // Prevent clipping.
  multipliers /= multipliers.maxCoeff();

  // Prevent having to start in the past.
  delays -= delays.minCoeff();

  std::vector<float> input_samples(sound_file.channels() * sound_file.frames());
  QCHECK_EQ(sound_file.read(input_samples.data(), input_samples.size()),
            input_samples.size());

  const int64_t num_output_frames = sound_file.frames() + delays.maxCoeff();

  SndfileHandle output_sound_file(output_file, /*mode=*/SFM_WRITE,
                                  /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
                                  /*channels=*/num_speakers,
                                  /*samplerate=*/sound_file.samplerate());
  QCHECK(output_sound_file) << output_sound_file.strError();
  std::vector<float> output_buffer(kBufferSize * output_sound_file.channels());
  for (int64_t i = 0; i < num_output_frames; i += kBufferSize) {
    absl::c_fill(output_buffer, 0.f);
    const int64_t buffer_size = std::min(kBufferSize, num_output_frames - i);
    for (int c = 0; c < output_sound_file.channels(); ++c) {
      // TODO: Gaussian or something like that
      const float window =
          (c == 0 || c == output_sound_file.channels() - 1) ? 0.5f : 1.f;
      for (int s = 0; s < virtual_speaker_positions.size(); ++s) {
        const int64_t delay = delays(c, s);
        const float multiplier = window * multipliers(c, s);
        const int64_t upper_bound =
            std::min(buffer_size, sound_file.frames() + delay - i);
        for (int64_t j = std::max<int64_t>(0, delay - i); j < upper_bound;
             ++j) {
          const int64_t source_i = i + j - delay;
          output_buffer[j * output_sound_file.channels() + c] +=
              input_samples[source_i * sound_file.channels() + s] * multiplier;
        }
      }
    }

    output_sound_file.writef(output_buffer.data(), buffer_size);
  }
}
