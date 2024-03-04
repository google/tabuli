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

/*
This utility is intended to improve the driver dynamics of a wave
field synthesis loudspeaker prototype currently located on Jyrki's desk.

This utility works best with 48k sample rate.
*/

#include <stdio.h>
#include <stdlib.h>

#include <algorithm>
#include <sndfile.hh>
#include <string>
#include <vector>

struct Sound {
 public:
  std::vector<float> wav;
  int numsamples;
  int numchannels;
  int samplerate;
};

void Read(const std::string &path, Sound *snd) {
  SndfileHandle f(path);
  if (!f) {
    fprintf(stderr, "%s\n", f.strError());
    exit(-1);
  }
  int numchannels = f.channels();
  snd->numchannels = numchannels;
  snd->samplerate = f.samplerate();
  int numsamples = f.frames();
  snd->numsamples = numsamples;
  snd->wav.resize(numchannels * numsamples);

  printf("read %d channels, %d samples: size()=%zu\n", numchannels, numsamples,
         snd->wav.size());
  if (f.readf(snd->wav.data(), numsamples) != numsamples) {
    fprintf(stderr, "%s\n", f.strError());
    exit(-1);
  }
}

void Write(const std::string &path, const Sound &snd) {
  int numsamples = snd.numsamples;
  int numchannels = snd.numchannels;
  int samplerate = snd.samplerate;
  printf("write %d channels, %d samples\n", numchannels, numsamples);
  SndfileHandle f(path, /*mode=*/SFM_WRITE,
                  /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
                  /*channels=*/numchannels, /*samplerate=*/samplerate);
  f.writef(snd.wav.data(), snd.numsamples);
}

void DriverModel(int n, float *p, int stride) {
  float dpos = 0.0;
  float pos = 0.0;

  const float kSuspension = 0.00039;

  // damping reduces the speed of the membrane passively as it
  // emits energy or converts it to heat in the suspension deformations
  const float damping = 0.99999;

  float hysteresis = 0;
  const float hysteresis_damping = 0.93;
  const float force_to_hysteresis = 3e-4;
  float min_hys = 99;
  float max_hys = -99;

  // Heat in the coil increases resistance.
  float coil_heat = 0;
  const float coil_cooling = 0.9997;
  const float coil_heat_mul = 3e-7;
  float max_heat = 0;

  // Magnetic field gets weaker when the coil is displaced.
  // const float displacement_mul = 1e-35;

  const float kInputMul = 0.3;

  for (int i = 0; i < n; i += stride) {
    float force = 0;
    if (i < n - 2 * stride - 1) {
      force = kInputMul * p[i + stride];
    }

    float v = force;
    for (int k = 0; k < 4; ++k) {
      dpos *= damping;
      dpos += 0.25 * force;
      pos += 0.25 * dpos;
      v += 0.25 * kSuspension * pos;
      pos *= 0.99999;
    }

    coil_heat += v * v;
    coil_heat *= coil_cooling;

    hysteresis += force_to_hysteresis * v;
    hysteresis *= hysteresis_damping;

    min_hys = std::min(min_hys, hysteresis);
    max_hys = std::max(max_hys, hysteresis);
    // hysteresis = min(max(hysteresis, -0.005), 0.005)

    max_heat = std::max(max_heat, coil_heat);
    v *= 1.0 - hysteresis;
    v *= 1.0 + coil_heat * coil_heat_mul;
    // v *= 1.0 + 0.0 * displacement_mul * pow(pos, 4);
    //     printf("%d/%d: %g -> %g\n", i, n, p[i], v);
    p[i] = v;
  }
  printf("hys: %g %g  heat: %g\n", min_hys, max_hys, coil_heat_mul * max_heat);
}

#include <stdlib.h>
#include <unistd.h>

bool SkipChannel(int c, const Sound &snd) {
  return snd.numchannels == 20 && (c == 0 || c == 1 || c == 10 || c == 11);
}

int main(int argc, char **argv) {
  if (argc != 3) {
    fprintf(stderr,
            "Usage: %s in.wav out.wav\n"
            "Only 32 bit float tracks [sox -e float -b 32] are supported\n",
            argv[0]);
    exit(1);
  }
  const char *wav_in = argv[1];
  const char *wav_out = argv[2];
  Sound snd;
  Read(wav_in, &snd);
  for (int c = 0; c < snd.numchannels; ++c) {
    if (SkipChannel(c, snd)) continue;
    fprintf(stderr, "processing channel %d\n", c);
    DriverModel(snd.wav.size(), &snd.wav[c], snd.numchannels);
  }
  Write(wav_out, snd);
  return 0;
}
