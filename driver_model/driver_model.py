# Copyright 2024 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Explorative code for studying loudspeaker physics.

This is not a software component to be used in larger engineering
designs as-is.

This algorithm will likely be integrated in
speaker_experiments/virtual_speakers in C++.
"""


import numpy
import scipy
import scipy.io
import scipy.io.wavfile
import scipy.ndimage


# Read the wave using scipy

samplerate, input_wav = scipy.io.wavfile.read('/tmp/1.wav')
input_wav = input_wav.astype(float)
#input_wav *= 1/32768 # 32 bit samples need scaling down

def fade(wav):
  for i in range(min(10000, len(wav))):
    mul = 1.0 - i / 10000.;
    wav[wav.shape[0] - i - 1][0] = 0
    wav[wav.shape[0] - i - 1][1] = 0

fade(input_wav)
print(input_wav.shape[0])

print("gaussian")
#blurred = scipy.ndimage.gaussian_filter1d(input_wav, 330.0, 0, 0)
#input_wav -= blurred
blurred = scipy.ndimage.gaussian_filter1d(input_wav, 440.0, 0, 0)
input_wav -= blurred
print("end gaussian")


output = numpy.zeros(input_wav.shape)

# parameters
# suspension strength is basically a correction level multiplier
suspension = 0.00034

# damping reduces the speed of the membrane passively as it
# emits energy or converts it to heat in the suspension deformations
damping = 0.99975

currpow = 0.0

hysteresis = 0
hysteresis_damping = 0.9
force_to_hysteresis = 9e-10
min_hys = 99
max_hys = -99

coil_heat = 0
coil_cooling = 0.9997
coil_heat_mul = 3e-15
max_heat = 0

displacement_mul = 1e-9

for c in range(input_wav.shape[1]):
  print("loopi", c)
  dpos = 0.0
  pos = 0.0
  for i in range(input_wav.shape[0]):
    force = 0
    if (i >= 2):
        force = (-0.01 * (input_wav[i-2][c] + input_wav[i][c]) +
               1.02 * input_wav[i-1][c])

    v = force
    for k in range(4):
      dpos *= damping
      dpos += 0.25 * force
      pos += 0.25 * dpos;
      v += 0.25 * suspension * pos
      pos *= 0.99825


    #print (v)
    coil_heat += abs(v * v)
    coil_heat *= coil_cooling

    hysteresis += force_to_hysteresis * v
    hysteresis *= hysteresis_damping

    min_hys = min(min_hys, hysteresis)
    max_hys = max(max_hys, hysteresis)
    #hysteresis = min(max(hysteresis, -0.005), 0.005)

    max_heat = max(max_heat, coil_heat)
    output[i][c] = v
    output[i][c] *= 1.0 - hysteresis
    output[i][c] *= 1.0 + coil_heat * coil_heat_mul
    #output[i][c] *= 1.0 + 1e-39 * abs(pos) ** 4


print("max stats", min_hys, max_hys, max_heat * coil_heat_mul)

output *= 0.3 / 32768.

scipy.io.wavfile.write('/tmp/o.wav', samplerate, output)
