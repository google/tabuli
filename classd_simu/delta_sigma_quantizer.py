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

"""Quantizer - dequantizer script."""
# Quantizes the input signal with 1 bit using delta-sigma modulation and
# dequantizes using a leaky integrator capacitor low pass filter.

import numpy
import scipy.io.wavfile
import scipy.signal

_OVERSAMPLE = 32


def rmse(original, reproduction):
  """Calculates RMSE (root mean square error) between original and reproduction.

  Args:
    original: Original signal (must be array-like).
    reproduction: Reproduced signal after quntization and dequantization (must
    be array-like).

  Returns:
    A scalar RMSE between original and reproduction.
  """
  return numpy.sqrt(
      numpy.mean((numpy.asarray(original) - numpy.asarray(reproduction))**2))


def quantize_delta_sigma(sig):
  """Quantizes sig using 1 bit delta-sigma modulation of second order.

  Args:
    sig: Signal to be quantized

  Returns:
    The binary output of the 1 bit delta sigma modulator
  """
  # Initialize result array
  result = numpy.zeros(len(sig) * _OVERSAMPLE)

  # Make sure we have an array
  sig = numpy.asarray(sig)

  # Upsample the signal with linear interpolation

  # Indices on which we will calculate interpolated values
  x = numpy.arange(len(sig) * _OVERSAMPLE)
  # Indices on which we have known values (multiples of _OVERSAMPLE)
  xp = numpy.arange(len(sig)) * _OVERSAMPLE
  signal_ovs = numpy.interp(x, xp, sig)

  # Possible quantizer outputs.
  d_minus_plus = (-1 << 15, 1<<15)

  integrator = 0
  integrator2 = 0

  for i in range(len(signal_ovs)):
    integrator += signal_ovs[i]
    integrator2_prev = integrator2

    # If integrator is over the threshold, decrease by D and add a pulse to
    # the output
    # Otherwise if it is not above the threshold add D and add a negative pulse.
    delta = d_minus_plus[integrator2_prev > 0]
    result[i] = delta
    integrator -= delta
    integrator2 -= delta

    integrator2 += integrator

  return result


def dequantize_delta_sigma(signal):
  """Dequantizes the binary output of the delta sigma modulator.

  Uses a low pass filter with leaky integrators.

  Args:
    signal: Signal to be dequantized

  Returns:
    The dequantized signal
  """
  retval = numpy.zeros(len(signal) // _OVERSAMPLE)
  integrator = 0
  integrator2 = 0
  integrator3 = 0
  k = 0.15
  k2 = k
  k3 = k
  kc = 1.0 - k
  k2c = 1.0 - k2
  k3c = 1.0 - k3

  # The following loop is implementing the filter of integrators using
  # arithmetic operations and reassignments. A better solution could describe
  # the integrators as LTI filters, and calculate their outputs by multiplying
  # the fourier transform of the input signal and the frequency response of the
  # integrator. IFFT of this product wields the desired output.
  # We keep the implementation as is because we want it to correspond to what is
  # going to happen in assembly.
  period = 0
  for i, v in enumerate(signal):
    integrator = integrator * kc + k * v
    integrator2 = integrator2 * k2c + k2 * integrator
    integrator3 = integrator3 * k3c + k3 * integrator2

    period += integrator3
    if not (i+1) % _OVERSAMPLE:
      retval[(i+1) // _OVERSAMPLE - 1] = period / _OVERSAMPLE
      period = 0
  return retval


def demo():
  """Demo that quantizes and dequantizes an audio signal.
  """
  fname = './../testdata/a.wav'
  rate, data = scipy.io.wavfile.read(fname)
  print('RMS value of signal: ', rmse(data, numpy.zeros(len(data))))

  result_quant = quantize_delta_sigma(data)
  result = dequantize_delta_sigma(result_quant)
  min_length = min(len(data), len(result))
  trimmed_data = data[:min_length]
  trimmed_result = result[:min_length]

  print('RMS error: ', rmse(trimmed_data, trimmed_result))
  scipy.io.wavfile.write('result.wav', rate, result.astype(numpy.int16))
