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
# Quantizes the input signal with 1 bit and dequantizes using leaky integrator
# capacitors.
import sys

import numpy
import scipy.io.wavfile

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


def quantize(signal):
  """Quantizes an audio signal with 1 bit.

  Args:
    signal: Signal to be quantized.

  Returns:
    A list containing the quantized signal samples.
  """
  retval = []
  bit15 = 1 << 15
  error = 0
  for i in range(len(signal) - 1):
    # We sharpen the signal with a filter that amplifies high frequencies,
    # because quantization with 1 bit will attenuate them.
    extrapolated_v = 3 * signal[i] - 2 * signal[i + 1]

    # We repeat _OVERSAMPLE times for every sample of the extrapolated signal.
    # This aims at reducing quantization noise power.
    for _ in range(_OVERSAMPLE):
      inp = extrapolated_v + error
      out = bit15 if inp >= 0 else -bit15
      error = inp - out
      retval.append(out)

  # Append _OVERSAMPLE extra values to result so that its size is
  # len(signal) * _OVERSAMPLE
  inp = signal[len(signal) - 1]
  out = bit15 if inp >= 0 else -bit15
  retval.extend([out] * _OVERSAMPLE)
  return retval


def dequantize(signal):
  """Dequantizes using a filter of 3 leaky integrator capacitors.

     Each integrator with constant k acts as an LTI filter with impulse response
     k*(1-k)**n. The three integrators are cascaded. The dequantized
     signal is derived by the output of the third integrator by averaging every
     _OVERSAMPLE samples.

  Args:
    signal: Signal to be dequantized.

  Returns:
    a list containing the samples of the dequantized signal.
  """
  retval = []
  integrator = 0
  integrator2 = 0
  integrator3 = 0
  k = 0.0253
  k2 = k
  k3 = k

  # The following loop is implementing the filter of integrators using
  # arithmetic operations and reassignments. A better solution could describe
  # the integrators as LTI filters, and calculate their outputs by multiplying
  # the fourier transform of the input signal and the frequency response of the
  # integrator. IFFT of this product wields the desired output.
  # We keep the implementation as is because we want it to correspond to what is
  # going to happen in assembly.
  period = 0
  for i, v in enumerate(signal):
    integrator = integrator * (1.0 - k) + k * v
    integrator2 = integrator2 * (1.0 - k2) + k2 * integrator
    integrator3 = integrator3 * (1.0 - k3) + k3 * integrator2
    period += integrator3
    if i & (_OVERSAMPLE - 1) == _OVERSAMPLE - 1:
      retval.append(period / _OVERSAMPLE)
      period = 0
  return retval


def reduce_frequency(signal):
  """Regroups 0's and 1's together in fixed size windows of the signal.

  Args:
    signal: Signal to be reordered in windows.

  Returns:
    a list containing the samples of the reordered signal.
  """
  # TODO(tseligas): If window_length == _OVERSAMPLING we can try doing
  # quantization and grouping in one step.
  window_length = 32

  # We are padding the signal with zeros to make its length a multiple of
  # window_length
  signal = numpy.asarray(signal)
  target_length = -(-signal.size // window_length) * window_length
  padded_signal = numpy.pad(signal, [(0, target_length - signal.size)])

  windowed_signal = padded_signal.reshape(-1, window_length)
  bit15 = 1 << 15

  return numpy.ravel(numpy.where(
      numpy.arange(windowed_signal.shape[-1])[numpy.newaxis, :] <
      (windowed_signal == -bit15).sum(axis=-1, keepdims=True), -bit15, bit15))


def demo():
  """Demo that reads and processes an audio signal.

     It quantizes with 1 bit and dequantizes using a triple integrator filter.
     It iteratively shifts the quantized signal to the left in order to find
     the offset that minimizes RMSE (root mean square error).
     Finally it groups together the logical low and high values in windows of
     the quantized signal to reduce high frequency components.
  """
  fname = './../testdata/a.wav'
  rate, data = scipy.io.wavfile.read(fname)

  result_quant = quantize(data)
  result = numpy.array(dequantize(result_quant))

  print('Rmse is: ' + str(rmse(data, result)))

  scipy.io.wavfile.write('result.wav', rate, result.astype(numpy.int16))

  # Shift the quantized signal to the left in order to find the offset that
  # minimizes rmse
  min_rmse = sys.float_info.max
  min_offset = 0
  for offset in range(150):
    offset_result = numpy.roll(result_quant, -offset)
    curr_rmse = rmse(data, numpy.array(dequantize(offset_result)))
    print('iteration: ' + str(offset) + ' with rmse: ' + str(curr_rmse))
    if curr_rmse < min_rmse:
      min_rmse = curr_rmse
      min_offset = offset

  result_min_rmse = numpy.roll(result, -min_offset)
  print('Offset that minimizes rmse is: ' + str(min_offset))
  print('Minimumm rmse is: ' + str(min_rmse))

  scipy.io.wavfile.write('result_min_rmse.wav', rate,
                         result_min_rmse.astype(numpy.int16))

  result_quant_reduced = reduce_frequency(result_quant)
  result_reduced = dequantize(result_quant_reduced)

  print('RMSE of signal with frequency reduction: '
        + str(rmse(data, numpy.array(result_reduced[0:len(data)]))))
  scipy.io.wavfile.write('result_reduced.wav', rate,
                         numpy.array(result_reduced).astype(numpy.int16))
