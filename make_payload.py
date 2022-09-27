import scipy.io.wavfile as wavfile
import numpy as np

MAX_SAMPLES = 32768

print(">>> Sample 1")
(rate1, data1) = wavfile.read('sample1.wav')
print(f"Rate: {rate1}")
print(f"Range before: {np.min(data1)}..{np.max(data1)}")
data1 = data1 // 2
data1 = data1 + 16384
data1 = data1.astype('<u2')
print(f"Range after: {np.min(data1)}..{np.max(data1)}")
print(f"Data len: {len(data1)}, data type: {data1.dtype}")

print(">>> Sample 2")
(rate2, data2) = wavfile.read('sample1.wav')
print(f"Rate: {rate2}")
print(f"Range before: {np.min(data2)}..{np.max(data2)}")
data2 = data2 // 2
data2 = data2 + 16384 + 8192 + 3500
data2 = data2.astype('<u2')
print(f"Range after: {np.min(data2)}..{np.max(data2)}")
print(f"Data len: {len(data2)}, data type: {data2.dtype}")

interleaved = np.empty((MAX_SAMPLES + MAX_SAMPLES,), dtype=data1.dtype)
OFFSET = 4096
interleaved[0::2] = data1[OFFSET:MAX_SAMPLES + OFFSET]
interleaved[1::2] = data2[OFFSET:MAX_SAMPLES + OFFSET]

with open("target/payload.bin", "wb") as file:
  file.write(interleaved.tobytes())
