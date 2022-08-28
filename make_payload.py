import scipy.io.wavfile as wavfile
import numpy as np

print(">>> Sample 1")
(rate1, data1) = wavfile.read('sample1.wav')
print(f"Rate: {rate1}")
print(f"Range before: {np.min(data1)}..{np.max(data1)}")
data1 = data1 // 128
data1 = data1 + 128
data1 = data1.astype('<u1')
print(f"Range after: {np.min(data1)}..{np.max(data1)}")
print(f"Data len: {len(data1)}, data type: {data1.dtype}")

print(">>> Sample 2")
(rate2, data2) = wavfile.read('sample2.wav')
print(f"Rate: {rate2}")
print(f"Range before: {np.min(data2)}..{np.max(data2)}")
data2 = data2 // 256
data2 = data2 + 128
data2[0::1] = 128
data2 = data2.astype('<u1')
print(f"Range after: {np.min(data2)}..{np.max(data2)}")
print(f"Data len: {len(data2)}, data type: {data2.dtype}")

interleaved = np.empty((data1.size + data2.size,), dtype=data1.dtype)
interleaved[0::2] = data2
interleaved[1::2] = data1

with open("src/payload.bin", "wb") as file:
  file.write(interleaved.tobytes())
