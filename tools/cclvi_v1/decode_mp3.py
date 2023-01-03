import os
import pydub # python3 -m pip install pydub
import numpy as np

SD = 1

src_dir = os.getcwd()
all_fnames = os.listdir(src_dir)
all_paths = [os.path.join(src_dir, fname) for fname in all_fnames]
file_paths = [path for path in all_paths if os.path.isfile(path)]
mp3_paths = [path for path in file_paths if path.endswith('.mp3')]

NUM_BRANCHES = 16
NUM_CH_PER_BRANCH = 16
NUM_CHANNELS = NUM_BRANCHES * NUM_CH_PER_BRANCH
PACKET_SIZE = 2 * NUM_CHANNELS

USB_CHUNK_SIZE = 16 * 1024
USB_ALIGN = int(USB_CHUNK_SIZE / PACKET_SIZE)

TARGET_RATE = 44100
TARGET_LEN_SEC = 60

# Truncate to ensure whole number of USB packets is used
TARGET_LEN = (TARGET_RATE * TARGET_LEN_SEC) & ~(USB_ALIGN - 1)

cntr = 0
for path in mp3_paths:
  audio_segment = pydub.AudioSegment.from_mp3(path)
  if (audio_segment.frame_rate != TARGET_RATE):
    print(f'Frame rate is not {TARGET_RATE}; ignored: {path}')
    continue
  samples = np.array(audio_segment.get_array_of_samples())
  if audio_segment.channels == 2:
    samples = samples.reshape((-1, 2))[:, 0]
  elif audio_segment.channels == 1:
    pass
  else:
    print(f'One/two channels expected, but got {audio_segment.channels}; ignored: {path}')
    continue
  if len(samples) < TARGET_LEN:
    print(f'Sample is too short: {len(samples)} < {TARGET_LEN}; ignored: {path}')
    continue
  samples = samples[:TARGET_LEN].astype('<i4')
  samples += 32768
  samples = samples.astype('<u2')
  if not SD:
    with open(f"{cntr:03}.pcm16", "wb") as file:
      file.write(samples.tobytes())
  else:
    # 1 sample -> 64 -> 8 bytes
    sd_samples = np.zeros(TARGET_LEN * 8, np.uint8)
    state = 0
    for i in range(TARGET_LEN):
      v = samples[i]
      for by in range(8):
        b = 0
        for bi in range(8):
          b = b >> 1
          state = state + v
          if state >= 65536:
            state = state - 65536
            b = b | 128
        sd_samples[i * 8 + by] = b
    with open(f"{cntr:03}.dsd64", "wb") as file:
      file.write(sd_samples.tobytes())
  cntr = cntr + 1
