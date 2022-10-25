import os
import pydub # python3 -m pip install pydub
import numpy as np
import random

src_dir = os.getcwd()
all_fnames = os.listdir(src_dir)
all_paths = [os.path.join(src_dir, fname) for fname in all_fnames]
file_paths = [path for path in all_paths if os.path.isfile(path)]
mp3_paths = [path for path in file_paths if path.endswith('.mp3')]

NUM_BRANCHES = 16
NUM_CH_PER_BRANCH = 16
NUM_CHANNELS = NUM_BRANCHES * NUM_CH_PER_BRANCH

USB_CHUNK_SIZE = 16 * 1024
USB_ALIGN = int(USB_CHUNK_SIZE / NUM_CHANNELS)

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
  with open(f"{cntr:03}.bin", "wb") as file:
    file.write(samples.tobytes())
  cntr = cntr + 1
