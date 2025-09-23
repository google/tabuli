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
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "absl/log/check.h"
#include "fourier_bank.h"
#include "sndfile.hh"

namespace tabuli {

class InputSignal {
 public:
  InputSignal(const char *path) {
    input_file_ = std::make_unique<SndfileHandle>(path);
    if (!*input_file_) {
      fprintf(stderr, "cannot open input path: %s\n", path);
      exit(2);
    }
    channels_ = input_file_->channels();
    samplerate_ = input_file_->samplerate();
  }

  size_t channels() const { return channels_; }
  size_t samplerate() const { return samplerate_; }

  int64_t readf(float* data, size_t nframes) {
    int64_t read = input_file_->readf(data, nframes);
    return read;
  }

 private:
  size_t channels_;
  size_t samplerate_;
  std::unique_ptr<SndfileHandle> input_file_;
};

class OutputSignal {
 public:
  OutputSignal(size_t channels, size_t freq_channels, size_t samplerate)
    : channels_(channels),
      freq_channels_(freq_channels),
      samplerate_(samplerate) { }

  void writef(const float* data, size_t nframes) {
    if (output_file_) {
      output_file_->writef(data, nframes);
    }
  }

  void SetWavFile(const char* fn) {
    output_file_ = std::make_unique<SndfileHandle>(
        fn, /*mode=*/SFM_WRITE, /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
        channels_, samplerate_);
  }
  
  size_t channels() const { return channels_; }
  size_t frame_size() const { return channels_ * freq_channels_; }

 private:
  size_t channels_;
  size_t freq_channels_;
  size_t samplerate_;
  std::unique_ptr<SndfileHandle> output_file_;
};

template <typename In, typename Out>
void Process(
    In& input_stream, Out& output_stream,
    const std::vector<float>& filter_gains = {},
    const std::function<void()>& start_progress = [] {},
    const std::function<void(int64_t)>& set_progress = [](int64_t written) {}) {
  const size_t num_channels = input_stream.channels();
  std::vector<float> history(num_channels * kHistorySize);
  std::vector<float> input(num_channels * kBlockSize);
  std::vector<float> output(output_stream.frame_size() * kBlockSize);
  
  RotatorFilterBank rotbank(kNumRotators, num_channels,
                            input_stream.samplerate(), /*num_threads=*/1,
                            filter_gains, 1.0);
  
  start_progress();
  int64_t total_in = 0;
  int64_t total_out = 0;
  bool done = false;
  while (!done) {
    int64_t read = input_stream.readf(input.data(), kBlockSize);
    if (read == 0) {
      done = true;
      read = total_in - total_out;
      std::fill(input.begin(), input.begin() + read, 0);
    }
    for (int i = 0; i < read; ++i) {
      int input_ix = i + total_in;
      size_t histo_ix = num_channels * (input_ix & kHistoryMask);
      for (size_t c = 0; c < num_channels; ++c) {
        history[histo_ix + c] = input[num_channels * i + c];
      }
    }
    int64_t output_len = 0;
    output_len = rotbank.FilterAllSingleThreaded(history.data(), total_in, read, output.data(), output.size());
    output_stream.writef(output.data(), output_len);
    total_in += read;
    total_out += output_len;
    set_progress(total_in);
  }
};

}  // namespace tabuli

using namespace tabuli;

int main(int argc, char** argv) {
  if(argc < 3) {
    fprintf(stderr, "Usage: %s in.wav out.wav\n", argv[0]);
    exit(1);
  }
  InputSignal input(argv[1]);
  size_t freq_channels = 1;
  OutputSignal output(input.channels(), freq_channels, input.samplerate());
  output.SetWavFile(argv[2]);
  std::vector<float> filter_gains;
  for (int i = 0; i < kNumRotators; ++i) {
    filter_gains.push_back(GetRotatorGains(i));
  }
  Process(input, output, filter_gains);
}
