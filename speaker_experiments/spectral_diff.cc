// Copyright 2025 Google LLC
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
#include <complex>
#include <functional>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/check.h"
#include "fftw3.h"
#include "sndfile.hh"

ABSL_FLAG(int, overlap, 128, "how much to overlap the FFTs");
ABSL_FLAG(int, window_size, 4096, "FFT window size");

namespace {

struct FFTWDeleter {
  void operator()(void *p) const { fftwf_free(p); }
};
template <typename T>
using FFTWUniquePtr = std::unique_ptr<T, FFTWDeleter>;

float SquaredNorm(const fftwf_complex c) { return c[0] * c[0] + c[1] * c[1]; }

template <typename In, typename Out>
void Process(const int window_size, const int overlap, const int num_channels,
             In &input_stream_1, In &input_stream_2, Out &output_stream_only_1,
             Out &output_stream_only_2, Out &output_stream_both,
             const std::function<void()> &start_progress,
             const std::function<void(int64_t)> &set_progress) {
  const int skip_size = window_size / overlap;
  const float normalizer = 1.f / (window_size * overlap);

  FFTWUniquePtr<fftwf_complex[]> input_fft_1(
      fftwf_alloc_complex(num_channels * (window_size / 2 + 1))),
      input_fft_2(fftwf_alloc_complex(num_channels * (window_size / 2 + 1))),
      center_fft(fftwf_alloc_complex(num_channels * (window_size / 2 + 1)));
  FFTWUniquePtr<float[]> input_1(fftwf_alloc_real(num_channels * window_size)),
      input_2(fftwf_alloc_real(num_channels * window_size)),
      center(fftwf_alloc_real(num_channels * window_size));
  std::fill_n(input_1.get(), num_channels * window_size, 0);
  std::fill_n(input_2.get(), num_channels * window_size, 0);
  std::vector<float> output_only_1(num_channels * window_size),
      output_only_2(num_channels * window_size),
      output_both(num_channels * window_size);

  fftwf_plan input_fft_1_plan = fftwf_plan_many_dft_r2c(
      /*rank=*/1, /*n=*/&window_size, /*howmany=*/num_channels,
      /*in=*/input_1.get(), /*inembed=*/nullptr, /*istride=*/num_channels,
      /*idist=*/1, /*out=*/input_fft_1.get(), /*onembed=*/nullptr,
      /*ostride=*/num_channels, /*odist=*/1,
      /*flags=*/FFTW_PATIENT | FFTW_PRESERVE_INPUT);

  fftwf_plan input_fft_2_plan = fftwf_plan_many_dft_r2c(
      /*rank=*/1, /*n=*/&window_size, /*howmany=*/num_channels,
      /*in=*/input_2.get(), /*inembed=*/nullptr, /*istride=*/num_channels,
      /*idist=*/1, /*out=*/input_fft_2.get(), /*onembed=*/nullptr,
      /*ostride=*/num_channels, /*odist=*/1,
      /*flags=*/FFTW_PATIENT | FFTW_PRESERVE_INPUT);

  fftwf_plan center_ifft_plan = fftwf_plan_many_dft_c2r(
      /*rank=*/1, /*n=*/&window_size, /*how_many=*/num_channels,
      /*in=*/center_fft.get(), /*inembed=*/nullptr, /*istride=*/num_channels,
      /*idist=*/1, /*out=*/center.get(), /*onembed=*/nullptr,
      /*ostride=*/num_channels, /*odist=*/1,
      /*flags=*/FFTW_MEASURE | FFTW_DESTROY_INPUT);

  start_progress();
  int64_t read = 0, written = 0, index = 0;
  for (;;) {
    read +=
        std::max(input_stream_1.readf(
                     input_1.get() + num_channels * (window_size - skip_size),
                     skip_size),
                 input_stream_2.readf(
                     input_2.get() + num_channels * (window_size - skip_size),
                     skip_size));
    std::copy_n(
        input_1.get() + num_channels * (window_size - skip_size),
        skip_size * num_channels,
        output_only_1.begin() + num_channels * (window_size - skip_size));
    std::copy_n(
        input_2.get() + num_channels * (window_size - skip_size),
        skip_size * num_channels,
        output_only_2.begin() + num_channels * (window_size - skip_size));
    std::fill_n(output_both.begin() + num_channels * (window_size - skip_size),
                skip_size * num_channels, 0);

    fftwf_execute(input_fft_1_plan);
    fftwf_execute(input_fft_2_plan);

    for (int i = 0; i < window_size / 2 + 1; ++i) {
      for (int c = 0; c < num_channels; ++c) {
        if (SquaredNorm(input_fft_1[i * num_channels + c]) <
            SquaredNorm(input_fft_2[i * num_channels + c])) {
          std::copy_n(input_fft_1[i * num_channels + c], 2,
                      center_fft[i * num_channels + c]);
        } else {
          std::copy_n(input_fft_2[i * num_channels + c], 2,
                      center_fft[i * num_channels + c]);
        }
      }
    }

    fftwf_execute(center_ifft_plan);

    for (int i = 0; i < window_size; ++i) {
      for (int c = 0; c < num_channels; ++c) {
        output_both[i * num_channels + c] += center[i * num_channels + c];
      }
    }

    if (index >= window_size - skip_size) {
      for (int i = 0; i < skip_size; ++i) {
        for (int c = 0; c < num_channels; ++c) {
          output_both[i * num_channels + c] *= normalizer;
          output_only_1[i * num_channels + c] -=
              output_both[i * num_channels + c];
          output_only_2[i * num_channels + c] -=
              output_both[i * num_channels + c];
        }
      }
      const int64_t to_write = std::min<int64_t>(skip_size, read - written);
      output_stream_only_1.writef(output_only_1.data(), to_write);
      output_stream_only_2.writef(output_only_2.data(), to_write);
      output_stream_both.writef(output_both.data(), to_write);
      written += to_write;
      set_progress(written);
      if (written == read) break;
    }

    for (float *const input : {input_1.get(), input_2.get()}) {
      std::copy(input + num_channels * skip_size,
                input + num_channels * window_size, input);
      std::fill_n(input + num_channels * (window_size - skip_size),
                  num_channels * skip_size, 0);
    }
    for (std::vector<float> *const output :
         {&output_only_1, &output_only_2, &output_both}) {
      std::copy(output->begin() + num_channels * skip_size, output->end(),
                output->begin());
      std::fill_n(output->begin() + num_channels * (window_size - skip_size),
                  num_channels * skip_size, 0);
    }

    index += skip_size;
  }

  fftwf_destroy_plan(input_fft_1_plan);
  fftwf_destroy_plan(input_fft_2_plan);
  fftwf_destroy_plan(center_ifft_plan);
}

}  // namespace

int main(int argc, char **argv) {
  absl::ParseCommandLine(argc, argv);

  const int window_size = absl::GetFlag(FLAGS_window_size);
  const int overlap = absl::GetFlag(FLAGS_overlap);

  QCHECK_EQ(window_size % overlap, 0);

  QCHECK_EQ(argc, 6)
      << "Usage: " << argv[0]
      << " <input 1> <input 2> <output only 1> <output only 2> <output both>";

  SndfileHandle input_file_1(argv[1]);
  QCHECK(input_file_1) << input_file_1.strError();

  SndfileHandle input_file_2(argv[2]);
  QCHECK(input_file_2) << input_file_2.strError();

  QCHECK_EQ(input_file_1.channels(), input_file_2.channels());
  QCHECK_EQ(input_file_1.samplerate(), input_file_2.samplerate());
  const int num_channels = input_file_1.channels();
  const int samplerate = input_file_1.samplerate();

  SndfileHandle output_file_only_1(
      argv[3], /*mode=*/SFM_WRITE, /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
      /*channels=*/num_channels, /*samplerate=*/samplerate);
  SndfileHandle output_file_only_2(
      argv[4], /*mode=*/SFM_WRITE, /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
      /*channels=*/num_channels, /*samplerate=*/samplerate);
  SndfileHandle output_file_both(
      argv[5], /*mode=*/SFM_WRITE, /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
      /*channels=*/num_channels, /*samplerate=*/samplerate);

  Process(
      window_size, overlap, num_channels, input_file_1, input_file_2,
      output_file_only_1, output_file_only_2, output_file_both, [] {},
      [](const int64_t written) {});
}
