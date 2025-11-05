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
#include <vector>

#define HWY_COMPILE_ONLY_STATIC
#include "hwy/highway.h"
#include "hwy/aligned_allocator.h"
#include "hwy/contrib/algo/transform-inl.h"

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

namespace HWY_NAMESPACE {

namespace hn = hwy::HWY_NAMESPACE;

HWY_ATTR void Process(const int window_size, const int overlap,
                      const int num_channels, SndfileHandle& input_stream_1,
                      SndfileHandle& input_stream_2,
                      SndfileHandle& output_stream_only_1,
                      SndfileHandle& output_stream_only_2,
                      SndfileHandle& output_stream_both) {
  HWY_FULL(float) d;

  const int skip_size = window_size / overlap;
  const auto normalizer = hn::Set(d, 1.f / (window_size * overlap));

  FFTWUniquePtr<fftwf_complex[]> input_fft_1(
      fftwf_alloc_complex(num_channels * (window_size / 2 + 1))),
      input_fft_2(fftwf_alloc_complex(num_channels * (window_size / 2 + 1))),
      center_fft(fftwf_alloc_complex(num_channels * (window_size / 2 + 1)));
  FFTWUniquePtr<float[]> input_1(fftwf_alloc_real(num_channels * window_size)),
      input_2(fftwf_alloc_real(num_channels * window_size)),
      center(fftwf_alloc_real(num_channels * window_size));
  std::fill_n(input_1.get(), num_channels * window_size, 0);
  std::fill_n(input_2.get(), num_channels * window_size, 0);
  hwy::AlignedFreeUniquePtr<float[]>
      output_only_1 = hwy::AllocateAligned<float>(num_channels * window_size),
      output_only_2 = hwy::AllocateAligned<float>(num_channels * window_size),
      output_both = hwy::AllocateAligned<float>(num_channels * window_size);

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

  int64_t read = 0, written = 0, index = 0;
  for (;;) {
    read +=
        std::max(input_stream_1.readf(
                     input_1.get() + num_channels * (window_size - skip_size),
                     skip_size),
                 input_stream_2.readf(
                     input_2.get() + num_channels * (window_size - skip_size),
                     skip_size));
    std::copy_n(input_1.get() + num_channels * (window_size - skip_size),
                skip_size * num_channels,
                output_only_1.get() + num_channels * (window_size - skip_size));
    std::copy_n(input_2.get() + num_channels * (window_size - skip_size),
                skip_size * num_channels,
                output_only_2.get() + num_channels * (window_size - skip_size));
    std::fill_n(output_both.get() + num_channels * (window_size - skip_size),
                skip_size * num_channels, 0);

    fftwf_execute(input_fft_1_plan);
    fftwf_execute(input_fft_2_plan);

    if (Lanes(d) == 1) {
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
    } else {
      hn::Transform2(d, reinterpret_cast<float*>(center_fft.get()),
                     num_channels * (window_size + 2),
                     reinterpret_cast<const float*>(input_fft_1.get()),
                     reinterpret_cast<const float*>(input_fft_2.get()),
                     [](auto d, auto unused, auto first, auto second) HWY_ATTR {
                       auto first_squared = hn::Mul(first, first);
                       auto second_squared = hn::Mul(second, second);
                       auto first_magnitudes_squared =
                           hn::PairwiseAdd(d, first_squared, first_squared);
                       auto second_magnitudes_squared =
                           hn::PairwiseAdd(d, second_squared, second_squared);
                       return hn::IfThenElse(hn::Lt(first_magnitudes_squared,
                                                    second_magnitudes_squared),
                                             first, second);
                     });
    }

    fftwf_execute(center_ifft_plan);

    hn::Transform1(d, output_both.get(), num_channels * window_size,
                   center.get(), [](auto d, auto output, auto center) HWY_ATTR {
                     return hn::Add(output, center);
                   });

    if (index >= window_size - skip_size) {
      {
        size_t i;
        for (i = 0; i + Lanes(d) <= num_channels * skip_size; i += Lanes(d)) {
          auto both = hn::Load(d, &output_both[i]);
          both = hn::Mul(both, normalizer);
          hn::Store(both, d, &output_both[i]);
          auto only_1 = hn::Load(d, &output_only_1[i]);
          auto only_2 = hn::Load(d, &output_only_2[i]);
          hn::Store(hn::Sub(only_1, both), d, &output_only_1[i]);
          hn::Store(hn::Sub(only_2, both), d, &output_only_2[i]);
        }
        if (i < num_channels * skip_size) {
          const size_t n = num_channels * skip_size - i;
          auto both = hn::LoadN(d, &output_both[i], n);
          both = hn::Mul(both, normalizer);
          hn::StoreN(both, d, &output_both[i], n);
          auto only_1 = hn::LoadN(d, &output_only_1[i], n);
          auto only_2 = hn::LoadN(d, &output_only_2[i], n);
          hn::StoreN(hn::Sub(only_1, both), d, &output_only_1[i], n);
          hn::StoreN(hn::Sub(only_2, both), d, &output_only_2[i], n);
        }
      }
      const int64_t to_write = std::min<int64_t>(skip_size, read - written);
      output_stream_only_1.writef(output_only_1.get(), to_write);
      output_stream_only_2.writef(output_only_2.get(), to_write);
      output_stream_both.writef(output_both.get(), to_write);
      written += to_write;
      if (written == read) break;
    }

    for (float* const input : {input_1.get(), input_2.get()}) {
      std::copy(input + num_channels * skip_size,
                input + num_channels * window_size, input);
      std::fill_n(input + num_channels * (window_size - skip_size),
                  num_channels * skip_size, 0);
    }
    for (float* const output :
         {output_only_1.get(), output_only_2.get(), output_both.get()}) {
      std::copy(output + num_channels * skip_size,
                output + num_channels * window_size, output);
      std::fill_n(output + num_channels * (window_size - skip_size),
                  num_channels * skip_size, 0);
    }

    index += skip_size;
  }

  fftwf_destroy_plan(input_fft_1_plan);
  fftwf_destroy_plan(input_fft_2_plan);
  fftwf_destroy_plan(center_ifft_plan);
}

}  // namespace HWY_NAMESPACE

HWY_EXPORT(Process);

}  // namespace

int main(int argc, char **argv) {
  const std::vector<char*> positional_args = absl::ParseCommandLine(argc, argv);

  const int window_size = absl::GetFlag(FLAGS_window_size);
  const int overlap = absl::GetFlag(FLAGS_overlap);

  QCHECK_EQ(window_size % overlap, 0);

  QCHECK_EQ(positional_args.size(), 6)
      << "Usage: " << argv[0]
      << " <input 1> <input 2> <output only 1> <output only 2> <output both>";

  SndfileHandle input_file_1(positional_args[1]);
  QCHECK(input_file_1) << input_file_1.strError();

  SndfileHandle input_file_2(positional_args[2]);
  QCHECK(input_file_2) << input_file_2.strError();

  QCHECK_EQ(input_file_1.channels(), input_file_2.channels());
  QCHECK_EQ(input_file_1.samplerate(), input_file_2.samplerate());
  const int num_channels = input_file_1.channels();
  const int samplerate = input_file_1.samplerate();

  SndfileHandle output_file_only_1(
      positional_args[3], /*mode=*/SFM_WRITE, /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
      /*channels=*/num_channels, /*samplerate=*/samplerate);
  SndfileHandle output_file_only_2(
      positional_args[4], /*mode=*/SFM_WRITE, /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
      /*channels=*/num_channels, /*samplerate=*/samplerate);
  SndfileHandle output_file_both(
      positional_args[5], /*mode=*/SFM_WRITE, /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
      /*channels=*/num_channels, /*samplerate=*/samplerate);

  HWY_STATIC_DISPATCH(Process)(window_size, overlap, num_channels, input_file_1,
                               input_file_2, output_file_only_1,
                               output_file_only_2, output_file_both);
}
