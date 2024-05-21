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
#include <complex>
#include <functional>
#include <iterator>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/check.h"
#include "fftw3.h"
#include "sndfile.hh"

namespace {

constexpr int kSubSourcePrecision = 10;

struct FFTWDeleter {
  void operator()(void* p) const { fftwf_free(p); }
};
template <typename T>
using FFTWUniquePtr = std::unique_ptr<T, FFTWDeleter>;

float SquaredNorm(const fftwf_complex c) { return c[0] * c[0] + c[1] * c[1]; }

float MicrophoneResponse(const float angle) {
  return 0.5f * (1.25f + std::cos(angle));
}

float ExpectedLeftToRightRatio(const float angle) {
  return (1e-3 + MicrophoneResponse(angle + M_PI / 4)) /
         (1e-3 + MicrophoneResponse(angle - M_PI / 4));
}

float ActualLeftToRightRatio(const fftwf_complex left,
                             const fftwf_complex right) {
  return std::sqrt((1e-3 + SquaredNorm(left)) / (1e-3 + SquaredNorm(right)));
}

template <typename In, typename Out>
void Process(
    const int window_size, const int overlap, const int output_channels,
    const float distance_to_interval_ratio, In& input_stream,
    Out& output_stream, const std::function<void()>& start_progress = [] {},
    const std::function<void(int64_t)>& set_progress = [](int64_t written) {}) {
  const int skip_size = window_size / overlap;
  const float normalizer = 2.f / (window_size * overlap);

  FFTWUniquePtr<fftwf_complex[]> input_fft(
      fftwf_alloc_complex(2 * (window_size / 2 + 1))),
      output_fft(fftwf_alloc_complex(output_channels * (window_size / 2 + 1)));
  FFTWUniquePtr<float[]> windowed_input(fftwf_alloc_real(2 * window_size)),
      synthesized_output(fftwf_alloc_real(output_channels * window_size));
  std::vector<float> input(2 * window_size),
      output(output_channels * window_size);

  fftwf_plan left_right_fft = fftwf_plan_many_dft_r2c(
      /*rank=*/1, /*n=*/&window_size, /*howmany=*/2,
      /*in=*/windowed_input.get(), /*inembed=*/nullptr, /*istride=*/2,
      /*idist=*/1, /*out=*/input_fft.get(), /*onembed=*/nullptr, /*ostride=*/2,
      /*odist=*/1, /*flags=*/FFTW_PATIENT | FFTW_DESTROY_INPUT);

  fftwf_plan output_ifft = fftwf_plan_many_dft_c2r(
      /*rank=*/1, /*n=*/&window_size, /*howmany=*/output_channels,
      /*in=*/output_fft.get(), /*inembed=*/nullptr, /*istride=*/output_channels,
      /*idist=*/1, /*out=*/synthesized_output.get(), /*onembed=*/nullptr,
      /*ostride=*/output_channels, /*odist=*/1,
      /*flags=*/FFTW_PATIENT | FFTW_DESTROY_INPUT);

  std::vector<float> speaker_to_ratio_table;
  speaker_to_ratio_table.reserve(kSubSourcePrecision * (output_channels - 1) +
                                 1);
  for (int i = 0; i < kSubSourcePrecision * (output_channels - 1) + 1; ++i) {
    const float x_div_interval = static_cast<float>(i) / kSubSourcePrecision -
                                 0.5f * (output_channels - 1);
    const float x_div_distance = x_div_interval / distance_to_interval_ratio;
    const float angle = std::atan(x_div_distance);
    speaker_to_ratio_table.push_back(ExpectedLeftToRightRatio(angle));
  }

  std::vector<float> window_function;
  window_function.reserve(window_size);
  for (int i = 0; i < window_size; ++i) {
    const float sine = std::sin(i * M_PI / (window_size - 1));
    window_function.push_back(sine * sine);
  }

  start_progress();
  int64_t read = 0, written = 0, index = 0;
  for (;;) {
    read += input_stream.readf(input.data() + 2 * (window_size - skip_size),
                               skip_size);

    for (int i = 0; i < window_size; ++i) {
      windowed_input[2 * i] = window_function[i] * input[2 * i];
      windowed_input[2 * i + 1] = window_function[i] * input[2 * i + 1];
    }

    fftwf_execute(left_right_fft);

    for (int i = 0; i < output_channels * (window_size / 2 + 1); ++i) {
      std::fill(std::begin(output_fft[i]), std::end(output_fft[i]), 0.f);
    }

    for (int i = 0; i < window_size / 2 + 1; ++i) {
      const float ratio =
          ActualLeftToRightRatio(input_fft[2 * i], input_fft[2 * i + 1]);
      const int subspeaker_index =
          std::lower_bound(speaker_to_ratio_table.begin(),
                           speaker_to_ratio_table.end(), ratio,
                           std::greater<>()) -
          speaker_to_ratio_table.begin();

      // amp-kludge to make borders louder -- it is a virtual line array where
      // the borders will be further away in rendering, so let's compensate for
      // it here.

      float distance_from_center =
          (subspeaker_index - 0.5 * (output_channels - 1));
      float assumed_distance_to_line = 0.75 * (output_channels - 1);
      float distance_to_virtual =
          sqrt(distance_from_center * distance_from_center +
               assumed_distance_to_line * assumed_distance_to_line);
      float dist_ratio =
          distance_to_virtual * (1.0f / assumed_distance_to_line);
      float amp = dist_ratio * dist_ratio;

      const float index =
          static_cast<float>(subspeaker_index) / kSubSourcePrecision;
      float integral_index_f;
      const float fractional_index = std::modf(index, &integral_index_f);
      const int integral_index = integral_index_f;
      const fftwf_complex source_coefficient = {
          0.5f * (input_fft[2 * i][0] + input_fft[2 * i + 1][0]),
          0.5f * (input_fft[2 * i][1] + input_fft[2 * i + 1][1])};
      const float a = amp * (1 - fractional_index);
      const float b = amp * (fractional_index);
      output_fft[i * output_channels + integral_index][0] =
          a * source_coefficient[0];
      output_fft[i * output_channels + integral_index][1] =
          a * source_coefficient[1];
      output_fft[i * output_channels + integral_index + 1][0] =
          b * source_coefficient[0];
      output_fft[i * output_channels + integral_index + 1][1] =
          b * source_coefficient[1];
    }

    fftwf_execute(output_ifft);

    for (int i = 0; i < output_channels * window_size; ++i) {
      output[i] += synthesized_output[i];
    }

    if (index >= window_size - skip_size) {
      for (int i = 0; i < output_channels * skip_size; ++i) {
        output[i] *= normalizer;
      }
      const int64_t to_write = std::min<int64_t>(skip_size, read - written);
      output_stream.writef(output.data(), to_write);
      written += to_write;
      set_progress(written);
      if (written == read) break;
    }

    std::copy(input.begin() + 2 * skip_size, input.begin() + 2 * window_size,
              input.begin());
    std::fill_n(input.begin() + 2 * (window_size - skip_size), 2 * skip_size,
                0);
    std::copy(output.begin() + output_channels * skip_size, output.end(),
              output.begin());
    std::fill_n(output.begin() + output_channels * (window_size - skip_size),
                output_channels * skip_size, 0);

    index += skip_size;
  }

  fftwf_destroy_plan(left_right_fft);
  fftwf_destroy_plan(output_ifft);
}

}  // namespace

ABSL_FLAG(int, overlap, 64, "how much to overlap the FFTs");
ABSL_FLAG(int, window_size, 4096, "FFT window size");
ABSL_FLAG(int, output_channels, 120, "number of output channels");
ABSL_FLAG(float, distance_to_interval_ratio, 4,
          "ratio of (distance between microphone and source array) / (distance "
          "between each source); default = 40cm / 10cm = 4");

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);

  const int window_size = absl::GetFlag(FLAGS_window_size);
  const int overlap = absl::GetFlag(FLAGS_overlap);
  const int output_channels = absl::GetFlag(FLAGS_output_channels);
  const float distance_to_interval_ratio =
      absl::GetFlag(FLAGS_distance_to_interval_ratio);

  QCHECK_EQ(window_size % overlap, 0);

  //  QCHECK_EQ(argc, 3) << "Usage: " << argv[0] << " <input> <output>";

  SndfileHandle input_file(argv[1]);
  QCHECK(input_file) << input_file.strError();

  QCHECK_EQ(input_file.channels(), 2);

  SndfileHandle output_file(
      argv[2], /*mode=*/SFM_WRITE, /*format=*/SF_FORMAT_WAV | SF_FORMAT_PCM_24,
      /*channels=*/output_channels, /*samplerate=*/input_file.samplerate());

  Process(
      window_size, overlap, output_channels, distance_to_interval_ratio,
      input_file, output_file, [] {}, [](const int64_t written) {});
}
