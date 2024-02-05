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
#include <cstdint>
#include <cstdio>
#include <functional>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/check.h"
#include "fftw3.h"
#include "sndfile.hh"

ABSL_FLAG(bool, autoscale, true,
          "whether to automatically scale the inputs so that the residuals "
          "have equal power");
ABSL_FLAG(int, overlap, 128, "how much to overlap the FFTs");
ABSL_FLAG(int, window_size, 4096, "FFT window size");

namespace {

struct FFTWDeleter {
  void operator()(void* p) const { fftwf_free(p); }
};
template <typename T>
using FFTWUniquePtr = std::unique_ptr<T, FFTWDeleter>;

float SquaredNorm(const fftwf_complex c) { return c[0] * c[0] + c[1] * c[1]; }

float Similarity(
    const int window_size, const int overlap, SndfileHandle& reference_input,
    SndfileHandle& candidate_input, const float candidate_scaling,
    float* reference_minus_candidate_residuals = nullptr,
    const std::function<void()>& start_progress = [] {},
    const std::function<void(int64_t)>& set_progress = [](int64_t) {}) {
  reference_input.seek(0, SEEK_SET);
  candidate_input.seek(0, SEEK_SET);

  const int skip_size = window_size / overlap;
  const float normalizer = 2.f / (window_size * overlap);

  FFTWUniquePtr<fftwf_complex[]> input_fft(
      fftwf_alloc_complex(2 * (window_size / 2 + 1))),
      center_fft(fftwf_alloc_complex(window_size / 2 + 1));
  FFTWUniquePtr<float[]> windowed_input(fftwf_alloc_real(2 * window_size)),
      center(fftwf_alloc_real(window_size));
  std::vector<float> input(2 * window_size, 0);
  std::vector<float> output(3 * window_size);
  std::vector<float> window;

  window.reserve(window_size);
  for (int i = 0; i < window_size; ++i) {
    window.push_back(std::sin((i + .5f) * (M_PI / window_size)));
    window.back() *= window.back();
  }

  fftwf_plan left_right_fft = fftwf_plan_many_dft_r2c(
      /*rank=*/1, /*n=*/&window_size, /*howmany=*/2,
      /*in=*/windowed_input.get(), /*inembed=*/nullptr, /*istride=*/1,
      /*idist=*/window_size, /*out=*/input_fft.get(), /*onembed=*/nullptr,
      /*ostride=*/2, /*odist=*/1, /*flags=*/FFTW_PATIENT | FFTW_DESTROY_INPUT);

  fftwf_plan center_ifft = fftwf_plan_dft_c2r_1d(
      /*n0=*/window_size, /*in=*/center_fft.get(), /*out=*/center.get(),
      /*flags=*/FFTW_MEASURE | FFTW_DESTROY_INPUT);

  float center_power = 0.f, total_power = 0.f;
  if (reference_minus_candidate_residuals != nullptr) {
    *reference_minus_candidate_residuals = 0.f;
  }

  start_progress();
  int64_t read = 0, analyzed = 0, index = 0;
  for (;;) {
    read +=
        std::min(reference_input.readf(input.data() + window_size - skip_size,
                                       skip_size),
                 candidate_input.readf(
                     input.data() + 2 * window_size - skip_size, skip_size));
    for (int i = 0; i < skip_size; ++i) {
      input[2 * window_size - skip_size + i] *= candidate_scaling;
      output[3 * (window_size - skip_size + i)] =
          input[window_size - skip_size + i];
      output[3 * (window_size - skip_size + i) + 1] =
          input[2 * window_size - skip_size + i];
      output[3 * (window_size - skip_size + i) + 2] = 0;
    }

    for (int c = 0; c < 2; ++c) {
      for (int i = 0; i < window_size; ++i) {
        windowed_input[c * window_size + i] =
            window[i] * input[c * window_size + i];
      }
    }

    fftwf_execute(left_right_fft);

    for (int i = 0; i < window_size / 2 + 1; ++i) {
      if (SquaredNorm(input_fft[i * 2]) < SquaredNorm(input_fft[i * 2 + 1])) {
        std::copy_n(input_fft[i * 2], 2, center_fft[i]);
      } else {
        std::copy_n(input_fft[i * 2 + 1], 2, center_fft[i]);
      }
    }

    fftwf_execute(center_ifft);

    for (int i = 0; i < window_size; ++i) {
      output[3 * i + 2] += center[i];
    }

    if (index >= window_size - skip_size) {
      for (int i = 0; i < skip_size; ++i) {
        output[3 * i + 2] *= normalizer;
        output[3 * i] -= output[3 * i + 2];
        output[3 * i + 1] -= output[3 * i + 2];
      }
      const int64_t to_analyze = std::min<int64_t>(skip_size, read - analyzed);
      for (int64_t i = 0; i < to_analyze; ++i) {
        const float center_squared = output[3 * i + 2] * output[3 * i + 2];
        const float left_squared = output[3 * i] * output[3 * i];
        const float right_squared = output[3 * i + 1] * output[3 * i + 1];
        center_power += center_squared;
        total_power += center_squared + left_squared + right_squared;
        if (reference_minus_candidate_residuals != nullptr) {
          *reference_minus_candidate_residuals += left_squared - right_squared;
        }
      }
      analyzed += to_analyze;
      set_progress(analyzed);
      if (analyzed == read) break;
    }

    std::copy(input.begin() + skip_size, input.begin() + window_size,
              input.begin());
    std::fill_n(input.begin() + window_size - skip_size, skip_size, 0);
    std::copy(input.begin() + window_size + skip_size,
              input.begin() + 2 * window_size, input.begin() + window_size);
    std::fill_n(input.begin() + 2 * window_size - skip_size, skip_size, 0);
    std::copy(output.begin() + 3 * skip_size, output.end(), output.begin());
    std::fill_n(output.begin() + 3 * (window_size - skip_size), 3 * skip_size,
                0);

    index += skip_size;
  }

  fftwf_destroy_plan(left_right_fft);
  fftwf_destroy_plan(center_ifft);

  return -10 * std::log10(center_power / total_power);
}

float FindScaling(const int window_size, const int overlap,
                  SndfileHandle& reference_input,
                  SndfileHandle& candidate_input) {
  // Scalings are in log2 scale until the very end.

  float min = 0.f, max = 0.f;

  float scaling = 0.f;
  float difference;
  Similarity(window_size, overlap, reference_input, candidate_input,
             std::exp2(scaling), &difference);
  const bool initial_sign = std::signbit(difference);

  bool have_both_bounds = false;
  do {
    if (initial_sign) {
      // reference - scaling * candidate is negative, so scaling too high
      scaling -= 1.f;
    } else {
      scaling += 1.f;
    }

    Similarity(window_size, overlap, reference_input, candidate_input,
               std::exp2(scaling), &difference);
    if (std::signbit(difference) != initial_sign) {
      have_both_bounds = true;

      if (initial_sign) {
        min = scaling;
        max = scaling + 1.f;
      } else {
        min = scaling - 1.f;
        max = scaling;
      }
    }
  } while (!have_both_bounds);

  while ((max - min) > 1e-2) {
    scaling = .5f * (max + min);
    Similarity(window_size, overlap, reference_input, candidate_input,
               std::exp2(scaling), &difference);
    if (std::signbit(difference)) {
      max = scaling;
    } else {
      min = scaling;
    }
  }

  return std::exp2(.5f * (max + min));
}

}  // namespace

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);

  const int window_size = absl::GetFlag(FLAGS_window_size);
  const int overlap = absl::GetFlag(FLAGS_overlap);

  QCHECK_EQ(window_size % overlap, 0);

  QCHECK_EQ(argc, 3) << "Usage: " << argv[0] << " <reference> <candidate>";

  SndfileHandle reference_input_file(argv[1]);
  QCHECK(reference_input_file) << reference_input_file.strError();
  SndfileHandle candidate_input_file(argv[2]);
  QCHECK(candidate_input_file) << candidate_input_file.strError();
  QCHECK_EQ(reference_input_file.channels(), 1);
  QCHECK_EQ(candidate_input_file.channels(), 1);
  QCHECK_EQ(reference_input_file.samplerate(),
            candidate_input_file.samplerate());

  const float scaling =
      absl::GetFlag(FLAGS_autoscale)
          ? FindScaling(window_size, /*overlap=*/8, reference_input_file,
                        candidate_input_file)
          : 1.f;

  const float similarity = Similarity(
      window_size, overlap, reference_input_file, candidate_input_file, scaling,
      nullptr, [] {}, [](const int64_t written) {});

  printf("%.17f\n", similarity);
}
