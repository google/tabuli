
#include "absl/random/random.h"
#include "absl/strings/match.h"
#include "absl/strings/string_view.h"

#include <cstdio>
#include <stddef.h>
#include <stdint.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

namespace {

constexpr size_t NUM_ENDPOINTS = 16;
constexpr size_t NUM_CH_PER_ENDPOINT = 16;
constexpr size_t NUM_CHANNELS = NUM_ENDPOINTS * NUM_CH_PER_ENDPOINT;

constexpr size_t PACKET_SIZE = NUM_CHANNELS * 2; // 2 bytes per sample per channel
constexpr size_t CHUNK_SIZE = NUM_ENDPOINTS * 2;
constexpr size_t USB_CHUNK_SIZE = 16 * 1024;
constexpr size_t USB_ALIGN = USB_CHUNK_SIZE / PACKET_SIZE;

static_assert((USB_ALIGN & (USB_ALIGN - 1)) == 0,
              "USB_ALIGN is not power of two");

constexpr size_t TARGET_RATE = 44100;
constexpr size_t TARGET_LEN_SEC = 60;

// Truncate to ensure whole number of USB packets is used
constexpr size_t TARGET_LEN = (TARGET_RATE * TARGET_LEN_SEC) & ~(USB_ALIGN - 1);

// Stem stream structure:
// packet = 16 chunks: 1 chunk per channel on endpoint
// chunk = 16 words: 1 word per bit of sample (MSB)
// word = 16 bits: 1 bit per endpoint

// Branch stream structure:
// packet = 16 words, one per sample per channel

std::vector<uint16_t> readFile(absl::string_view fname) {
  std::vector<uint16_t> result(TARGET_LEN);
  std::ifstream file(fname.cbegin(), std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    fprintf(stderr, "Failed to open %s\n", fname.cbegin());
    __builtin_trap();
  }
  file.seekg(0, std::ios::beg);
  // 2 bytes per sample
  size_t byteLen = result.size() * 2;
  file.read(reinterpret_cast<char *>(result.data()), byteLen);
  if (!file.good()) {
    fprintf(stderr, "Failed to read %s\n", fname.cbegin());
    __builtin_trap();
  }
  file.close();
  return result;
}

} // namespace

int main(int argc, char **argv) {
  std::vector<std::vector<uint16_t>> input;
  for (const auto &entry : std::__fs::filesystem::directory_iterator(".")) {
    absl::string_view path = entry.path().c_str();
    if (!absl::EndsWith(path, ".pcm16")) {
      continue;
    }
    fprintf(stderr, "Loading %s\n", path.cbegin());
    input.emplace_back(readFile(path.cbegin()));
  }

  std::vector<size_t> sampleMap(NUM_CHANNELS);
  {
    absl::BitGen rng;
    size_t num_samples = input.size();
    for (size_t i = 0; i < NUM_CHANNELS; ++i) {
      sampleMap[i] = absl::Uniform<size_t>(rng, 0, num_samples);
    }
  }

  std::vector<uint8_t> output(TARGET_LEN * PACKET_SIZE);
  const uint16_t *inputMap[NUM_CHANNELS];
  for (size_t c = 0; c < NUM_CH_PER_ENDPOINT; ++c) {
    for (size_t p = 0; p < NUM_ENDPOINTS; ++p) {
      inputMap[c * NUM_ENDPOINTS + p] =
          input[sampleMap[c * NUM_ENDPOINTS + p]].data();
    }
  }

  double phase = 0.0;
  for (size_t s = 0; s < TARGET_LEN; ++s) {
    uint16_t src[NUM_CHANNELS];
    for (size_t c = 0; c < NUM_CHANNELS; ++c) {
#if 1
      src[c] = inputMap[c][s];
#else
      src[c] = 65535 * (0.5 * (sin(phase) + 1)); //inputMap[c][s];
#endif
    }
    double freq = 220.0 * (20 * s + TARGET_LEN + 0.0) / TARGET_LEN;
    phase += freq * 2 * 3.14159265359 / 44100.0;
    // 2 == bytes per sample
    size_t sOffset = s * PACKET_SIZE;
    for (size_t c = 0; c < NUM_CH_PER_ENDPOINT; ++c) { // 16 channels per branch
      // 2 == bytes per sample
      size_t cOffset = sOffset + c * CHUNK_SIZE; // 32 bytes per sample
      const uint16_t *samples = src + c * NUM_ENDPOINTS;
      // 16 == number of bits; by lucky coincedence == NUM_ENDPOINTS
      for (size_t w = 0; w < 16; ++w) {
        uint16_t result = 0;
        for (size_t p = 0; p < NUM_ENDPOINTS; ++p) {
          uint16_t sample = samples[p];
          uint16_t sampleBit = (sample >> (15 - w)) & 1;
          result |= sampleBit << p;
        }
        output[cOffset + 2 * w] = result & 0xFF;
        output[cOffset + 2 * w + 1] = result >> 8;
      }
    }
    if (((s + 1) & 0xFFFF) == 0) {
      fprintf(stderr, "Processed %0.2fs\n", s / 44100.0f);
    }
  }

  fprintf(stderr, "Writing output\n");

  std::string fname = "snd.mux";
  std::ofstream file(fname.c_str(), std::ios::out | std::ios::binary);
  if (!file.is_open()) {
    fprintf(stderr, "Failed to open %s\n", fname.c_str());
    __builtin_trap();
  }
  size_t byteLen = output.size();
  file.write(reinterpret_cast<char*>(output.data()), byteLen);
  if (!file.good()) {
    fprintf(stderr, "Failed to write %s\n", fname.c_str());
    __builtin_trap();
  }
  file.close();

  fprintf(stderr, "Done\n");

  return 0;
}