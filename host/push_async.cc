#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <libusb.h>

namespace {

template <typename Callback> class ScopeGuard {
public:
  // Discourage unnecessary moves / copies.
  ScopeGuard(const ScopeGuard &) = delete;
  ScopeGuard &operator=(const ScopeGuard &) = delete;
  ScopeGuard &operator=(ScopeGuard &&) = delete;

  // Pre-C++17 does not guarantee RVO -> require move constructor.
  ScopeGuard(ScopeGuard &&other) : callback_(std::move(other.callback_)) {}

  template <typename CallbackParam>
  explicit ScopeGuard(CallbackParam &&callback)
      : callback_(std::forward<CallbackParam>(callback)) {}

  ~ScopeGuard() { callback_(); }

private:
  Callback callback_;
};

template <typename Callback>
ScopeGuard<Callback> MakeScopeGuard(Callback &&callback) {
  return ScopeGuard<Callback>{std::forward<Callback>(callback)};
}

std::chrono::steady_clock::time_point start;
#define NUM_TRANSFERS 3
libusb_transfer *transfers[NUM_TRANSFERS] = {0};
#define CHUNK_SIZE (16 * 1024)
unsigned char *tx_buf = nullptr;
std::vector<unsigned char> tx_buffers(CHUNK_SIZE *NUM_TRANSFERS);
size_t tx_buf_len = 0;
size_t free_transfer;
size_t num_chunks = 0;
size_t next_offset = 0;
size_t msg_id = 0;

void on_transfer_complete(struct libusb_transfer *transfer);

#define EIGHT_TO_SIX 0

#if EIGHT_TO_SIX
#define DATA_MASK 0x7F
const auto encode_byte = [](uint8_t b) {
  // simple, but works well only for for 0..189
  if (b > 189) {
    return 0;
  }
  return b + ((b + 1) >> 7);
};

const auto decode_byte = [](uint8_t b) { return b - (b >> 7); };
// 4x version: u32 - ((u32 & 0x80808080) >> 7)
#else
#define DATA_MASK 0xFF
const auto encode_byte = [](uint8_t b) { return b; };
const auto decode_byte = [](uint8_t b) { return b; };
#endif

void prepareTxBuffer(size_t idx, struct libusb_device_handle *usb_dev) {
  for (size_t i = 0; i < CHUNK_SIZE; ++i) {
    tx_buffers[idx * CHUNK_SIZE + i] =
        encode_byte(tx_buf[next_offset + i] & DATA_MASK);
  }

  libusb_fill_bulk_transfer(transfers[idx], usb_dev, /* endpoint */ 0x02,
                            tx_buffers.data() + idx * CHUNK_SIZE, CHUNK_SIZE,
                            &on_transfer_complete,
                            /* user data */ (void *)idx, /* timeout */ 0);
  next_offset += CHUNK_SIZE;
  if (next_offset >= tx_buf_len) {
    next_offset = 0;
  }
}

void on_transfer_complete(struct libusb_transfer *transfer) {
  auto end = std::chrono::high_resolution_clock::now();
  auto delta =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start)
          .count();
  num_chunks++;
  size_t this_transfer = (size_t)transfer->user_data;
  if ((num_chunks & 0xFF) == 0) {
    float size = num_chunks * CHUNK_SIZE / 1024.0 / 1024.0;
    float speed = size / (static_cast<size_t>(delta) / 1000000.0);
    fprintf(stderr,
            "%04zX | sent: %0.1fMiB, time: %0.3fms, speed: %0.3fMiB/s\n",
            msg_id++, size, delta / 1000.0f, speed);
    num_chunks = 0;
    start = end;
  }

  prepareTxBuffer(free_transfer, transfer->dev_handle);
  libusb_submit_transfer(transfers[free_transfer]);

  free_transfer = this_transfer;
}

/*void readFile(const char *fname, std::vector<unsigned char> *out) {
  std::ifstream file(fname, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    fprintf(stderr, "Failed to open %s\n", fname);
    __builtin_trap();
  }
  file.seekg(0, std::ios::end);
  size_t byteLen = file.tellg();
  byteLen &= ~(CHUNK_SIZE - 1);
  if (byteLen == 0) {
    fprintf(stderr, "Input file too short (%zu) %s\n", byteLen, fname);
    __builtin_trap();
  }
  file.seekg(0, std::ios::beg);
  out->resize(byteLen);

  file.read(reinterpret_cast<char *>(out->data()), byteLen);
  if (!file.good()) {
    fprintf(stderr, "Failed to read %s\n", fname);
    __builtin_trap();
  }
  file.close();
}*/

uint8_t *mapFile(const char *fname, size_t *size) {
  int fd = open(fname, O_RDONLY);
  if (fd < 0) {
    fprintf(stderr, "Failed to open %s\n", fname);
    __builtin_trap();
  }

  struct stat stats;
  int err = fstat(fd, &stats);
  if (err < 0) {
    fprintf(stderr, "Could not read file stats %s\n", fname);
    __builtin_trap();
  }

  size_t byteLen = stats.st_size;
  byteLen &= ~(CHUNK_SIZE - 1);
  if (byteLen == 0) {
    fprintf(stderr, "Input file too short (%zu) %s\n", byteLen, fname);
    __builtin_trap();
  }

  void *mem = mmap(nullptr, byteLen, PROT_READ, MAP_SHARED, fd, 0);
  if (mem == MAP_FAILED) {
    fprintf(stderr, "Mapping failed %s\n", fname);
    __builtin_trap();
  }

  *size = byteLen;
  return (unsigned char *)mem;
}

} // namespace

int main(int argc, char **argv) {
  std::vector<unsigned char> buf;
  if (argc == 2) {
    fprintf(stderr, "Loading input file: %s\n", argv[1]);
    tx_buf = mapFile(argv[1], &tx_buf_len);
    fprintf(stderr, "Estimated sample len: %0.3fs\n",
            tx_buf_len / (44100.f * 256 * 2));
  } else {
    buf.resize(4096 * CHUNK_SIZE);
    tx_buf = buf.data();
    tx_buf_len = buf.size();
    for (size_t i = 0; i < tx_buf_len; i++) {
      uint8_t result = 0;
      size_t n = (i >> 5) & 0xFFF; // number of word in the stream
      size_t block_offset = i & 0x1F;
      size_t value_bit_offset = block_offset >> 1;
      size_t branch_offset = (block_offset & 1) * 8;
      for (size_t bit = 0; bit < 8; ++bit) {
        size_t branch = branch_offset + bit;
        uint16_t value = 0xCAF0 | branch;
        size_t value_bit =
            (value >> (15 - value_bit_offset)) & 1; // reverse bits
        result |= value_bit << bit;
      }
      tx_buf[i] = result;
    }
  }

  for (uint8_t b = 0; b <= 189; ++b) {
    if ((decode_byte(encode_byte(b))) != b) {
      fprintf(stderr, "OOOPS %d\n", b);
      __builtin_trap();
    }
  }

  constexpr unsigned int vendor = 0x0403;
  constexpr unsigned int product = 0x6014;

  struct libusb_context *usb_ctx = nullptr;

  struct libusb_device_handle *usb_dev = nullptr;
  int interface = 0;

  auto cleanup = MakeScopeGuard([&]() {
    if (usb_dev) {
      libusb_release_interface(usb_dev, interface);
      libusb_close(usb_dev);
      usb_dev = nullptr;
    }

    if (usb_ctx) {
      libusb_exit(usb_ctx);
      usb_ctx = nullptr;
    }
  });

  fprintf(stderr, "libusb_init\n");
  if (libusb_init(&usb_ctx) < 0) {
    return 0;
  }

  {
    libusb_device **devs = nullptr;
    auto cleanup_list = MakeScopeGuard([&]() {
      if (devs) {
        libusb_free_device_list(devs, /*unref_devices*/ 1);
        devs = nullptr;
      }
    });

    fprintf(stderr, "libusb_get_device_list\n");
    if (libusb_get_device_list(usb_ctx, &devs) < 0) {
      return 0;
    }

    int match_idx = -1;

    for (int dev_idx = 0; devs[dev_idx]; ++dev_idx) {
      // fprintf(stderr, "device #%d\n", dev_idx);
      struct libusb_device_descriptor desc;
      libusb_device *dev = devs[dev_idx];
      if (libusb_get_device_descriptor(dev, &desc) < 0) {
        return 0;
      }
      // Optional: check description / serial
      // TODO: check desc.bNumConfigurations == 1?
      bool match = (vendor == desc.idVendor) && (product == desc.idProduct);
      // fprintf(stderr, "%c vendor: %04hx, product: %04hx | # configs: %d\n",
      //         match ? '>' : ' ', desc.idVendor, desc.idProduct,
      //         desc.bNumConfigurations);
      if (match) {
        match_idx = dev_idx;
      }
    }

    if (match_idx == -1) {
      fprintf(stderr, "no matching devices found\n");
      return 0;
    }

    libusb_device *dev = devs[match_idx];
    fprintf(stderr, "libusb_open\n");
    if (libusb_open(dev, &usb_dev) < 0) {
      return 0;
    }

    // fprintf(stderr, "libusb_detach_kernel_driver\n");
    // if (libusb_detach_kernel_driver(usb_dev, /* interface */ 0) == 0) {
    //   fprintf(stderr, "success\n");
    // } else {
    //   fprintf(stderr, "failure\n");
    // }

    fprintf(stderr, "libusb_claim_interface\n");
    if (libusb_claim_interface(usb_dev, /* interface */ 0) != 0) {
      return 0;
    }

    uint16_t interface_index = 1; // INTERFACE_A

    fprintf(stderr, "libusb_control_transfer (reset)\n");
    if (libusb_control_transfer(
            usb_dev,
            LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE |
                LIBUSB_ENDPOINT_OUT,
            /* bRequest = SIO_RESET */ 0, /* wValue = SIO_RESET_SIO */ 0,
            /* wIndex */ interface_index, nullptr, 0, /* timeout */ 5000) < 0) {
      return 0;
    }
  }

  timeval timeout{0};
  int completed = 0;

  auto free_transfers = MakeScopeGuard([&]() {
    for (size_t i = 0; i < 3; ++i) {
      if (transfers[i]) {
        libusb_free_transfer(transfers[i]);
      }
    }
  });

  for (size_t i = 0; i < NUM_TRANSFERS; ++i) {
    transfers[i] = libusb_alloc_transfer(0);
    if (!transfers[i]) {
      fprintf(stderr, "libusb_alloc_transfer failed\n");
      return 0;
    }
    if (i == NUM_TRANSFERS - 1) {
      free_transfer = i;
      continue;
    }
    prepareTxBuffer(i, usb_dev);
  }

  start = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < NUM_TRANSFERS - 1; ++i) {
    libusb_submit_transfer(transfers[i]);
  }
  while (true) {
    int r =
        libusb_handle_events_timeout_completed(usb_ctx, &timeout, &completed);
    if (r != 0) {
      fprintf(stderr, "libusb_handle_events failed: %d\n", r);
    }
  }
}
