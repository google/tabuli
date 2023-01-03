#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <libusb.h>
#include <vector>

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

} // namespace

int main(int argc, char **argv) {
  constexpr unsigned int vendor = 0x0403;
  constexpr unsigned int product = 0x6014;

  struct libusb_context *usb_ctx = nullptr;

  struct libusb_device_handle *usb_dev = nullptr;
  int interface = 0;

  const auto encode_byte = [](uint8_t b) {
    // simple, but works well only for for 0..189
    if (b > 189) {
      return 0;
    }
    return b + ((b + 1) >> 7);
  };

  const auto decode_byte = [](uint8_t b) { return b - (b >> 7); };
  // 4x version: u32 - ((u32 & 0x80808080) >> 7)

  for (uint8_t b = 0; b <= 189; ++b) {
    if ((decode_byte(encode_byte(b))) != b) {
      fprintf(stderr, "OOOPS %d\n", b);
      __builtin_trap();
    }
  }

  int tx_chunk_size = 2048 * 512; // 1MB
  std::vector<unsigned char> tx_buf(tx_chunk_size);
  for (size_t i = 0; i < tx_buf.size(); i++) {
    uint8_t result = 0;
    size_t n = (i >> 5) & 0xFFF; // number of word in the stream
    size_t block_offset = i & 0x1F;
    size_t value_bit_offset = block_offset >> 1;
    size_t branch_offset = (block_offset & 1) * 8;
    for (size_t bit = 0; bit < 8; ++bit) {
      size_t branch = branch_offset + bit;
      size_t value = branch | (n << 4);
      size_t value_bit = value >> value_bit_offset;
      result |= value_bit << bit;
    }
    tx_buf[i] = encode_byte(result);
  }

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
            /* wIndex */ interface_index, nullptr, 0,
            /* timeout */ 15000) < 0) {
      return 0;
    }
  }

  fprintf(stderr, "sending...\n");
  size_t num_chunks = 64;
  auto start = std::chrono::high_resolution_clock::now();
  for (size_t chunk = 0; chunk < num_chunks; ++chunk) {
    int actual_length;
    if (libusb_bulk_transfer(usb_dev, /* endpoint */ 0x02, tx_buf.data(),
                             tx_chunk_size, &actual_length,
                             /* timeout */ 5000) < 0) {
      fprintf(stderr, "write failed\n");
      return 0;
    }
    if (actual_length < tx_chunk_size) {
      fprintf(stderr, "%d < %d\n", actual_length, tx_chunk_size);
    }
  }
  auto end = std::chrono::high_resolution_clock::now();
  auto delta =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();
  fprintf(stderr, "time: %lldms\n", delta);
}
