#include "play.h"

#include "hardware/pio.h"

//#include "hardware/address_mapped.h"
//#include "hardware/structs/systick.h"
//#include "pico/assert.h"

// Sampling frequency: 44100
// Number of channels: 16
// FIFO length: 8
// FIFO bits: 8 * 32 = 256
// Number of channels per PIO: 2
// Bits per channel ber PIO: 256 / 2 = 128
// Clk per PIO bit: 8
// Clk per loop: 128 * 8 = 1024
// Avg clk per channel per loop: 1024 / 16 = 8 * 8 = 64
// Oversampling: 420M / 44100 / 1024 ~= 9.3
// Sample step per loop: 44100 * 1024 / 420M ~= 0.107
// -//- * 65536: 7046 + 1346/3125

uint32_t kStepInt = 7046;
uint32_t kStepRem = 1346;
uint32_t kStepDiv = 3125;

volatile uint32_t read_pos = 0;
volatile uint32_t write_pos = 0;
volatile uint32_t rw_flag = 0;

extern const uint32_t *sd_patterns;

#define MAX_TICK 0xFFFFFF
#define DEBUG_CLK 0
#define CLK_PER_LOOP (1024 - 0)

void __attribute__((noinline))
merge(uint32_t *to, uint32_t *from1, uint32_t *from2) {
  for (uint32_t j = 0; j < 8; ++j) {
    to[j * 8] = from1[j] | from2[j];
  }
}

void __attribute__((noinline)) dummy(Cookie *cookie) {
  asm volatile("" : : : "memory");
}

void play(Cookie cookie, uint16_t *src) {
  // io_wo_32 *set = &sio_hw->gpio_set;
  // io_wo_32 *clr = &sio_hw->gpio_clr;
  // io_wo_32 *togl = &sio_hw->gpio_togl;
  //  uint32_t dbg[16] = {0};
#if DEBUG_CLK
  systick_hw->rvr = MAX_TICK;
  systick_hw->csr = 5; // enable + no-int + cpu-clk
#endif

  // pio_set_sm_mask_enabled(pio0, 0xF, true);
  // pio_set_sm_mask_enabled(pio1, 0xF, true);
  // --
  // pio0->ctrl = 0xF;
  // pio1->ctrl = 0xF;
  // --
  io_wo_32 *pio0_ctrl = &pio0->ctrl;
  io_wo_32 *pio1_ctrl = &pio1->ctrl;
  uint32_t all_sms = 0xF;
  // Force PIO SM start with predictable delay (2 ticks)
  asm volatile(".thumb\n\t"
               ".syntax unified\n\t"
               "str %2, [%0, 0]\n\t"
               "str %2, [%1, 0]\n\t"
               : // no output
               : "l"(pio0_ctrl), "l"(pio1_ctrl), "l"(all_sms)
               : "memory");

  // After launching SMs immediately consume 2 items; replenish.
  pio0->txf[0] = 0;
  pio1->txf[0] = 0;
  pio0->txf[1] = 0;
  pio1->txf[1] = 0;
  pio0->txf[2] = 0;
  pio1->txf[2] = 0;
  pio0->txf[3] = 0;
  pio1->txf[3] = 0;
  pio0->txf[0] = 0;
  pio1->txf[0] = 0;
  pio0->txf[1] = 0;
  pio1->txf[1] = 0;
  pio0->txf[2] = 0;
  pio1->txf[2] = 0;
  pio0->txf[3] = 0;
  pio1->txf[3] = 0;

  while (rw_flag == 1) {
    uint32_t pos = cookie.pos + kStepInt;
    uint32_t tail = cookie.tail + kStepRem;
    if (tail >= kStepDiv) {
      tail -= kStepDiv;
      pos++;
    }
    uint32_t increment = (pos >> 16) * 16;
    pos &= 0xFFFF;
    cookie.pos = pos;
    cookie.tail = tail;

    uint32_t next_mul = pos;
    uint32_t mul = 0x10000 - next_mul;

    pos = read_pos + increment;
    read_pos = pos;
    uint16_t *sample = src + (pos & BUF_MASK);

#pragma GCC unroll 16
    for (uint32_t i = 0; i < 16; ++i) {
      uint32_t tmp1 = sample[i]; // fixed offset after unroll
      tmp1 *= mul;
      uint32_t tmp2 = sample[i + 16]; // fixed offset after unroll
      tmp2 *= next_mul;
      tmp1 += tmp2;
      tmp1 >>= 16;
      tmp2 = cookie.qs[i]; // on stack

      // tmp1 += tmp2;
      //  Alas, GCC is too buggy to do that!
      asm(".thumb\n\t"
          ".syntax unified\n\t"
          "adds %0, %1, %2"
          : "=l"(tmp1)
          : "0"(tmp1), "l"(tmp2));

      tmp2 = tmp1 << (32 - 9);
      tmp2 >>= (32 - 9);
      cookie.qs[i] = tmp2; // on stack

      tmp1 >>= 9;
      tmp1 <<= 2;
      cookie.bank[i] = tmp1; // on stack
    }

    io_wo_32 *pio0txf = pio0->txf;
    io_wo_32 *pio1txf = pio1->txf;
#pragma GCC unroll 4
    for (uint32_t j = 0; j < 4; ++j) {
      while (pio_sm_is_tx_fifo_full(pio0, 0)) {
        // wait for FIFO; all SM should become ready since they are synced
      }
      const uint32_t *base = sd_patterns + j;
      pio0txf[0] = base[cookie.bank[0]];
      pio0txf[0] = base[cookie.bank[1]];
      pio0txf[1] = base[cookie.bank[2]];
      pio0txf[1] = base[cookie.bank[3]];
      pio0txf[2] = base[cookie.bank[4]];
      pio0txf[2] = base[cookie.bank[5]];
      pio0txf[3] = base[cookie.bank[6]];
      pio0txf[3] = base[cookie.bank[7]];
      pio1txf[0] = base[cookie.bank[8]];
      pio1txf[0] = base[cookie.bank[9]];
      pio1txf[1] = base[cookie.bank[10]];
      pio1txf[1] = base[cookie.bank[11]];
      pio1txf[2] = base[cookie.bank[12]];
      pio1txf[2] = base[cookie.bank[13]];
      pio1txf[3] = base[cookie.bank[14]];
      pio1txf[3] = base[cookie.bank[15]];
      // dbg[j] = pio0->fdebug | pio1->fdebug;
    }

#if DEBUG_CLK
    uint32_t tick = systick_hw->cvr;
    uint32_t delta = MAX_TICK - tick;
    uint32_t in_time = ((delta - CLK_PER_LOOP) >> 31) & 1;
    *(in_time ? set : clr) = 1 << 25;
    systick_hw->cvr = 0;
#endif
    uint32_t dbg = pio0->fdebug | pio1->fdebug;
    //*((dbg & 0x00FF0000) ? set : clr) = 1 << 25;
    pio0->fdebug = 0xFFFF0000;
    pio1->fdebug = 0xFFFF0000;
  }
}
