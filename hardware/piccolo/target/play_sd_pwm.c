#include "play.h"

#include "hardware/pio.h"

volatile uint32_t read_pos = 0;
volatile uint32_t write_pos = 0;
volatile uint32_t rw_flag = 0;

void play(Cookie cookie, uint16_t *src) {
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

  // After launching SMs immediately consumes; replenish.
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
    uint32_t fine_pos = cookie.pos + kStepInt;
    uint32_t tail = cookie.tail + kStepRem;
    if (tail >= kStepDiv) {
      tail -= kStepDiv;
      fine_pos++;
    }
    uint32_t increment = fine_pos >> 16;
    cookie.pos = fine_pos & 0xFFFF;
    cookie.tail = tail;

    uint32_t next_mul = fine_pos;
    uint32_t mul = 0x10000 - next_mul;

    uint32_t pos = read_pos + increment * 16;
    read_pos = pos;
    uint16_t *sample_addr = src + (pos & BUF_MASK);
    uint16_t *sample = sample_addr;
#pragma GCC unroll 16
    for (uint32_t i = 0; i < 16; ++i) {
      if ((i & 7) >= 6)
        continue;
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

      tmp2 = tmp1 << (15 + PWM_BITS);
      tmp2 >>= (15 + PWM_BITS);
      cookie.qs[i] = tmp2; // on stack

      tmp1 >>= 17 - PWM_BITS;
      tmp1 <<= 2 + PWM_BAND_BITS - PWM_BITS;
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
      // pio0txf[3] = base[cookie.bank[6]];
      // pio0txf[3] = base[cookie.bank[7]];
      pio1txf[0] = base[cookie.bank[8]];
      pio1txf[0] = base[cookie.bank[9]];
      pio1txf[1] = base[cookie.bank[10]];
      pio1txf[1] = base[cookie.bank[11]];
      pio1txf[2] = base[cookie.bank[12]];
      pio1txf[2] = base[cookie.bank[13]];
      // pio1txf[3] = base[cookie.bank[14]];
      // pio1txf[3] = base[cookie.bank[15]];
    }
  }
}
