#include <stdint.h>

typedef struct Cookie {
  uint16_t bank[16];
  uint16_t qs[16];
  uint32_t pos;
  uint32_t tail;
} Cookie;

#define BUF_LEN 16384
#define BUF_MASK (BUF_LEN - 1)
#define LAG 8192

extern volatile uint32_t read_pos;
extern volatile uint32_t write_pos;

// TODO: enum
// 0 - not started
// 1 - play
// 2 - request stop
// 3 - confirmed stop
extern volatile uint32_t rw_flag;

void play(Cookie cookie, uint16_t *src);
