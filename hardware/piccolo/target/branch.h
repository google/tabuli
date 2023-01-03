
// Each item is 2 bits x 16 channel;
// bundle 8 of them for word-oriented transfer.
#define BUNDLE_LEN 8
// Time slice is 1ms = 1000us
#define TICK_STEP 1000

#define PICCOLO_PLAY_MODE_RAW 0
#define PICCOLO_PLAY_MODE_SD_DITHER 1
#define PICCOLO_PLAY_MODE_SD_PWM 2

#if PICCOLO_PLAY_MODE == PICCOLO_PLAY_MODE_RAW
// bytes per second: 22550335.5705 = 420000000/(149*64)*512
// items per second: 5637583.89262 = 22550335.5705 / 4
// bundles per second: 704697.986577 = 5637583.89262 / 8
// bundles per 1000us: 704.697986577
#define BUNDLE_STEP_INT 704
#define BUNDLE_STEP_REM 697986577
#define BUNDLE_STEP_DIV 1000000000
#elif PICCOLO_PLAY_MODE == PICCOLO_PLAY_MODE_SD_DITHER
// bytes per second: 22579200 = 44100*256*2
// items per second: 5644800 = 22579200 / 4
// bundles per second: 705600 = 5644800 / 8
// bundles per 1000us: 705.6
#define BUNDLE_STEP_INT 705
#define BUNDLE_STEP_REM 6
#define BUNDLE_STEP_DIV 10
#elif PICCOLO_PLAY_MODE == PICCOLO_PLAY_MODE_SD_PWM
// bytes per second: 22550335.5705 = 420000000/(149*64)*512
// items per second: 5637583.89262 = 22550335.5705 / 4
// bundles per second: 704697.986577 = 5637583.89262 / 8
// bundles per 1000us: 704.697986577
#define BUNDLE_STEP_INT 704
#define BUNDLE_STEP_REM 697986577
#define BUNDLE_STEP_DIV 1000000000
#else
#error Unknown play mode.
#endif
