#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdio.h>

/* Master switch for debug console output.
 *
 * The console is USART2 at 9600 baud and _write() blocks with HAL_MAX_DELAY,
 * so output is expensive in superloop time: the ~280-character status block
 * in App_Run costs roughly 290 ms, which is three missed slots of the 10 Hz
 * RS232 test cadence. Keep this at 0 whenever the ground-test link matters.
 *
 * Set to 1 for bring-up / wiring diagnosis (I2C bus scan, sensor init errors,
 * baro calibration result), and expect the RS232 TX rate to drop while it is
 * on. Has no effect in Release builds, where DEBUG is not defined at all. */
#define DEBUG_PRINTS_ENABLED 1

#if defined(DEBUG) && DEBUG_PRINTS_ENABLED
#define DBG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define DBG_PRINT(fmt, ...) ((void)0)
#endif

#endif /* DEBUG_UART_H */
