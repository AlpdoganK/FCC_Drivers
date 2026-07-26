#include "debug_uart.h"
#include "usart.h"

/* Overrides the weak _write() in syscalls.c. Routes printf to USART2,
 * repurposed as the debug console (GPS is not in use at the same time). */
int _write(int file, char *ptr, int len) {
    (void)file;
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}
