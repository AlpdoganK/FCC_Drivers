#include "debug_uart.h"

/* Overrides the weak _write() in syscalls.c. Debug print is disabled for now:
 * USART2 is dedicated to the NEO-M8N GPS link, so there's no free UART to
 * route printf to until USART6's RS232 test link is implemented. DBG_PRINT
 * call sites are left in place; this just drops the output. */
int _write(int file, char *ptr, int len) {
    (void)file;
    (void)ptr;
    return len;
}
