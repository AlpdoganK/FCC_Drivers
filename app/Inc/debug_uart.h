#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdio.h>

#ifdef DEBUG
#define DBG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define DBG_PRINT(fmt, ...) ((void)0)
#endif

#endif /* DEBUG_UART_H */
