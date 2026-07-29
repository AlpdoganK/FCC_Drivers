/*
 * rs232_loopback.h
 * USART6 physical-layer loopback / echo bring-up test.
 *
 * This is a BENCH tool for isolating a break in the RS232 chain
 *     MCU (PA11 TX / PA12 RX) -> MAX3232 -> DB9
 * by moving the loopback point progressively further down the line.
 * See rs232_loopback.c for the wiring of each stage.
 *
 * It deliberately does NOT use the normal receive path: no DMA, no frame
 * assembler, no protocol. Polled single-byte transfers only, so a failure
 * here is a wiring/level fault and cannot be blamed on rs232.c.
 */
#ifndef RS232_LOOPBACK_H
#define RS232_LOOPBACK_H

#include "main.h"

/* Mode selected by RS232_LOOPBACK_TEST in app.c. */
typedef enum {
    RS232_LB_SELFTEST = 1, /* board sends a pattern and checks it returns  */
    RS232_LB_ECHO     = 2  /* board mirrors back whatever a PC sends       */
} RS232_LB_Mode;

/* Call once instead of the normal RS232 init. Must NOT be combined with
 * UKB_RS232_Init() — the circular DMA would consume the bytes first. */
void RS232_Loopback_Init(RS232_LB_Mode mode);

/* Call every superloop iteration. Self-paces internally. */
void RS232_Loopback_Run(void);

#endif /* RS232_LOOPBACK_H */
