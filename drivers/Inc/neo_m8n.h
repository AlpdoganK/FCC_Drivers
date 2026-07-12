/*
 * u-blox NEO-M8N GPS Driver (NMEA GGA over UART, interrupt-driven RX)
 * Author: Alpdogan
 * Created: 01 July 2026
 */

#ifndef NEO_M8N_H_
#define NEO_M8N_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define NEO_M8N_LINE_BUF_SIZE 96

typedef struct {

    UART_HandleTypeDef *uartHandle;

    // ---- ISR-owned line assembly state — do not touch outside the driver ----
    char    isr_line[NEO_M8N_LINE_BUF_SIZE];
    uint8_t isr_index;
    uint8_t rx_byte; // one-byte scratch handed to HAL_UART_Receive_IT

    char             line_buf[NEO_M8N_LINE_BUF_SIZE]; // completed line, swapped out of the ISR
    volatile bool    line_ready;                       // set by ISR, cleared by NEO_M8N_Process

    // ---- Parsed fix data, updated by NEO_M8N_Process() ----
    float   latitude_deg;  // +N / -S, 0 if never fixed
    float   longitude_deg; // +E / -W, 0 if never fixed
    float   altitude_m;    // MSL altitude from GGA
    uint8_t fix_quality;   // 0 = no fix, 1 = GPS fix, 2 = DGPS fix
    uint8_t satellites;    // satellites used in fix

} NEO_M8N;

// Configures the module for a 5 Hz fix rate, GGA-only NMEA output, then
// starts interrupt-driven reception. Blocking (init-time UBX config writes).
void NEO_M8N_Init(NEO_M8N *dev, UART_HandleTypeDef *huart);

// Call every App_Run loop iteration: parses any line the ISR has buffered.
void NEO_M8N_Process(NEO_M8N *dev);

// Call from HAL_UART_RxCpltCallback for every UART instance in use.
void NEO_M8N_RxCpltCallback(NEO_M8N *dev, UART_HandleTypeDef *huart);

#endif /* NEO_M8N_H_ */
