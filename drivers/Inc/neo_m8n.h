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

// ---------------------------------------------------------------------------
// Bring-up survey (diagnostic only — not used in flight)
//
// GSV sentences report every satellite the receiver can *see*, with a C/N0
// signal strength, whether or not it has a fix. That is the one measurement
// that separates the three reasons a receiver sits at zero satellites:
//
//   sentences climb, sats_in_view == 0   hearing nothing at all — antenna
//                                        disconnected, facing the wrong way,
//                                        or a dead/counterfeit module
//   sats_in_view > 0, max_snr < ~25      hearing them but too weak to lock:
//                                        obstruction, poor antenna, or an
//                                        interferer desensing the front end
//   max_snr > ~35 and still no fix       signal is fine, the problem is
//                                        upstream in the fix logic
//
// A healthy patch antenna under open sky gives 4+ satellites at 35-45 dB-Hz.
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t  sats_in_view;  // high-water mark of the GSV "satellites in view" field
    uint8_t  sats_with_snr; // most satellites reporting a non-blank C/N0 in one cycle
    uint8_t  max_snr;       // best C/N0 seen, dB-Hz
    uint8_t  last_snr;      // best C/N0 in the most recent cycle — falls back when
                            // conditions worsen, unlike max_snr
    uint32_t sentences;     // GSV sentences fed in; proves the feed is alive
} NEO_M8N_Survey;

// Configures the module for a 5 Hz fix rate, GGA-only NMEA output, then
// starts interrupt-driven reception. Blocking (init-time UBX config writes).
void NEO_M8N_Init(NEO_M8N *dev, UART_HandleTypeDef *huart);

// Call every App_Run loop iteration: parses any line the ISR has buffered.
void NEO_M8N_Process(NEO_M8N *dev);

// Call from HAL_UART_RxCpltCallback for every UART instance in use.
void NEO_M8N_RxCpltCallback(NEO_M8N *dev, UART_HandleTypeDef *huart);

// Re-enables the GSV sentences NEO_M8N_Init silenced. Blocking; call once,
// after NEO_M8N_Init. Roughly quadruples the byte rate on the link, which is
// still comfortable for GGA+GSV at 9600 baud but is why it is opt-in.
void NEO_M8N_EnableSurvey(NEO_M8N *dev);

// Feed every completed NMEA line to accumulate survey statistics. Returns true
// if the line was a GSV sentence (i.e. was consumed here). Does not modify the
// line, so it is safe to call before NEO_M8N_Process.
bool NEO_M8N_SurveyFeed(NEO_M8N_Survey *survey, const char *line);

#endif /* NEO_M8N_H_ */
