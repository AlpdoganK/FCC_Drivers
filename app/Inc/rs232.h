/*
 * rs232.h
 * TEKNOFEST UKB - RS232 (USART6) command handling for ground test device
 */
#ifndef RS232_H_
#define RS232_H_

#include "stm32f4xx_hal.h"

/* UKB ground-test mode, mirrors the last validated RS232 command.
 * Test_Stop doubles as the idle / normal-flight-computer state. */
typedef enum {
    Test_SIT = 0,   /* Sistem Ici Test running    */
    Test_SUT,       /* Sistem Ustu Test running   */
    Test_Stop       /* stopped / normal operation */
} Test_Status;

/* Synthetic sensor-data packet (Tablo 4) payload length. */
#define UKB_SUT_DATA_LEN 34u

/* ---- Event flags ----
 * Set by the USART6 RX-DMA path (HAL_UARTEx_RxEventCallback, ISR context).
 * App_Run() polls and clears these each loop iteration instead of the
 * driver calling back into app-level code directly - keeps anything
 * slow (arming timers, telemetry cadence changes) out of the ISR. */
extern volatile uint8_t flag_sit_pending;    // new SIT command just validated
extern volatile uint8_t flag_sut_pending;    // new SUT command just validated
extern volatile uint8_t flag_stop_pending;   // new STOP command just validated
extern volatile uint8_t flag_sut_data_ready; // new 0xAB synthetic data packet buffered

/* Valid only while flag_sut_data_ready is set; App_Run should copy out
 * whatever it needs then clear flag_sut_data_ready before the next
 * packet can arrive and overwrite this buffer. */
extern volatile uint8_t  ukb_sut_data[UKB_SUT_DATA_LEN];
extern volatile uint16_t ukb_sut_data_len;

void UKB_RS232_Init(void);
Test_Status UKB_RS232_GetMode(void);
uint8_t UKB_Checksum(const uint8_t *buf, uint16_t n);

#endif /* RS232_H_ */
