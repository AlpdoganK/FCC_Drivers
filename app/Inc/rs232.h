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

/* Sensor-data packet length (Tablo 3 outbound / Tablo 4 inbound - same layout):
 *   [0]      Header 0xAB
 *   [1..32]  8 x FLOAT32: altitude, pressure, accX, accY, accZ, angX, angY, angZ
 *   [33]     Checksum
 *   [34]     Footer1 0x0D
 *   [35]     Footer2 0x0A
 * = 36 bytes. (Tablo 3/4 mislabel the tail bytes as 32/33/34 - the byte
 * numbering repeats after "AÇI Z 4"; the field list itself is unambiguous.) */
#define UKB_SENSOR_PACKET_LEN 36u
#define UKB_SUT_DATA_LEN      UKB_SENSOR_PACKET_LEN

/* One sample of the eight FLOAT32 fields, in Tablo 3 order.
 * Angles follow the Bolum 1.2 body axes: X = longitudinal (roll),
 * Y = lateral (pitch), Z = vertical (yaw). */
typedef struct {
    float altitude_m;    /* irtifa   - m, above sea level */
    float pressure_mbar; /* basinc   - mBar (== hPa)      */
    float acc_x;         /* ivme X   - m/s^2              */
    float acc_y;         /* ivme Y   - m/s^2              */
    float acc_z;         /* ivme Z   - m/s^2              */
    float ang_x;         /* aci X    - deg                */
    float ang_y;         /* aci Y    - deg                */
    float ang_z;         /* aci Z    - deg                */
} UKB_SensorSample;

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

/* Build and blocking-transmit one Tablo 3 sensor packet on USART6.
 * ~3.1 ms on the wire at 115200 8N1. Returns the HAL_UART_Transmit status. */
HAL_StatusTypeDef UKB_RS232_SendSensorPacket(const UKB_SensorSample *s);

/* Status packet (Tablo 6): [0xAA][Data1][Data2][Checksum][0x0D][0x0A].
 * Data1 carries status bits 0-7, Data2 bits 8-15 (Tablo 5 / EK-15 Tablo 3):
 *   0 KTE liftoff        4 ATE descent detected
 *   1 YSD burn time      5 SPE drogue command
 *   2 IEA min altitude   6 BIT altitude threshold
 *   3 GAA body angle     7 APE main command
 * Bits 8-15 are reserved. ~0.5 ms on the wire. */
#define UKB_STATUS_PACKET_LEN 6u
HAL_StatusTypeDef UKB_RS232_SendStatusPacket(uint16_t status_bits);

/* Blocking-transmit a NUL-terminated string on USART6 verbatim.
 * For human-readable bench output only — the test software expects the
 * binary frames above, so nothing sent through here is part of the
 * SIT/SUT protocol. */
HAL_StatusTypeDef UKB_RS232_SendText(const char *s);

#endif /* RS232_H_ */
