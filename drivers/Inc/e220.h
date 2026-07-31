/*
 * Telemetry & EBYTE E220-900T30S LoRa Driver
 * Author: Alpdogan
 * Created: 23 May 2026
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "stm32f4xx_hal.h"

// Telemetry Packet Structure (Packed to avoid padding bytes)
typedef struct __attribute__((__packed__)) {
    uint8_t  Lora_ADDRH;         // Header byte = 0x7B
    uint8_t  Lora_ADDRL;         // Header byte = 0xD3
    uint8_t  Lora_CH;         // Header byte = 0x2B
    uint8_t  header;         // Header byte = 0xAB
    uint32_t timestamp;      // Milliseconds since bootup
    uint8_t  flight_state;   // Finite State Machine State ID

    float    ax;             // Acceleration X (m/s^2)
    float    ay;             // Acceleration Y (m/s^2)
    float    az;             // Acceleration Z (m/s^2)

    // Body rates straight off the MPU6050, in the same axis order as the accel
    // triplet above. Note gy is the PITCH rate (gyro[1]) — it was labelled
    // "Yaw Rate" here while app.c has always fed it gyro[1], and it is the one
    // FlightSM_Update consumes, so the label was the thing that was wrong.
    float    gx;             // Gyro Roll Rate  (degrees/s)
    float    gy;             // Gyro Pitch Rate (degrees/s)
    float    gz;             // Gyro Yaw Rate   (degrees/s)

    // Complementary-filtered attitude. pitch is measured FROM VERTICAL:
    // 0 = nose up, 90 = horizontal, 180 = nose down (see app.c).
    float    pitch;          // Pitch Angle (degrees)
    float    roll;           // Roll Angle  (degrees)

    float    baro_alt;       // Barometric altitude in meters (fused)
    float    gps_lat;        // Raw Latitude from secondary parser
    float    gps_lon;        // Raw Longitude from secondary parser
    uint16_t crc;            // CRC-16 for error checking
    uint8_t  footer;         // Footer byte = 0x0A
} TelemetryPacket;

typedef struct {
    UART_HandleTypeDef *uartHandle;
    GPIO_TypeDef       *auxPort;
    uint16_t           auxPin;
    TelemetryPacket    packet;
    volatile uint8_t   tx_busy; // Volatile: modified in DMA ISR interrupt context

    // Diagnostics from the last blocking transmit. aux_low_ms is how long the
    // module held AUX low after being handed the packet: a few ms means it only
    // buffered the bytes (nothing radiated), while tens-to-hundreds of ms is the
    // air time of a real transmission (48 B at 2.4 kbps air rate is ~160 ms).
    uint32_t           aux_low_ms;
} LoRa_E220;

void LoRa_Init(LoRa_E220 *lora, UART_HandleTypeDef *huart, GPIO_TypeDef *auxPort, uint16_t auxPin);
uint8_t LoRa_TransmitTelemetry_NonBlocking(LoRa_E220 *lora);
uint8_t LoRa_TransmitTelemetry_Blocking(LoRa_E220 *lora, uint32_t timeout_ms);
void LoRa_TxCpltCallback(LoRa_E220 *lora, UART_HandleTypeDef *huart);

#endif /* TELEMETRY_H */