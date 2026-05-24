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
    uint8_t  header;         // Header byte = 0xAB
    uint32_t timestamp;      // Milliseconds since bootup
    uint8_t  flight_state;   // Finite State Machine State ID

    // 3D Orientation Quaternions (Scaled from -1.0 to 1.0 into int16_t)
    int16_t  q_w;            
    int16_t  q_x;            
    int16_t  q_y;            
    int16_t  q_z;

    float    baro_alt;       // Barometric altitude in meters
    float    gps_lat;        // Raw Latitude from secondary parser
    float    gps_lon;        // Raw Longitude from secondary parser
    uint8_t  battery_v;      // Voltage telemetry (scaled)
    uint16_t crc;            // CRC-16 for error checking
    uint8_t  footer;         // Footer byte = 0x0A
} TelemetryPacket;

typedef struct {
    UART_HandleTypeDef *uartHandle;
    GPIO_TypeDef       *auxPort;
    uint16_t           auxPin;
    TelemetryPacket    packet;
    volatile uint8_t   tx_busy; // Volatile: modified in DMA ISR interrupt context
} LoRa_E220;

void LoRa_Init(LoRa_E220 *lora, UART_HandleTypeDef *huart, GPIO_TypeDef *auxPort, uint16_t auxPin);
uint8_t LoRa_TransmitTelemetry_NonBlocking(LoRa_E220 *lora);
void LoRa_TxCpltCallback(LoRa_E220 *lora, UART_HandleTypeDef *huart);

#endif /* TELEMETRY_H */