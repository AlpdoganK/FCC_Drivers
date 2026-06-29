/*
 * Telemetry & EBYTE E220-900T30S LoRa Driver
 * Author: Alpdogan
 * Created: 24 May 2026
 */

#include "e220.h"
#include <string.h> // For memset

static uint16_t CalculateCRC16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF; // Initial value
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8; 
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021; // Polynomial used in CRC-CCITT
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void LoRa_Init(LoRa_E220 *lora, UART_HandleTypeDef *huart, GPIO_TypeDef *auxPort, uint16_t auxPin) {
    lora->uartHandle = huart;
    lora->auxPort = auxPort;
    lora->auxPin = auxPin;
    lora->tx_busy = 0;
    
    memset(&lora->packet, 0, sizeof(TelemetryPacket));

    lora->packet.header = 0xAB; // Set header byte
    lora->packet.footer = 0x0A; // Set footer byte
    lora->packet.Lora_ADDRH = 5; // Set address header high byte
    lora->packet.Lora_ADDRL = 122; // Set address header low byte
    lora->packet.Lora_CH = 31; // Set channel
}

uint8_t LoRa_TransmitTelemetry_NonBlocking(LoRa_E220 *lora) {
    // 1. Hardware Check: Verify physical module buffer via AUX pin
    if (HAL_GPIO_ReadPin(lora->auxPort, lora->auxPin) == GPIO_PIN_RESET) {
        return 1; // Radio RF buffer full / transmitting previous frame
    }

    // 2. Software Lockout: Check if STM32 DMA engine is currently busy shifting out data
    if (lora->tx_busy == 1) {
        return 2; // STM32 DMA TX pipeline busy
    }

    // 3. CRC Calculation: Fixed function naming mismatch
    // Computes over everything between the header and the CRC field itself
    uint8_t *crc_start_ptr = ((uint8_t*)&lora->packet) + 1; 
    uint16_t crc_payload_len = sizeof(TelemetryPacket) - 4; // Total size minus header(1), crc(2), footer(1)
    
    lora->packet.crc = CalculateCRC16(crc_start_ptr, crc_payload_len);

    // Set software lockout flag before triggering DMA transfer
    lora->tx_busy = 1;

    // Kick off non-blocking background memory transfer
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(lora->uartHandle, (uint8_t*)&lora->packet, sizeof(TelemetryPacket));

    if (status != HAL_OK) {
        lora->tx_busy = 0; // Release lock instantly on failure
        return 3;
    }

    return 0; // Success: Data handed over to DMA
}

uint8_t LoRa_TransmitTelemetry_Blocking(LoRa_E220 *lora, uint32_t timeout) {
    // 1. Hardware Check: AUX pin must be HIGH before transmitting
    if (HAL_GPIO_ReadPin(lora->auxPort, lora->auxPin) == GPIO_PIN_RESET) {
        return 1; // Radio buffer busy
    }

    // 2. CRC Calculation (same region as DMA variant)
    uint8_t *crc_start_ptr = ((uint8_t*)&lora->packet) + 4;
    uint16_t crc_payload_len = sizeof(TelemetryPacket) - 7;
    lora->packet.crc = CalculateCRC16(crc_start_ptr, crc_payload_len);

    // 3. Blocking transmit — HAL internally polls until complete or timeout
    HAL_StatusTypeDef status = HAL_UART_Transmit(
        lora->uartHandle,
        (uint8_t*)&lora->packet,
        sizeof(TelemetryPacket),
        timeout
    );

    if (status == HAL_TIMEOUT) return 3;
    if (status != HAL_OK)      return 4;

    // 4. Confirm the module actually accepted the packet: AUX should pulse LOW
    // within ~10 ms as the module loads its TX buffer.
    uint32_t t = HAL_GetTick();
    while (HAL_GPIO_ReadPin(lora->auxPort, lora->auxPin) == GPIO_PIN_SET) {
        if (HAL_GetTick() - t > 50) return 5; // AUX never went LOW — module ignored data
    }
    // Wait for AUX to return HIGH (RF transmission finished)
    t = HAL_GetTick();
    while (HAL_GPIO_ReadPin(lora->auxPort, lora->auxPin) == GPIO_PIN_RESET) {
        if (HAL_GetTick() - t > 3000) return 6; // AUX stuck LOW — module hung
    }

    return 0;
}

// 4. ISR Unlock Mechanism: Call this inside your main.c HAL_UART_TxCpltCallback
void LoRa_TxCpltCallback(LoRa_E220 *lora, UART_HandleTypeDef *huart) {
    if (lora->uartHandle == huart) {
        lora->tx_busy = 0; // Clear software lock once DMA transmission finishes
    }
}