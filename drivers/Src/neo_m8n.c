/*
 * u-blox NEO-M8N GPS Driver (NMEA GGA over UART, interrupt-driven RX)
 * Author: Alpdogan
 * Created: 01 July 2026
 */

#include "neo_m8n.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// UBX protocol helpers (init-time only, blocking) — used to configure the
// module's output rate and trim NMEA chatter down to just GGA.
// ---------------------------------------------------------------------------

static void UBX_Send(UART_HandleTypeDef *huart, uint8_t msg_class, uint8_t msg_id,
                      const uint8_t *payload, uint16_t len)
{
    uint8_t header[6] = {
        0xB5, 0x62, msg_class, msg_id,
        (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)
    };

    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 6; i++) { ck_a += header[i]; ck_b += ck_a; }
    for (uint16_t i = 0; i < len; i++) { ck_a += payload[i]; ck_b += ck_a; }
    uint8_t trailer[2] = { ck_a, ck_b };

    HAL_UART_Transmit(huart, header, sizeof(header), 100);
    if (len) HAL_UART_Transmit(huart, (uint8_t *)payload, len, 100);
    HAL_UART_Transmit(huart, trailer, sizeof(trailer), 100);
}

// UBX-CFG-MSG (0x06 0x01): set the output rate of an NMEA (class 0xF0) sentence
// on the port this command was received on. rate=0 disables it, rate=1 sends
// it every navigation epoch.
static void UBX_SetNmeaRate(UART_HandleTypeDef *huart, uint8_t msg_id, uint8_t rate)
{
    uint8_t payload[3] = { 0xF0, msg_id, rate };
    UBX_Send(huart, 0x06, 0x01, payload, sizeof(payload));
    HAL_Delay(20); // give the module time to apply + ACK before the next command
}

// UBX-CFG-RATE (0x06 0x08): measurement interval / nav solution ratio / time base.
static void UBX_SetRate(UART_HandleTypeDef *huart, uint16_t meas_ms, uint16_t nav_cycles, uint16_t time_ref)
{
    uint8_t payload[6] = {
        (uint8_t)(meas_ms & 0xFF),    (uint8_t)(meas_ms >> 8),
        (uint8_t)(nav_cycles & 0xFF), (uint8_t)(nav_cycles >> 8),
        (uint8_t)(time_ref & 0xFF),   (uint8_t)(time_ref >> 8),
    };
    UBX_Send(huart, 0x06, 0x08, payload, sizeof(payload));
    HAL_Delay(20);
}

// NMEA (class 0xF0) sentence IDs used below
#define NMEA_ID_GGA 0x00
#define NMEA_ID_GLL 0x01
#define NMEA_ID_GSA 0x02
#define NMEA_ID_GSV 0x03
#define NMEA_ID_RMC 0x04
#define NMEA_ID_VTG 0x05
#define NMEA_ID_GBS 0x09
#define NMEA_ID_GNS 0x0D

// ---------------------------------------------------------------------------
// NMEA GGA parsing
// ---------------------------------------------------------------------------

// ddmm.mmmm / dddmm.mmmm -> decimal degrees. Works for both 2- and 3-digit
// degree fields since minutes always occupy the last two integer digits.
static float NmeaToDecimalDegrees(float raw)
{
    float degrees = truncf(raw / 100.0f);
    float minutes = raw - degrees * 100.0f;
    return degrees + minutes / 60.0f;
}

// Parses a single completed NMEA line (already NUL-terminated, no CR/LF) in
// place. Only *GGA sentences (any talker: GP/GN/GL/...) update dev's fields.
static void ParseGGA(NEO_M8N *dev, char *line)
{
    char *star = strchr(line, '*');
    if (!star) return;

    uint8_t checksum = 0;
    for (char *p = line + 1; p < star; p++) checksum ^= (uint8_t)*p;
    uint8_t expected = (uint8_t)strtol(star + 1, NULL, 16);
    if (checksum != expected) return; // corrupted on the wire — drop it

    if (line[0] != '$' || strncmp(line + 3, "GGA", 3) != 0) return;

    // Split the comma-delimited fields in place, preserving empty fields
    // (e.g. lat/lon are blank whenever there's no fix yet).
    char *fields[15] = { 0 };
    int   nfields = 0;

    char *comma = strchr(line, ',');
    if (!comma) return;
    char *cursor = comma + 1;

    while (nfields < 15) {
        fields[nfields++] = cursor;
        comma = strchr(cursor, ',');
        if (!comma) break;
        *comma = '\0';
        cursor = comma + 1;
    }
    if (nfields < 9) return; // truncated sentence

    // fields: [0]time [1]lat [2]N/S [3]lon [4]E/W [5]fixQuality [6]numSV [7]HDOP [8]altitude
    if (fields[1][0] == '\0' || fields[3][0] == '\0') return; // no fix yet

    float lat = NmeaToDecimalDegrees(strtof(fields[1], NULL));
    float lon = NmeaToDecimalDegrees(strtof(fields[3], NULL));
    if (fields[2][0] == 'S') lat = -lat;
    if (fields[4][0] == 'W') lon = -lon;

    dev->latitude_deg  = lat;
    dev->longitude_deg = lon;
    dev->fix_quality   = (uint8_t)strtol(fields[5], NULL, 10);
    dev->satellites    = (uint8_t)strtol(fields[6], NULL, 10);
    if (fields[8][0] != '\0') dev->altitude_m = strtof(fields[8], NULL);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void NEO_M8N_Init(NEO_M8N *dev, UART_HandleTypeDef *huart)
{
    memset(dev, 0, sizeof(*dev));
    dev->uartHandle = huart;

    // Silence everything except GGA — at 5 Hz over the module's default
    // 9600 baud UART, the full default NMEA set would overrun the link.
    UBX_SetNmeaRate(huart, NMEA_ID_GGA, 1);
    UBX_SetNmeaRate(huart, NMEA_ID_GLL, 0);
    UBX_SetNmeaRate(huart, NMEA_ID_GSA, 0);
    UBX_SetNmeaRate(huart, NMEA_ID_GSV, 0);
    UBX_SetNmeaRate(huart, NMEA_ID_RMC, 0);
    UBX_SetNmeaRate(huart, NMEA_ID_VTG, 0);
    UBX_SetNmeaRate(huart, NMEA_ID_GBS, 0);
    UBX_SetNmeaRate(huart, NMEA_ID_GNS, 0);

    UBX_SetRate(huart, 200, 1, 1); // 200 ms measurement interval = 5 Hz fixes

    HAL_UART_Receive_IT(dev->uartHandle, &dev->rx_byte, 1);
}

void NEO_M8N_Process(NEO_M8N *dev)
{
    if (!dev->line_ready) return;

    char line[NEO_M8N_LINE_BUF_SIZE];
    strcpy(line, dev->line_buf); // ISR won't touch line_buf while line_ready is still true
    dev->line_ready = false;

    ParseGGA(dev, line);
}

void NEO_M8N_RxCpltCallback(NEO_M8N *dev, UART_HandleTypeDef *huart)
{
    if (dev->uartHandle != huart) return;

    uint8_t b = dev->rx_byte;

    if (b == '\n') {
        dev->isr_line[dev->isr_index] = '\0';
        if (!dev->line_ready) { // else: main loop hasn't consumed the last line — drop this one
            memcpy(dev->line_buf, dev->isr_line, dev->isr_index + 1);
            dev->line_ready = true;
        }
        dev->isr_index = 0;
    } else if (b != '\r') {
        if (dev->isr_index < NEO_M8N_LINE_BUF_SIZE - 1) {
            dev->isr_line[dev->isr_index++] = (char)b;
        } else {
            dev->isr_index = 0; // overflow — resync on the next newline
        }
    }

    HAL_UART_Receive_IT(dev->uartHandle, &dev->rx_byte, 1);
}
