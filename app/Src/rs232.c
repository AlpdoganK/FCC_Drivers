/*
 * rs232.c
 * TEKNOFEST UKB - RS232 (USART6) command handling for ground test device
 *
 * Target: STM32F411 Blackpill, USART6 @ 115200 8N1 full-duplex
 * RX: HAL_UARTEx_ReceiveToIdle_DMA (circular) -> IDLE-line framing
 * Command frames (Tablo 1): 5 bytes, big-endian order:
 *     [Header 0xAA][Command][Checksum][Footer1 0x0D][Footer2 0x0A]
 *
 * Command set (Tablo 1):
 *     SIT Start : AA 20 8C 0D 0A
 *     SUT Start : AA 22 8E 0D 0A
 *     Stop      : AA 24 90 0D 0A
 *
 * NOTE on checksums: the Tablo 1 command checksums do NOT match the
 * "sum of bytes mod 256" rule in section 3. We therefore validate
 * INCOMING commands by exact match against the three known frames.
 * For OUTGOING telemetry/status we DO use sum-mod-256 (section 3),
 * which is what the test device verifies.
 *
 * This module only validates frames and raises flags (see rs232.h) -
 * it never calls back into app-level logic. App_Run() polls those
 * flags and clears them once handled.
 */
#include "rs232.h"
#include <string.h>

/* ---- Frame constants ---- */
#define UKB_HEADER      0xAAu   /* command / status header          */
#define UKB_HEADER_DATA 0xABu   /* synthetic sensor data header (SUT)*/
#define UKB_FOOTER1     0x0Du
#define UKB_FOOTER2     0x0Au

#define CMD_SIT_START   0x20u
#define CMD_SUT_START   0x22u
#define CMD_STOP        0x24u

#define CMD_FRAME_LEN   5u      /* Header+Command+Checksum+F1+F2     */

/* RX ring buffer for IDLE-DMA. Sized > longest inbound frame.
 * Inbound during SUT = 34-byte synthetic data packet (Tablo 4),
 * plus 5-byte commands. 128 gives comfortable headroom. */
#define RX_BUF_LEN      128u

extern UART_HandleTypeDef huart6;

static uint8_t rx_buf[RX_BUF_LEN];
static Test_Status ukb_mode = Test_Stop;

/* ---- Public event flags (see rs232.h) ---- */
volatile uint8_t flag_sit_pending    = 0;
volatile uint8_t flag_sut_pending    = 0;
volatile uint8_t flag_stop_pending   = 0;
volatile uint8_t flag_sut_data_ready = 0;

volatile uint8_t  ukb_sut_data[UKB_SUT_DATA_LEN];
volatile uint16_t ukb_sut_data_len = 0;

/* ---- Known-good command frames (for exact-match validation) ---- */
static const uint8_t FRAME_SIT[CMD_FRAME_LEN] = {UKB_HEADER, CMD_SIT_START, 0x8C, UKB_FOOTER1, UKB_FOOTER2};
static const uint8_t FRAME_SUT[CMD_FRAME_LEN] = {UKB_HEADER, CMD_SUT_START, 0x8E, UKB_FOOTER1, UKB_FOOTER2};
static const uint8_t FRAME_STP[CMD_FRAME_LEN] = {UKB_HEADER, CMD_STOP,      0x90, UKB_FOOTER1, UKB_FOOTER2};

/* ================================================================ */
/*  Init: kick off IDLE-DMA reception                               */
/* ================================================================ */
void UKB_RS232_Init(void)
{
    ukb_mode = Test_Stop;
    flag_sit_pending    = 0;
    flag_sut_pending    = 0;
    flag_stop_pending   = 0;
    flag_sut_data_ready = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buf, RX_BUF_LEN);
    /* Disable the DMA half-transfer interrupt: we only care about
     * IDLE + transfer-complete, HT just adds noise for framing. */
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
}

Test_Status UKB_RS232_GetMode(void)
{
    return ukb_mode;
}

/* ================================================================ */
/*  Command frame parser                                            */
/*  Called from the RX-event callback with a complete IDLE burst.   */
/*  A burst may contain: one 5-byte command, OR a 34-byte SUT data  */
/*  packet (0xAB header, handled elsewhere), OR noise.              */
/*  Sets event flags only - does not call into app-level code.      */
/* ================================================================ */
static void UKB_RS232_ProcessCommand(const uint8_t *frame, uint16_t len)
{
    /* Only command frames are handled here. They are exactly 5 bytes
     * and start with 0xAA. Synthetic-data packets (0xAB, 34 bytes)
     * are routed to the SUT data handler instead. */
    if (len < CMD_FRAME_LEN) {
        return;
    }

    /* Scan for a valid command header inside the burst. In practice
     * IDLE framing delivers one frame per burst, but scanning makes
     * us robust to a leading byte of noise. */
    for (uint16_t i = 0; i + CMD_FRAME_LEN <= len; i++) {
        if (frame[i] != UKB_HEADER) {
            continue;
        }
        const uint8_t *f = &frame[i];

        /* Footers must line up, else it's not a command frame */
        if (f[3] != UKB_FOOTER1 || f[4] != UKB_FOOTER2) {
            continue;
        }

        if (memcmp(f, FRAME_SIT, CMD_FRAME_LEN) == 0) {
            ukb_mode = Test_SIT;
            flag_sut_pending  = 0;
            flag_stop_pending = 0;
            flag_sit_pending  = 1; // App_Run: arm 1 s timer + start 10 Hz TX
            return;
        }
        else if (memcmp(f, FRAME_SUT, CMD_FRAME_LEN) == 0) {
            ukb_mode = Test_SUT;
            flag_sit_pending  = 0;
            flag_stop_pending = 0;
            flag_sut_pending  = 1; // App_Run: arm 1 s timer + start 10 Hz TX
            return;
        }
        else if (memcmp(f, FRAME_STP, CMD_FRAME_LEN) == 0) {
            ukb_mode = Test_Stop;
            flag_sit_pending  = 0;
            flag_sut_pending  = 0;
            flag_stop_pending = 1; // App_Run: clear all test state -> NORMAL mode
            return;
        }
        /* header+footers matched but command/checksum didn't:
         * ignore and keep scanning */
    }
}

/* ================================================================ */
/*  HAL RX-event callback: fires on IDLE line or buffer full.       */
/*  With ReceiveToIdle_DMA, 'Size' = bytes received this burst.     */
/*  We re-arm reception every time.                                 */
/* ================================================================ */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART6) {
        return;
    }

    if (Size > 0) {
        if (rx_buf[0] == UKB_HEADER) {
            /* command / status frame */
            UKB_RS232_ProcessCommand(rx_buf, Size);
        }
        else if (rx_buf[0] == UKB_HEADER_DATA && ukb_mode == Test_SUT) {
            /* 0xAB synthetic sensor data during SUT (Tablo 4). Copy out
             * now - rx_buf is handed straight back to DMA below and will
             * be overwritten by the next burst. App_Run consumes
             * ukb_sut_data and clears flag_sut_data_ready once done. */
            uint16_t n = (Size < UKB_SUT_DATA_LEN) ? Size : UKB_SUT_DATA_LEN;
            memcpy((void *)ukb_sut_data, rx_buf, n);
            ukb_sut_data_len   = n;
            flag_sut_data_ready = 1;
        }
        /* else: stray bytes, drop */
    }

    /* Re-arm IDLE-DMA reception for the next burst */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buf, RX_BUF_LEN);
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
}

/* ================================================================ */
/*  Outbound checksum per section 3: sum of bytes mod 256.          */
/*  'buf' points at the first byte to include; 'n' = count.         */
/*  Per Tablo 3/6 the checksum covers everything from Header up to  */
/*  (but not including) the checksum byte itself.                   */
/* ================================================================ */
uint8_t UKB_Checksum(const uint8_t *buf, uint16_t n)
{
    uint16_t sum = 0;
    for (uint16_t i = 0; i < n; i++) {
        sum += buf[i];
    }
    return (uint8_t)(sum & 0xFFu);
}
