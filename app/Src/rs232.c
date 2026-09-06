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
#include <math.h>
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

volatile uint32_t ukb_sut_rx_count   = 0;
volatile uint32_t ukb_sut_cks_errors = 0;

/* Checksum byte the SPEC's Tablo 1 lists for each command. The real test
 * software does NOT send these - it sends (0xAA + cmd) & 0xFF, the same
 * header-inclusive rule confirmed for outbound packets (observed on the
 * wire: SIT = AA 20 CA, STOP = AA 24 CE). We accept either, so the board
 * works against the real software and against a spec-literal device. */
#define CKS_SPEC_SIT 0x8Cu
#define CKS_SPEC_SUT 0x8Eu
#define CKS_SPEC_STP 0x90u

/* Read cursor into the circular DMA buffer: the index of the first byte we
 * have not consumed yet. Only touched from ISR context. */
static volatile uint16_t rx_read_pos = 0;

/* ---- Byte-wise frame assembler ----
 * Fed one byte at a time, so it does not care how the DMA happens to
 * fragment a frame across bursts or how the ring buffer wraps. */
static uint8_t  frame[UKB_SENSOR_PACKET_LEN];
static uint16_t frame_len    = 0;
static uint16_t frame_expect = 0;

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
    rx_read_pos  = 0;
    frame_len    = 0;
    frame_expect = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buf, RX_BUF_LEN);
}

Test_Status UKB_RS232_GetMode(void)
{
    return ukb_mode;
}

/* ================================================================ */
/*  Complete-frame handler                                          */
/*  Sets event flags only - never calls into app-level code.        */
/* ================================================================ */
static void UKB_HandleFrame(const uint8_t *f, uint16_t len)
{
    if (len == CMD_FRAME_LEN && f[0] == UKB_HEADER) {
        /* Accept the checksum the real software sends (header-inclusive)
         * or the one Tablo 1 documents. See CKS_SPEC_* above. */
        uint8_t computed = UKB_Checksum(f, 2u); /* header + command */
        uint8_t spec;
        switch (f[1]) {
            case CMD_SIT_START: spec = CKS_SPEC_SIT; break;
            case CMD_SUT_START: spec = CKS_SPEC_SUT; break;
            case CMD_STOP:      spec = CKS_SPEC_STP; break;
            default:            return; /* unknown command */
        }
        if (f[2] != computed && f[2] != spec) {
            return; /* bad checksum */
        }

        switch (f[1]) {
            case CMD_SIT_START:
                ukb_mode = Test_SIT;
                flag_sut_pending  = 0;
                flag_stop_pending = 0;
                flag_sit_pending  = 1; /* App_Run: arm 1 s timer, then 10 Hz TX */
                break;
            case CMD_SUT_START:
                ukb_mode = Test_SUT;
                flag_sit_pending  = 0;
                flag_stop_pending = 0;
                flag_sut_pending  = 1;
                break;
            case CMD_STOP:
                ukb_mode = Test_Stop;
                flag_sit_pending  = 0;
                flag_sut_pending  = 0;
                flag_stop_pending = 1; /* App_Run: tear down -> normal mode */
                break;
            default:
                break;
        }
    }
    else if (len == UKB_SENSOR_PACKET_LEN && f[0] == UKB_HEADER_DATA
             && ukb_mode == Test_SUT) {
        /* Synthetic sensor data during SUT (Tablo 4). App_Run consumes
         * ukb_sut_data and clears flag_sut_data_ready once done.
         *
         * Accept EITHER checksum range. Section 3 only says "sum of the bytes
         * mod 256" and never states where the sum starts; our own outbound
         * packets include the header because that is what the ground software
         * demanded on the bench, but nothing proves the test device applies
         * the same rule to what it sends us. Rejecting every synthetic packet
         * over a one-byte disagreement would look exactly like a dead test,
         * so take either and count the misses instead. */
        uint8_t cks_with_header = UKB_Checksum(f, 33u);      /* [0..32]  */
        uint8_t cks_payload     = UKB_Checksum(&f[1], 32u);  /* [1..32]  */

        if (f[33] != cks_with_header && f[33] != cks_payload) {
            ukb_sut_cks_errors++;
            return; /* corrupt - drop it rather than fly on bad data */
        }

        memcpy((void *)ukb_sut_data, f, UKB_SENSOR_PACKET_LEN);
        ukb_sut_data_len    = UKB_SENSOR_PACKET_LEN;
        ukb_sut_rx_count++;
        flag_sut_data_ready = 1;
    }
}

/* Feed one received byte into the frame assembler. */
static void UKB_FeedByte(uint8_t b)
{
    if (frame_len == 0u) {
        /* Waiting for a header; anything else is inter-frame noise. */
        if (b == UKB_HEADER) {
            frame_expect = CMD_FRAME_LEN;
        } else if (b == UKB_HEADER_DATA && ukb_mode == Test_SUT) {
            /* A 36-byte data packet is only ever legitimate during SUT -
             * UKB_HandleFrame rejects it in any other mode. Refusing to arm
             * one outside SUT is what stops a single noise byte deafening the
             * board: 0xAA and 0xAB differ only in bit 0, the first bit on the
             * wire after the start bit, so one flipped bit turns a command
             * header into a data header. That armed a 36-byte frame which
             * then swallowed the next 35 bytes - seven whole commands - and
             * the board looked dead the entire time. Reproduced on the bench
             * by injecting one 0xAB; recovery took exactly 7 retries.
             *
             * Gating on the mode kills that at the source and, unlike an
             * idle-gap resync, assumes nothing about how the sender chunks
             * its bytes. That matters: a USB-serial driver's latency timer
             * can split a 5-byte command into fragments, and any scheme that
             * discards a partial frame on an idle line loses every such
             * command - measured at 100% loss, at gaps from 1 ms to 50 ms. */
            frame_expect = UKB_SENSOR_PACKET_LEN;
        } else {
            return;
        }
        frame[frame_len++] = b;
        return;
    }

    frame[frame_len++] = b;

    if (frame_len >= frame_expect) {
        /* Footers are the frame's self-check; if they are absent we were
         * mis-synchronised, so drop it and resync on the next header. */
        if (frame[frame_expect - 2u] == UKB_FOOTER1 &&
            frame[frame_expect - 1u] == UKB_FOOTER2) {
            UKB_HandleFrame(frame, frame_expect);
        }
        frame_len    = 0u;
        frame_expect = 0u;
    }
}

/* ================================================================ */
/*  HAL RX-event callback: IDLE line, half-transfer, or wrap.       */
/*  The DMA is CIRCULAR (see cube.ioc), so 'Size' is the absolute   */
/*  write index into rx_buf - NOT a per-burst byte count, and it    */
/*  does not reset between callbacks. We therefore consume forward  */
/*  from our own read cursor and let the assembler find frames.     */
/* ================================================================ */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART6) {
        return;
    }

    /* A completely full buffer is reported as RX_BUF_LEN, which is index 0
     * wrapped around; normalising here keeps the loop below terminating. */
    uint16_t write_pos = (Size >= RX_BUF_LEN) ? 0u : Size;

    while (rx_read_pos != write_pos) {
        UKB_FeedByte(rx_buf[rx_read_pos]);
        rx_read_pos++;
        if (rx_read_pos >= RX_BUF_LEN) {
            rx_read_pos = 0u;
        }
    }

    /* Deliberately NOT resetting the frame assembler on an idle line.
     *
     * It looks tempting - a stale fragment left by noise would be cleared -
     * and it was tried here. It is wrong, twice over, and both failures were
     * measured on the bench:
     *
     *  - It assumes a frame arrives as one contiguous burst. A USB-serial
     *    driver's latency timer can split a 5-byte command into fragments
     *    separated by milliseconds of idle line, and discarding the partial
     *    frame then loses EVERY such command - 100% loss at gaps from 1 ms to
     *    50 ms, versus the ~3% it was trying to fix.
     *
     *  - Even placed after the consume loop, it interacts badly with the
     *    circular buffer wrap. A frame straddling the wrap has its head
     *    delivered on the transfer-complete event and its tail on IDLE.
     *
     * The wedge this was meant to cure is handled at its source instead - see
     * the mode gate in UKB_FeedByte, which needs no timing assumption at all.
     *
     * Circular DMA never stops, so there is nothing to re-arm here. */
}

/* A UART error (overrun, framing, noise) aborts DMA reception and would
 * otherwise leave the board permanently deaf to commands. Restart it.
 *
 * READ THE HAL BEFORE TOUCHING THIS. stm32f4xx_hal_uart.c,
 * HAL_UART_IRQHandler, under "If Overrun error occurs, or if any error
 * occurs in DMA mode reception, consider error as blocking": with DMAR set,
 * EVERY error flag - FE, NE and ORE alike, not just overrun - makes the HAL
 * call UART_EndRxTransfer (RxState -> READY, EIE/IDLEIE off, DMAR off) and
 * abort the DMA stream. We are then called from the abort-complete callback
 * with reception already dead. Nothing restarts it unless we do it here.
 *
 * An earlier version of this function did nothing but clear ErrorCode, on
 * the belief that DMA-mode errors were non-blocking. Measured 2026-09-07:
 * a SUT run latched KTE and then froze - the board kept streaming its 10 Hz
 * status word for the remaining 20 s of the profile while seeing none of
 * it, ignored the STOP that followed and ten more STOPs after that, and
 * only a power cycle brought it back. That is precisely one noise/framing
 * hit at 115200 through a MAX3232, which is running near its rated 120 kbps,
 * turning into a failed algorithm test.
 *
 * What this does, in order:
 *  1. Drain what the DMA had already written before it was aborted. NDTR
 *     still holds the remaining count after an abort, so the write index is
 *     RX_BUF_LEN - NDTR. Those bytes are real and the assembler keeps its
 *     partial-frame state, so a frame straddling the error survives intact.
 *  2. Re-arm HAL_UARTEx_ReceiveToIdle_DMA from the top of rx_buf and reset
 *     the read cursor to match. UART_Start_Receive_DMA clears the sticky
 *     flags (SR read then DR read) before enabling DMAR, so it cannot
 *     immediately re-trip on the same event.
 *
 * Two things this must NOT do, both learned the hard way on the bench:
 *
 *  - Do not touch DR ourselves. The DMA is the only thing that should be
 *    draining DR; a CPU read steals a byte out of the stream. The HAL's
 *    restart path does the one dummy read that is needed, at the one
 *    moment it is safe (reception is stopped).
 *
 *  - Do not restart while RxState is still BUSY_RX, and never judge that
 *    with HAL_UART_GetState() - it returns gState | RxState, so during any
 *    blocking transmit it reads as "not BUSY_RX", and restarting a HEALTHY
 *    DMA rewinds the write pointer while rx_read_pos stays put. Measured: a
 *    complete command sat at offset 23 in rx_buf while the cursor had been
 *    reset to 0 and the write pointer to 2, so it could never be consumed.
 *    Test huart->RxState directly; it is READY only after the HAL has
 *    genuinely ended the transfer.
 *
 * Diagnostics, read over SWD after a run: ukb_rx_errors counts every hit,
 * ukb_rx_last_error is the HAL_UART_ERROR_* mask of the latest one
 * (0x01 PE, 0x02 NE, 0x04 FE, 0x08 ORE), ukb_rx_restarts how many times
 * reception actually had to be re-armed. A steadily climbing error count
 * with a clean-looking test means the line itself is marginal.
 */
volatile uint32_t ukb_rx_errors     = 0;
volatile uint32_t ukb_rx_last_error = 0;
volatile uint32_t ukb_rx_restarts   = 0;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART6) {
        return;
    }

    ukb_rx_errors++;
    ukb_rx_last_error = huart->ErrorCode;
    huart->ErrorCode  = HAL_UART_ERROR_NONE;

    if (huart->RxState != HAL_UART_STATE_READY) {
        /* Non-blocking path (never taken in DMA mode, kept for safety):
         * reception is still running, so there is nothing to repair. */
        return;
    }

    /* Reception is dead. Consume what arrived before the abort, from our
     * cursor up to where the DMA stopped writing. */
    if (huart->hdmarx != NULL) {
        uint32_t remaining = __HAL_DMA_GET_COUNTER(huart->hdmarx);
        uint16_t write_pos = (remaining >= RX_BUF_LEN)
                             ? 0u : (uint16_t)(RX_BUF_LEN - remaining);
        while (rx_read_pos != write_pos) {
            UKB_FeedByte(rx_buf[rx_read_pos]);
            rx_read_pos++;
            if (rx_read_pos >= RX_BUF_LEN) {
                rx_read_pos = 0u;
            }
        }
    }

    /* Re-arm from the top of the buffer; the cursor must follow it. The
     * frame assembler is deliberately left alone - see above. */
    rx_read_pos = 0u;
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buf, RX_BUF_LEN) == HAL_OK) {
        ukb_rx_restarts++;
    }
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

/* ================================================================ */
/*  Tablo 3 sensor packet (SIT telemetry)                           */
/* ================================================================ */

/* Which bytes the outbound checksum covers. Section 3 says "sum of all
 * bytes in the data mod 256" but never pins down the range, and the
 * worked examples in Tablo 1/6 match NEITHER reading (Tablo 6: header
 * 0xAA + 0x14 + 0x00 = 0xBE, payload-only = 0x14, listed checksum 0x1A).
 *
 * RESOLVED ON THE BENCH: the test software rejected an idle status packet
 * (AA 00 00 ...) with "expected 0xAA, received 0x00" - i.e. it wants
 * 0xAA + 0x00 + 0x00, so the sum DOES include the header byte. Range is
 * header through the last data byte, excluding the checksum and footers. */
#define UKB_CHECKSUM_INCLUDES_HEADER 1

/* Byte order inside each FLOAT32.
 *
 * Section 4.2's conversion example copies the union's bytes out in index
 * order, which on Cortex-M is little-endian, and that is how this was first
 * implemented. WRONG: on the bench the ground software decoded altitude and
 * pressure as nonsense while every all-zero field looked fine - the classic
 * byte-swap signature, since 0.0f is byte-order agnostic. The checksum did
 * not catch it because a sum is order-independent.
 *
 * The spec's "Big Endian" note therefore means what it says, and applies to
 * the bytes within a FLOAT32, not just to field order. Send MSB first.
 * NOTE: when SUT lands, inbound synthetic floats must be unpacked the same
 * way round. */
#define UKB_FLOAT_BIG_ENDIAN 1

typedef union {
    float   f32;
    uint8_t bytes[4];
} UKB_FloatBytes;

/* Test device only accepts 2 decimal places (section 2.1.1). */
static float UKB_Round2(float v)
{
    return roundf(v * 100.0f) / 100.0f;
}

static uint16_t UKB_PackFloat(uint8_t *dst, float v)
{
    UKB_FloatBytes c;
    c.f32 = UKB_Round2(v);
#if UKB_FLOAT_BIG_ENDIAN
    dst[0] = c.bytes[3];
    dst[1] = c.bytes[2];
    dst[2] = c.bytes[1];
    dst[3] = c.bytes[0];
#else
    dst[0] = c.bytes[0];
    dst[1] = c.bytes[1];
    dst[2] = c.bytes[2];
    dst[3] = c.bytes[3];
#endif
    return 4u;
}

/* Inverse of UKB_PackFloat: 4 wire bytes -> float, same byte order. */
static float UKB_UnpackFloat(const uint8_t *src)
{
    UKB_FloatBytes c;
#if UKB_FLOAT_BIG_ENDIAN
    c.bytes[3] = src[0];
    c.bytes[2] = src[1];
    c.bytes[1] = src[2];
    c.bytes[0] = src[3];
#else
    c.bytes[0] = src[0];
    c.bytes[1] = src[1];
    c.bytes[2] = src[2];
    c.bytes[3] = src[3];
#endif
    return c.f32;
}

bool UKB_ParseSensorPacket(const uint8_t *pkt, UKB_SensorSample *s)
{
    if (pkt == NULL || s == NULL || pkt[0] != UKB_HEADER_DATA) {
        return false;
    }

    /* Field order is Tablo 4, which is the same layout we transmit in Tablo 3.
     * Offsets: header at [0], then eight consecutive FLOAT32s from [1]. */
    s->altitude_m    = UKB_UnpackFloat(&pkt[1]);
    s->pressure_mbar = UKB_UnpackFloat(&pkt[5]);
    s->acc_x         = UKB_UnpackFloat(&pkt[9]);
    s->acc_y         = UKB_UnpackFloat(&pkt[13]);
    s->acc_z         = UKB_UnpackFloat(&pkt[17]);
    s->ang_x         = UKB_UnpackFloat(&pkt[21]);
    s->ang_y         = UKB_UnpackFloat(&pkt[25]);
    s->ang_z         = UKB_UnpackFloat(&pkt[29]);

    return true;
}

HAL_StatusTypeDef UKB_RS232_SendSensorPacket(const UKB_SensorSample *s)
{
    uint8_t  pkt[UKB_SENSOR_PACKET_LEN];
    uint16_t i = 0;

    pkt[i++] = UKB_HEADER_DATA;             /* 0xAB */

    i += UKB_PackFloat(&pkt[i], s->altitude_m);
    i += UKB_PackFloat(&pkt[i], s->pressure_mbar);
    i += UKB_PackFloat(&pkt[i], s->acc_x);
    i += UKB_PackFloat(&pkt[i], s->acc_y);
    i += UKB_PackFloat(&pkt[i], s->acc_z);
    i += UKB_PackFloat(&pkt[i], s->ang_x);
    i += UKB_PackFloat(&pkt[i], s->ang_y);
    i += UKB_PackFloat(&pkt[i], s->ang_z);
    /* i == 33 here: header + 8 floats */

#if UKB_CHECKSUM_INCLUDES_HEADER
    uint8_t cks = UKB_Checksum(pkt, i);
#else
    uint8_t cks = UKB_Checksum(&pkt[1], i - 1u);
#endif
    pkt[i++] = cks;
    pkt[i++] = UKB_FOOTER1;
    pkt[i++] = UKB_FOOTER2;

    return HAL_UART_Transmit(&huart6, pkt, UKB_SENSOR_PACKET_LEN, 20);
}

/* ================================================================ */
/*  Tablo 6 status packet                                           */
/*  Doubles as the idle-mode link heartbeat: it is the smallest      */
/*  well-formed frame we can put on the wire, so the test software's */
/*  RX counter and "Checksum Hatasi" tally validate our framing and  */
/*  checksum range before a test is ever started.                    */
/* ================================================================ */
HAL_StatusTypeDef UKB_RS232_SendStatusPacket(uint16_t status_bits)
{
    uint8_t pkt[UKB_STATUS_PACKET_LEN];

    pkt[0] = UKB_HEADER;                            /* 0xAA          */
    pkt[1] = (uint8_t)(status_bits & 0xFFu);        /* Data1, bits 0-7  */
    pkt[2] = (uint8_t)((status_bits >> 8) & 0xFFu); /* Data2, bits 8-15 */
#if UKB_CHECKSUM_INCLUDES_HEADER
    pkt[3] = UKB_Checksum(pkt, 3u);
#else
    pkt[3] = UKB_Checksum(&pkt[1], 2u);
#endif
    pkt[4] = UKB_FOOTER1;
    pkt[5] = UKB_FOOTER2;

    return HAL_UART_Transmit(&huart6, pkt, UKB_STATUS_PACKET_LEN, 20);
}

HAL_StatusTypeDef UKB_RS232_SendText(const char *s)
{
    size_t n = strlen(s);
    if (n == 0u) {
        return HAL_OK;
    }
    /* ~11 us/char at 115200; a 100-char line blocks for about 9 ms. */
    return HAL_UART_Transmit(&huart6, (uint8_t *)s, (uint16_t)n, 50);
}
