/*
 * rs232_loopback.c
 * USART6 physical-layer loopback / echo bring-up test.
 *
 * Purpose: find WHERE the RS232 chain is broken, by moving the loopback
 * point progressively further from the MCU. Each stage that passes clears
 * everything upstream of it, so the first stage that fails contains the fault.
 *
 *   Stage 1 - MCU pins     : jumper PA11 (TX) to PA12 (RX).
 *                            Proves the USART, its clock, baud and the GPIO
 *                            alternate-function mapping. NOTE: if the MAX3232
 *                            is fitted, its receiver output R1OUT also drives
 *                            PA12 and will fight the jumper - lift that net
 *                            (or pull the chip) before trusting this stage.
 *   Stage 2 - MAX3232 side : jumper T1OUT to R1IN, i.e. the RS232-level pins,
 *                            before the DB9. Adds the transceiver and its
 *                            charge pump. No contention, so this is the most
 *                            informative single test.
 *   Stage 3 - DB9 header   : jumper pin 2 to pin 3 at the connector. Adds the
 *                            board-to-connector wiring and the connector itself.
 *
 * Reading the result:
 *   - 0 bytes returned      -> open circuit: wrong pins, TX/RX not crossed, or
 *                              (stage 2) a dead charge pump - check V+/V- on
 *                              the MAX3232, they should sit near +/-5..6 V.
 *   - bytes returned, wrong -> framing/level problem, not an open. FE on most
 *                              bytes means the bit timing or polarity is wrong
 *                              (baud mismatch, or RS232 levels reaching the MCU
 *                              directly because the transceiver is bypassed).
 *   - 0x00 or 0xFF returned -> line stuck at one level.
 *
 * This path deliberately shares nothing with rs232.c: no DMA, no frame
 * assembler, no protocol. Polled single-byte transfers only, so a failure
 * here is a hardware fault and cannot be an artefact of the receive path.
 * Results print on the DEBUG CONSOLE (USART2, 9600 baud) and therefore need
 * DEBUG_PRINTS_ENABLED = 1 in debug_uart.h.
 */
#include "rs232_loopback.h"
#include "debug_uart.h"
#include "usart.h"
#include <stdio.h>

/* Mixed 1/0 runs, both all-ones and all-zeros, and both alternating phases.
 * 0x55/0xAA are the ones that fail first on a marginal baud rate; 0x00 and
 * 0xFF are the ones that still "pass" on a line stuck at a level, which is
 * why the summary reports the byte pattern and not just a pass count. */
static const uint8_t kPattern[] = { 0x55, 0xAA, 0x00, 0xFF, 0x0F, 0xF0, 0x5A, 0xA5 };
#define PATTERN_LEN ((uint16_t)(sizeof(kPattern) / sizeof(kPattern[0])))

#define LB_ROUND_MS      1000u  /* one self-test round per second        */
#define LB_BYTE_TMO_MS     50u  /* generous: one byte is ~87 us @115200  */
#define LB_MAX_DETAIL       3u  /* cap failure detail lines (9600 baud)  */
#define LB_MSG_MAX         96u  /* readable round-trip message buffer    */

static RS232_LB_Mode lb_mode;
static uint32_t      lb_last_tick;
static uint32_t      lb_round;

/* Echo-mode counters, also useful to read over SWD. */
static uint32_t lb_echo_bytes;
static uint32_t lb_echo_errors;

/* Latched for SWD inspection when the console is unavailable. */
uint32_t rs232_lb_rounds_pass;
uint32_t rs232_lb_rounds_fail;
uint8_t  rs232_lb_last_rx[PATTERN_LEN];
uint32_t rs232_lb_last_sr[PATTERN_LEN];

/* Text-phase result, also latched for SWD. BinaryPatternCheck only runs when
 * the text phase passes completely, so on a failing link rs232_lb_last_rx/_sr
 * above are never written and stay at their BSS zeros - which is silence, not
 * evidence. These four are the ones that tell a FAILING link apart:
 *
 *   returned == 0            open circuit: no jumper, wrong pins, or a dead
 *                            charge pump leaving T1OUT unable to drive
 *   returned == len,
 *     correct < len          the loop is closed and bytes are flowing, but
 *                            levels/timing are marginal - check first_sr
 *   0 < returned < len       intermittent, usually slew rate at 115200
 *
 * first_sr is the USART SR captured at the first byte that did not round-trip
 * correctly, so FE/NE/ORE is readable without the console. */
uint32_t rs232_lb_last_returned;
uint32_t rs232_lb_last_correct;
uint32_t rs232_lb_last_len;
uint32_t rs232_lb_last_first_sr;
char     rs232_lb_last_back[LB_MSG_MAX];

/* Drain and clear any stale byte or sticky error flag. On F4 the ORE/FE/NE/PE
 * flags clear on a read of SR followed by a read of DR, in that order. */
static void Usart6_Flush(void)
{
    volatile uint32_t scratch;
    for (uint8_t i = 0; i < 4; i++) {
        scratch = huart6.Instance->SR;
        scratch = huart6.Instance->DR;
    }
    (void)scratch;
    huart6.ErrorCode = HAL_UART_ERROR_NONE;
}

/* Hand-rolled single-byte receive. HAL_UART_Receive would work, but it reads
 * DR itself and so clears the error flags before we can see them - and which
 * error fired is the whole point of this test. Snapshot SR first, then DR. */
static uint8_t Usart6_ReceiveByte(uint8_t *out, uint32_t *sr_out, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((huart6.Instance->SR & USART_SR_RXNE) == 0u) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            *sr_out = huart6.Instance->SR;
            *out    = 0u;
            return 0u; /* timed out - nothing came back */
        }
    }

    *sr_out = huart6.Instance->SR;
    *out    = (uint8_t)(huart6.Instance->DR & 0xFFu);
    return 1u;
}

static const char *SrErrorText(uint32_t sr)
{
    if (sr & USART_SR_FE)  { return " FE";  }  /* framing: baud/polarity   */
    if (sr & USART_SR_NE)  { return " NE";  }  /* noise on the line        */
    if (sr & USART_SR_ORE) { return " ORE"; }  /* we were too slow         */
    if (sr & USART_SR_PE)  { return " PE";  }  /* parity (unused at 8N1)   */
    return "";
}

void RS232_Loopback_Init(RS232_LB_Mode mode)
{
    lb_mode        = mode;
    lb_last_tick   = HAL_GetTick();
    lb_round       = 0u;
    lb_echo_bytes  = 0u;
    lb_echo_errors = 0u;

    rs232_lb_rounds_pass = 0u;
    rs232_lb_rounds_fail = 0u;

    Usart6_Flush();

    DBG_PRINT("\r\n=== RS232 LOOPBACK TEST (USART6 115200 8N1) ===\r\n");
    if (mode == RS232_LB_SELFTEST) {
        DBG_PRINT("Mode: SELFTEST - fit a jumper, board sends and verifies.\r\n");
        DBG_PRINT("  stage 1 MCU     : PA11 <-> PA12 (lift MAX3232 R1OUT first)\r\n");
        DBG_PRINT("  stage 2 MAX3232 : T1OUT <-> R1IN\r\n");
        DBG_PRINT("  stage 3 DB9     : pin 2 <-> pin 3\r\n");
    } else {
        DBG_PRINT("Mode: ECHO - no jumper; PC sends, board mirrors back.\r\n");
    }
    DBG_PRINT("Normal flight logic is DISABLED in this build.\r\n\r\n");
}

/* Send one byte and read the byte the loop returns. Returns 1 if a byte came
 * back at all (whether or not it was the right one), 0 on timeout. */
static uint8_t LoopByte(uint8_t tx, uint8_t *rx, uint32_t *sr)
{
    *rx = 0u;
    *sr = 0u;

    /* One byte in flight at a time. The F4 USART has no RX FIFO, so blasting
     * a whole string out and reading it back afterwards would overrun on every
     * byte but the last - and would also lose which byte failed. */
    if (HAL_UART_Transmit(&huart6, &tx, 1u, LB_BYTE_TMO_MS) != HAL_OK) {
        return 0u; /* TX failing points at the handle/clock, not the wiring */
    }
    return Usart6_ReceiveByte(rx, sr, LB_BYTE_TMO_MS);
}

/* Show a returned byte as itself when it is printable, '.' otherwise, so a
 * corrupted line stays readable instead of throwing control codes at the
 * terminal. */
static char Printable(uint8_t b)
{
    return (b >= 0x20u && b < 0x7Fu) ? (char)b : '.';
}

/* Phase 2: the bytes an ASCII message cannot carry - all-zeros, all-ones and
 * both alternating phases. Silent unless it fails, so the common case stays a
 * single readable line. Returns the number of bytes that round-tripped. */
static uint16_t BinaryPatternCheck(void)
{
    uint16_t ok = 0u;

    for (uint16_t i = 0; i < PATTERN_LEN; i++) {
        uint8_t  rx;
        uint32_t sr;
        uint8_t  got = LoopByte(kPattern[i], &rx, &sr);

        rs232_lb_last_rx[i] = rx;
        rs232_lb_last_sr[i] = sr;
        if (got && rx == kPattern[i]) { ok++; }
    }
    return ok;
}

static void RS232_Loopback_SelfTest(void)
{
    char     msg[LB_MSG_MAX];
    char     back[LB_MSG_MAX];
    uint32_t sr[LB_MSG_MAX];
    uint8_t  got[LB_MSG_MAX];
    uint8_t  okb[LB_MSG_MAX];
    uint16_t n_returned = 0u;
    uint16_t n_correct  = 0u;

    lb_round++;
    Usart6_Flush();

    /* The round number goes INSIDE the message on purpose. A fixed string
     * could in principle be satisfied by a stale byte sitting in a buffer or
     * by something else on the line repeating itself; a message that has to
     * come back carrying this round's own counter cannot. The 'U' run is
     * 0x55 - the classic alternating-bit baud stress - which happens to be
     * printable, so it costs nothing to include here. */
    int n = snprintf(msg, sizeof msg,
                     "LOOP-%04lu UUUU The quick brown fox 0123456789",
                     (unsigned long)lb_round);
    if (n < 0) { return; }
    uint16_t len = (uint16_t)n;
    if (len >= sizeof msg) { len = (uint16_t)(sizeof msg - 1u); }

    for (uint16_t i = 0; i < len; i++) {
        uint8_t rx;
        got[i] = LoopByte((uint8_t)msg[i], &rx, &sr[i]);
        okb[i] = (uint8_t)(got[i] && rx == (uint8_t)msg[i]);

        /* '_' marks a slot nothing came back for, which reads differently from
         * '.' (a byte arrived but was not printable). */
        back[i] = got[i] ? Printable(rx) : '_';

        if (got[i])  { n_returned++; }
        if (okb[i])  { n_correct++;  }
    }
    back[len] = '\0';

    /* Latch before any early return, so a failing round is just as readable
     * over SWD as a passing one. */
    rs232_lb_last_returned = n_returned;
    rs232_lb_last_correct  = n_correct;
    rs232_lb_last_len      = len;
    rs232_lb_last_first_sr = 0u;
    for (uint16_t i = 0; i < len; i++) {
        if (!okb[i]) { rs232_lb_last_first_sr = sr[i]; break; }
    }
    for (uint16_t i = 0; i < LB_MSG_MAX; i++) {
        rs232_lb_last_back[i] = (i <= len) ? back[i] : '\0';
    }

    if (n_returned == 0u) {
        rs232_lb_rounds_fail++;
        DBG_PRINT("LB %lu: FAIL - nothing came back (open circuit / TX-RX not crossed)\r\n",
                  (unsigned long)lb_round);
        return;
    }

    /* Print what CAME BACK, never what was sent. Reading this line and seeing
     * the current round number in it is the proof that the round trip is real
     * and fresh, which a pass count alone cannot give you. */
    if (n_correct == len) {
        uint16_t bin_ok = BinaryPatternCheck();
        if (bin_ok == PATTERN_LEN) {
            rs232_lb_rounds_pass++;
            DBG_PRINT("LB %lu PASS rx=\"%s\"\r\n", (unsigned long)lb_round, back);
        } else {
            rs232_lb_rounds_fail++;
            DBG_PRINT("LB %lu TEXT OK but binary %u/%u rx=\"%s\"\r\n",
                      (unsigned long)lb_round, bin_ok, PATTERN_LEN, back);
            DBG_PRINT("   (00/FF/55/AA fail while text passes = level or slew problem)\r\n");
        }
        return;
    }

    rs232_lb_rounds_fail++;
    DBG_PRINT("LB %lu FAIL %u/%u rx=\"%s\"\r\n",
              (unsigned long)lb_round, n_correct, len, back);

    /* Detail only for the bad bytes, and capped: each line is ~34 chars, which
     * is ~35 ms of blocking at 9600 baud on the debug console. */
    uint8_t shown = 0u;
    for (uint16_t i = 0; i < len && shown < LB_MAX_DETAIL; i++) {
        if (okb[i]) { continue; }
        if (!got[i]) {
            DBG_PRINT("   [%u] tx='%c' no reply\r\n", i, Printable((uint8_t)msg[i]));
        } else {
            DBG_PRINT("   [%u] tx='%c'(%02X) rx='%c'%s\r\n",
                      i, Printable((uint8_t)msg[i]), (uint8_t)msg[i],
                      back[i], SrErrorText(sr[i]));
        }
        shown++;
    }
}

static void RS232_Loopback_Echo(void)
{
    uint8_t  b;
    uint32_t sr;

    /* Mirror everything available this iteration, without printing in the hot
     * path - at 115200 in, a per-byte console line at 9600 out would overrun
     * immediately. Counters are reported once a second instead. */
    while (Usart6_ReceiveByte(&b, &sr, 0u)) {
        if (sr & (USART_SR_FE | USART_SR_NE | USART_SR_ORE | USART_SR_PE)) {
            lb_echo_errors++;
        }
        lb_echo_bytes++;
        (void)HAL_UART_Transmit(&huart6, &b, 1u, LB_BYTE_TMO_MS);
    }

    if ((HAL_GetTick() - lb_last_tick) >= LB_ROUND_MS) {
        lb_last_tick = HAL_GetTick();
        DBG_PRINT("LB echo: %lu bytes, %lu errors\r\n",
                  (unsigned long)lb_echo_bytes, (unsigned long)lb_echo_errors);
    }
}

void RS232_Loopback_Run(void)
{
    if (lb_mode == RS232_LB_ECHO) {
        RS232_Loopback_Echo();
        return;
    }

    if ((HAL_GetTick() - lb_last_tick) < LB_ROUND_MS) {
        return;
    }
    lb_last_tick = HAL_GetTick();
    RS232_Loopback_SelfTest();
}
