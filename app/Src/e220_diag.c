/*
 * E220-900T30S configuration-register diagnostic (BENCH)
 * Author: Alpdogan
 */

#include "e220_diag.h"
#include "debug_uart.h"
#include "main.h"   // M0_Pin / M1_Pin / LORA_AUX_Pin

// Mode pins. MEASURED on this board, not taken from the E220 datasheet - the
// module fitted here follows the E22 table, where configuration is mode 2 and
// mode 3 is deep sleep with the UART shut down:
//   M1 M0 = 0 0  transmission (normal)
//           0 1  WOR
//           1 0  CONFIGURATION  <- answers C1/C0 here
//           1 1  deep sleep     <- deaf; every probe here reads as a dead module
// In configuration mode the serial port is fixed at 9600 8N1 regardless of
// what REG0 says.
#define E220_CFG_BAUD 9600u
#define E220_CFG_M1   1u
#define E220_CFG_M0   0u

// Register addresses, from the E22-900T30S user manual section 7.2. The NETID
// byte at 0x02 is the trap: it pushes REG0..REG3 to 0x03..0x06, one address
// higher than an E220-style map, so decoding this module as an E220 reads
// every field shifted by one and reports plausible nonsense.
#define E22_ADDR_ADDH   0x00u
#define E22_ADDR_ADDL   0x01u
#define E22_ADDR_NETID  0x02u
#define E22_ADDR_REG0   0x03u   // baud (7-5) | parity (4-3) | air rate (2-0)
#define E22_ADDR_REG1   0x04u   // sub-packet (7-6) | RSSI ambient (5) | power (1-0)
#define E22_ADDR_REG2   0x05u   // channel: freq = 850.125 + CH MHz
#define E22_ADDR_REG3   0x06u   // RSSI byte (7) | fixed (6) | repeater (5) | LBT (4) | WOR
#define E22_ADDR_CRYPTH 0x07u   // write-only, always reads back 0
#define E22_ADDR_CRYPTL 0x08u

// Decode tables, same source. Indices are the raw register bit fields.
static const uint32_t baud_tbl[8] = { 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200 };
static const uint32_t air_tbl[8]  = { 2400, 2400, 2400, 4800, 9600, 19200, 38400, 62500 };
static const uint16_t pkt_tbl[4]  = { 240, 128, 64, 32 };
static const uint8_t  pwr_tbl[4]  = { 30, 27, 24, 21 };   // dBm, -30S variant
static const char    *par_tbl[4]  = { "8N1", "8O1", "8E1", "8N1" };

// The module drops AUX while it is busy and raises it when it is ready to talk.
// Returns 1 if AUX is high on exit, 0 if it timed out low.
static uint8_t AuxWaitHigh(uint32_t timeout_ms)
{
    uint32_t t = HAL_GetTick();
    while (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_RESET) {
        if (HAL_GetTick() - t > timeout_ms) return 0;
    }
    return 1;
}

// Reconfigure the baud rate WITHOUT HAL_UART_DeInit. DeInit runs MspDeInit,
// which calls HAL_GPIO_DeInit on PA9/PA10 - that leaves the module's RXD line
// floating for the moment before MspInit restores it, and the resulting edge
// or break condition arrives at the module immediately before the command.
// HAL_UART_Init on an already-initialised handle skips MspInit and only
// recomputes BRR, so the TX line stays driven the whole time.
static void SetBaud(UART_HandleTypeDef *huart, uint32_t baud)
{
    huart->Init.BaudRate = baud;
    HAL_UART_Init(huart);
    HAL_Delay(5);
}

// One read attempt at the currently configured baud. Collects whatever comes
// back rather than demanding an exact length: a module that answers "FF FF FF"
// (parameter error) or replies short is a completely different fault from one
// that says nothing, and a fixed-length receive cannot tell them apart - it
// times out and discards the bytes either way.
//
// Also reports whether AUX dropped during the exchange, which separates "the
// module never heard the command" from "it answered and the reply did not
// reach PA10".
static uint8_t TryRead(UART_HandleTypeDef *huart, const uint8_t *cmd,
                       uint8_t *rsp, uint8_t rsp_max,
                       uint8_t *n_rx, uint8_t *aux_dropped)
{
    *n_rx = 0;
    *aux_dropped = 0;

    // Discard anything the module left in the receiver from the flight-mode
    // stream, or the reply parses one byte out of phase.
    __HAL_UART_CLEAR_OREFLAG(huart);
    (void)huart->Instance->DR;

    if (HAL_UART_Transmit(huart, (uint8_t *)cmd, 3, 200) != HAL_OK) return 1;

    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 500) {
        if (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_RESET) {
            *aux_dropped = 1;
        }
        if (*n_rx < rsp_max) {
            uint8_t b;
            if (HAL_UART_Receive(huart, &b, 1, 5) == HAL_OK) {
                rsp[(*n_rx)++] = b;
            }
        }
    }

    if (*n_rx == 0) return 2;                 // silence
    if (rsp[0] != 0xC1) return 3;             // answered, but not a register reply
    return (*n_rx >= 11) ? 0 : 4;             // C1-framed but short
}

// Sends a 48-byte dummy packet (same length as TelemetryPacket, so the timing
// is directly comparable to the flight path's aux_low_ms) and returns how long
// AUX stayed low. *went_low is 0 if AUX never dropped at all.
//
// The transmit is interrupt-driven and AUX is sampled across a fixed window
// that opens BEFORE it starts. Waiting for the AUX edges in sequence after a
// blocking transmit cannot work: the module drops AUX while the bytes are
// still going out, so a short pulse can begin and end inside HAL_UART_Transmit
// and be missed entirely - which reads as "no response" and looks exactly like
// a module that ignored the data.
#define SWEEP_WINDOW_MS 1200u

static uint32_t TimedDummyTx(UART_HandleTypeDef *huart, uint8_t *went_low, uint8_t routed)
{
    static uint8_t dummy[48];
    for (uint8_t i = 0; i < sizeof(dummy); i++) dummy[i] = i;

    // In fixed transmission mode the module strips the first three bytes as
    // target ADDH/ADDL/CH and only then decides whether to radiate. The flight
    // packet carries 7B D3 2B; a dummy that does not is asking to be routed to
    // address 0x0001 on channel 2, which is not the same experiment.
    if (routed) {
        dummy[0] = 0x7B; dummy[1] = 0xD3; dummy[2] = 0x2B; dummy[3] = 0xAB;
    }

    *went_low = 0;
    uint32_t first = 0, last = 0;

    if (HAL_UART_Transmit_IT(huart, dummy, sizeof(dummy)) != HAL_OK) return 0;

    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < SWEEP_WINDOW_MS) {
        if (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_RESET) {
            uint32_t now = HAL_GetTick();
            if (!*went_low) { *went_low = 1; first = now; }
            last = now;
        }
    }

    // Do not leave a transfer in flight - the next row reuses the buffer.
    while (huart->gState != HAL_UART_STATE_READY) { }

    return *went_low ? (last - first) : 0;
}

uint8_t E220_Diag_WriteConfig(UART_HandleTypeDef *huart, uint8_t start_addr,
                              const uint8_t *vals, uint8_t len)
{
    // Writes a raw block of registers: C0 + start address + length + values,
    // saved to the module's flash. Explicit addresses rather than named
    // registers, because naming them is exactly where this went wrong - a
    // "write REG0 and REG1" helper that assumed an E220 layout actually wrote
    // NETID and REG0 on this module, setting its UART to a baud the firmware
    // does not speak and later moving it off its channel. See the E22_ADDR_*
    // defines at the top; the manual's map is the authority, not the field
    // names in any helper's signature.
    uint8_t cmd[3 + 16];
    if (len == 0 || len > 16) return 2;

    cmd[0] = 0xC0;
    cmd[1] = start_addr;
    cmd[2] = len;
    for (uint8_t i = 0; i < len; i++) cmd[3 + i] = vals[i];

    uint8_t  rsp[24] = {0};
    uint8_t  n_rx = 0;
    uint32_t saved_baud = huart->Init.BaudRate;

    DBG_PRINT("\r\n--- E220 config write ---\r\n");
    DBG_PRINT("C0 %02X %02X :", start_addr, len);
    for (uint8_t i = 0; i < len; i++) DBG_PRINT(" %02X", vals[i]);
    DBG_PRINT("\r\n");

    HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, E220_CFG_M1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, E220_CFG_M0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_Delay(200);
    AuxWaitHigh(1000);

    SetBaud(huart, E220_CFG_BAUD);
    __HAL_UART_CLEAR_OREFLAG(huart);
    (void)huart->Instance->DR;

    // 3 + len, NOT sizeof(cmd) - the buffer is fixed-size and mostly unused.
    if (HAL_UART_Transmit(huart, cmd, (uint16_t)(3 + len), 200) != HAL_OK) {
        DBG_PRINT("write transmit failed\r\n");
    }

    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 500) {
        if (n_rx < sizeof(rsp)) {
            uint8_t b;
            if (HAL_UART_Receive(huart, &b, 1, 5) == HAL_OK) rsp[n_rx++] = b;
        }
    }

    DBG_PRINT("write reply:");
    for (uint8_t i = 0; i < n_rx && i < 16; i++) DBG_PRINT(" %02X", rsp[i]);
    if (n_rx == 0) DBG_PRINT(" (none)");
    DBG_PRINT("\r\n");

    // Back to transmission mode at the flight baud - which the module should
    // now actually be speaking.
    HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_RESET);
    HAL_Delay(200);
    SetBaud(huart, saved_baud);
    AuxWaitHigh(1000);

    DBG_PRINT("--- end config write ---\r\n\r\n");
    return (n_rx > 0 && rsp[0] == 0xC1) ? 0 : 1;
}

void E220_Diag_BaudSweep(UART_HandleTypeDef *huart)
{
    // Sweep OUR baud, not the module's. Everything up to here has trusted my
    // reading of REG0's baud field; this makes no assumption about it. Send
    // one isolated 48-byte packet at each rate and watch AUX: at the module's
    // true baud it receives a whole packet and keys the PA (hundreds of ms at
    // 2.4 kbps), at every other rate it salvages a couple of framing-error
    // bytes and does nothing.
    static uint8_t pkt[48];
    static const uint32_t bauds[8] = { 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200 };

    for (uint8_t i = 0; i < sizeof(pkt); i++) pkt[i] = i;
    pkt[0] = 0x7B; pkt[1] = 0xD3; pkt[2] = 0x2B; pkt[3] = 0xAB;

    uint32_t saved_baud = huart->Init.BaudRate;

    HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    AuxWaitHigh(1000);

    DBG_PRINT("\r\n--- E220 baud sweep (one 48 B packet per rate) ---\r\n");

    for (uint8_t b = 0; b < 8; b++) {
        SetBaud(huart, bauds[b]);
        HAL_Delay(500);                        // identical idle before each

        uint8_t  dropped = 0;
        uint32_t first = 0, last = 0;

        HAL_UART_Transmit(huart, pkt, sizeof(pkt), 1000);

        uint32_t t0 = HAL_GetTick();
        while (HAL_GetTick() - t0 < 1200) {
            if (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_RESET) {
                uint32_t now = HAL_GetTick();
                if (!dropped) { dropped = 1; first = now; }
                last = now;
            }
        }

        DBG_PRINT("  %6lu baud : aux low %lu ms%s\r\n", (unsigned long)bauds[b],
                  dropped ? (unsigned long)(last - first) : 0UL,
                  dropped ? "" : "  (never dropped)");
    }

    SetBaud(huart, saved_baud);
    DBG_PRINT("--- end baud sweep ---\r\n\r\n");
}

void E220_Diag_TriggerTest(UART_HandleTypeDef *huart)
{
    // Every register combination tried so far behaves the same way: a lone
    // 48-byte packet does not key the PA, a back-to-back stream does. That is
    // a property of how much data arrives, not of any register, so measure it
    // directly - identical idle before each send, only the size changes.
    //
    // If a big enough single write transmits, the fix is padding the telemetry
    // packet up to that size. If nothing transmits regardless of size, the
    // trigger is the sustained stream itself and padding will not help.
    static uint8_t buf[200];
    static const uint16_t sizes[4] = { 48, 96, 150, 200 };

    for (uint16_t i = 0; i < sizeof(buf); i++) buf[i] = (uint8_t)i;
    buf[0] = 0x7B; buf[1] = 0xD3; buf[2] = 0x2B; buf[3] = 0xAB;

    HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    AuxWaitHigh(1000);

    DBG_PRINT("\r\n--- E220 trigger test (2 s idle before each) ---\r\n");

    for (uint8_t s = 0; s < 4; s++) {
        HAL_Delay(2000);                       // settle: no serial activity at all

        uint8_t  dropped = 0;
        uint32_t first = 0, last = 0;

        HAL_UART_Transmit_IT(huart, buf, sizes[s]);

        uint32_t t0 = HAL_GetTick();
        while (HAL_GetTick() - t0 < 1500) {
            if (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_RESET) {
                uint32_t now = HAL_GetTick();
                if (!dropped) { dropped = 1; first = now; }
                last = now;
            }
        }
        while (huart->gState != HAL_UART_STATE_READY) { }

        DBG_PRINT("  %3u B : aux low %lu ms%s\r\n", sizes[s],
                  dropped ? (unsigned long)(last - first) : 0UL,
                  dropped ? "" : "  (never dropped)");
    }

    DBG_PRINT("--- end trigger test ---\r\n\r\n");
}

void E220_Diag_BurstTest(UART_HandleTypeDef *huart, uint32_t cadence_ms, uint8_t count)
{
    // Reproduces App_Run's LoRa path exactly - transmission mode, blocking
    // transmit, 48-byte routed packet, 200 ms cadence - and measures AUX the
    // same way the driver does. The sweep sends isolated packets and sees a
    // few ms; the flight path sees 190/380 ms. Only one of those can be air
    // time, and this says which condition produces it.
    static uint8_t pkt[48];
    for (uint8_t i = 0; i < sizeof(pkt); i++) pkt[i] = i;
    pkt[0] = 0x7B; pkt[1] = 0xD3; pkt[2] = 0x2B; pkt[3] = 0xAB;

    HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    AuxWaitHigh(1000);

    // Once the sub-packet size is large enough to swallow a whole telemetry
    // packet, AUX stops measuring air time - the module buffers the bytes and
    // releases AUX while the RF is still going, so 2.4 kbps and 9.6 kbps both
    // read as ~25 ms. Sending FASTER than the air can drain is what separates
    // them: the buffer fills, and AUX-low grows packet over packet.
    DBG_PRINT("\r\n--- E220 burst test (%u packets @%lu ms, blocking TX) ---\r\n",
              count, (unsigned long)cadence_ms);

    for (uint8_t k = 0; k < count; k++) {
        uint32_t start = HAL_GetTick();
        uint8_t  aux_before = (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_SET);

        HAL_UART_Transmit(huart, pkt, sizeof(pkt), 500);

        // Same sequence the driver uses: wait for the drop, then time the low.
        uint8_t  dropped = 0;
        uint32_t t = HAL_GetTick();
        while (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_SET) {
            if (HAL_GetTick() - t > 50) break;
        }
        if (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_RESET) {
            dropped = 1;
            t = HAL_GetTick();
            while (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_RESET) {
                if (HAL_GetTick() - t > 3000) break;
            }
        }
        uint32_t low = dropped ? (HAL_GetTick() - t) : 0;

        DBG_PRINT("  pkt %u: aux_before=%s  low=%s%lu ms\r\n", k,
                  aux_before ? "HIGH" : "LOW",
                  dropped ? "" : "(never dropped) ", (unsigned long)low);

        uint32_t elapsed = HAL_GetTick() - start;
        if (elapsed < cadence_ms) HAL_Delay(cadence_ms - elapsed);
    }

    DBG_PRINT("--- end burst test ---\r\n\r\n");
}

void E220_Diag_ModeSweep(UART_HandleTypeDef *huart)
{
    // (M1, M0) in datasheet order. Only the first and last are used by the
    // flight path, and those two are the same under an M0/M1 swap - so this
    // sweep is really testing whether the pins have any effect at all, and
    // whether the sense is inverted on this board.
    static const struct { uint8_t m1, m0; const char *name; } modes[4] = {
        { 0, 0, "0 transmission" },
        { 0, 1, "1 WOR sending " },
        { 1, 0, "2 WOR receive " },
        { 1, 1, "3 config/sleep" },
    };

    DBG_PRINT("\r\n--- E220 mode sweep (radio keys - antenna required) ---\r\n");
    DBG_PRINT("48 B dummy per row. Long AUX-low = real air time, few ms = buffered only.\r\n");

    for (uint8_t i = 0; i < 4; i++) {
        HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, modes[i].m1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, modes[i].m0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_Delay(100);          // mode change needs a few ms to settle
        AuxWaitHigh(1000);

        // Same mode, both payloads: plain counter bytes vs the flight packet's
        // routing header. A difference here means the module is in fixed
        // transmission mode and is deciding what to do from those three bytes.
        uint8_t  low_plain_seen = 0, low_routed_seen = 0;
        uint32_t low_plain  = TimedDummyTx(huart, &low_plain_seen, 0);
        HAL_Delay(500);
        uint32_t low_routed = TimedDummyTx(huart, &low_routed_seen, 1);

        DBG_PRINT("M1=%u M0=%u  mode %s : plain ", modes[i].m1, modes[i].m0, modes[i].name);
        if (low_plain_seen) DBG_PRINT("%lu ms", (unsigned long)low_plain);
        else                DBG_PRINT("no drop");
        DBG_PRINT("   routed ");
        if (low_routed_seen) DBG_PRINT("%lu ms\r\n", (unsigned long)low_routed);
        else                 DBG_PRINT("no drop\r\n");

        HAL_Delay(500);          // let the module finish and settle between rows
    }

    // Leave it where the flight path expects it.
    HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    AuxWaitHigh(1000);
    DBG_PRINT("--- end mode sweep ---\r\n\r\n");
}

uint8_t E220_Diag_ReadConfig(UART_HandleTypeDef *huart)
{
    uint8_t  rsp[24] = {0};               // C1 + addr + len + ADDH, ADDL, REG0..REG3, CRYPT_H/L
    uint32_t saved_baud = huart->Init.BaudRate;
    uint8_t  rc, n_rx = 0, aux_dropped = 0;

    DBG_PRINT("\r\n--- E220 config read ---\r\n");
    DBG_PRINT("AUX before mode switch: %s\r\n",
              HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_SET
                  ? "HIGH (idle)" : "LOW (busy)");

    // 1. Into configuration mode. The mode change takes effect a few ms after
    //    AUX returns high; the datasheet asks for >2 ms, 50 ms is free here.
    HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_SET);
    HAL_Delay(200);

    if (!AuxWaitHigh(1000)) {
        DBG_PRINT("AUX stuck LOW entering config mode - module busy or unpowered\r\n");
    }

    // 2. Probe grid. Which M1/M0 combination is "configuration" is exactly
    //    what is in doubt on this board, so do not assume the datasheet table
    //    - send the register-read command in all four and let the module say.
    //    Both the documented register-read and the legacy E32-style read-all
    //    are tried, at the fixed config baud and at the flight baud, since a
    //    module whose stored baud is neither would look identically dead.
    static const uint8_t cmd_reg[3] = { 0xC1, 0x00, 0x08 };  // read regs 0x00..0x07
    static const uint8_t cmd_all[3] = { 0xC1, 0xC1, 0xC1 };  // legacy read-all
    static const struct { uint8_t m1, m0; } mode_grid[4] = {
        { E220_CFG_M1, E220_CFG_M0 },     // the one that answers on this board
        { 1, 1 }, { 0, 1 }, { 0, 0 }
    };

    rc = 2;
    for (uint8_t m = 0; m < 4 && rc != 0 && n_rx == 0; m++) {
        HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, mode_grid[m].m1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, mode_grid[m].m0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_Delay(200);
        AuxWaitHigh(1000);

        const struct { uint32_t baud; const uint8_t *cmd; const char *name; } probes[3] = {
            { E220_CFG_BAUD, cmd_reg, "9600 C1 00 08" },
            { E220_CFG_BAUD, cmd_all, "9600 C1 C1 C1" },
            { saved_baud,    cmd_reg, "flgt C1 00 08" },
        };

        for (uint8_t p = 0; p < 3; p++) {
            SetBaud(huart, probes[p].baud);
            rc = TryRead(huart, probes[p].cmd, rsp, sizeof(rsp), &n_rx, &aux_dropped);
            DBG_PRINT("  M1=%u M0=%u %s : rc=%u rx=%u aux=%u\r\n",
                      mode_grid[m].m1, mode_grid[m].m0, probes[p].name,
                      rc, n_rx, aux_dropped);
            if (rc == 0 || n_rx > 0) break;
        }
    }

    if (n_rx > 0) {
        DBG_PRINT("rx bytes:");
        for (uint8_t i = 0; i < n_rx && i < 16; i++) DBG_PRINT(" %02X", rsp[i]);
        DBG_PRINT("\r\n");
    }

    if (rc == 0) {
        // rsp[0..2] is the C1/address/length echo, so register 0xNN lands at
        // rsp[3 + NN]. Getting this indexing wrong by one byte - which is what
        // NETID does to anyone assuming an E220 layout - silently reports a
        // different register for every field.
        const uint8_t *reg = &rsp[3];
        uint8_t r0 = reg[E22_ADDR_REG0], r1 = reg[E22_ADDR_REG1];
        uint8_t r2 = reg[E22_ADDR_REG2], r3 = reg[E22_ADDR_REG3];

        DBG_PRINT("raw: %02X %02X %02X | %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                  rsp[0], rsp[1], rsp[2], rsp[3], rsp[4], rsp[5], rsp[6],
                  rsp[7], rsp[8], rsp[9], rsp[10]);
        DBG_PRINT("addr    : 0x%02X%02X   netid 0x%02X\r\n",
                  reg[E22_ADDR_ADDH], reg[E22_ADDR_ADDL], reg[E22_ADDR_NETID]);
        DBG_PRINT("uart    : %lu baud %s   (firmware sends at %lu)\r\n",
                  (unsigned long)baud_tbl[(r0 >> 5) & 7], par_tbl[(r0 >> 3) & 3],
                  (unsigned long)saved_baud);
        DBG_PRINT("air rate: %lu bps\r\n", (unsigned long)air_tbl[r0 & 7]);
        DBG_PRINT("power   : %u dBm   sub-packet %u B   rssi-noise %u\r\n",
                  pwr_tbl[r1 & 3], pkt_tbl[(r1 >> 6) & 3], (r1 >> 5) & 1);
        DBG_PRINT("channel : %u  (%u.125 MHz)\r\n", r2, 850u + r2);
        DBG_PRINT("mode    : %s\r\n",
                  ((r3 >> 6) & 1) ? "FIXED (first 3 TX bytes = ADDH/ADDL/CH)"
                                  : "TRANSPARENT (all bytes are payload)");
        DBG_PRINT("reg3    : 0x%02X  rssi-byte %u repeater %u LBT %u\r\n",
                  r3, (r3 >> 7) & 1, (r3 >> 5) & 1, (r3 >> 4) & 1);
    } else if (rc == 2) {
        // Nothing came back at all. AUX is the discriminator: if it dropped,
        // the module heard the command and its answer did not reach PA10; if
        // it did not, the command never got in or the module is not in config
        // mode (or not powered).
        DBG_PRINT("NO REPLY. AUX %s during the exchange => %s\r\n",
                  aux_dropped ? "DID drop" : "did NOT drop",
                  aux_dropped ? "module answered, RX path to PA10 is the fault"
                              : "module never processed the command");
    } else {
        DBG_PRINT("UNEXPECTED REPLY (rc=%u) - answered, but not a C1 register frame\r\n", rc);
    }

    // 3. Back to transmission mode and the flight baud, whatever happened.
    HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    SetBaud(huart, saved_baud);
    AuxWaitHigh(1000);

    DBG_PRINT("--- end E220 config read ---\r\n\r\n");
    return rc;
}
