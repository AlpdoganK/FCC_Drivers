/*
 * E220-900T30S configuration-register diagnostic (BENCH)
 * Author: Alpdogan
 *
 * The flight driver (drivers/Src/e220.c) only ever pushes bytes at the module
 * in transmission mode - it never asks the module what it is actually
 * configured to do. That leaves a whole class of faults invisible: the board
 * reports a clean AUX handshake ("LoRa TX OK") while the module is in
 * configuration mode, on a different channel, at a different UART baud, or at
 * minimum output power, and nothing radiates.
 *
 * This reads the module's registers back over USART1 and prints them, which
 * distinguishes those cases in one shot. Bench tool - not part of the flight
 * path. See E220_CONFIG_DIAG in app.c.
 */

#ifndef E220_DIAG_H
#define E220_DIAG_H

#include "stm32f4xx_hal.h"

// Enters configuration mode (M0=M1=HIGH), reads registers 0x00-0x07 and prints
// them raw + decoded on the debug console, then restores transmission mode and
// the caller's UART baud rate. Returns 0 on a valid reply, non-zero otherwise.
uint8_t E220_Diag_ReadConfig(UART_HandleTypeDef *huart);

// Drives all four M1/M0 combinations in turn, sends a 48-byte dummy packet in
// each, and prints how long the module held AUX low. Air time is the tell: the
// transmission mode radiates (hundreds of ms for 48 B at the default air rate)
// while configuration/deep-sleep does not (a few ms of buffering). Answers two
// questions at once - which combination this board's module treats as
// "transmit", and whether the mode pins reach it at all (identical timings in
// all four rows mean they do not).
//
// This KEYS THE RADIO in at least one row: antenna must be fitted.
void E220_Diag_ModeSweep(UART_HandleTypeDef *huart);

// Sends `count` packets on App_Run's terms (transmission mode, blocking
// transmit) at the given cadence and prints the AUX-low time of each. At a
// cadence longer than the air time this just shows the module keeping up; at a
// shorter one the AUX-low times grow as the buffer fills, which is the only
// way from this side to tell one air rate from another once the sub-packet
// size is big enough to swallow a whole packet.
void E220_Diag_BurstTest(UART_HandleTypeDef *huart, uint32_t cadence_ms, uint8_t count);

// Sends 48, 96, 150 and 200 bytes as single writes, each after an identical
// 2 s idle, and reports the AUX-low time of each. Distinguishes "the module
// needs a minimum amount of data before it will key" (padding fixes it) from
// "it needs a sustained stream" (padding will not).
void E220_Diag_TriggerTest(UART_HandleTypeDef *huart);

// Sends one isolated 48-byte packet at each of the eight standard baud rates
// and reports AUX-low for each. Makes no assumption about REG0's baud field -
// the rate at which the module actually keys the PA is its real UART baud.
void E220_Diag_BaudSweep(UART_HandleTypeDef *huart);

// Writes `len` raw register bytes starting at `start_addr` and saves them to
// the module's flash (C0 command). Returns 0 if the module acknowledged.
//
// Raw addresses on purpose. Per the E22-900T30S manual the map is
// 00 ADDH, 01 ADDL, 02 NETID, 03 REG0, 04 REG1, 05 REG2 (channel), 06 REG3 -
// the NETID byte at 0x02 shifts every register one address up from an
// E220-style layout, and a helper naming its arguments "reg0, reg1" invites
// writing the wrong ones. Air rate, channel and address are link parameters:
// THE RECEIVER MUST BE MOVED TO MATCH or the link goes silent.
uint8_t E220_Diag_WriteConfig(UART_HandleTypeDef *huart, uint8_t start_addr,
                              const uint8_t *vals, uint8_t len);

#endif /* E220_DIAG_H */
