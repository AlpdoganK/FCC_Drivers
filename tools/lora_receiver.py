#!/usr/bin/env python3
"""
Receives and decodes TelemetryPacket frames from an EBYTE E220-900T30S LoRa
module connected directly to this computer (USB-UART adapter).

Wire format matches drivers/Inc/e220.h's TelemetryPacket, as transmitted by
LoRa_TransmitTelemetry_Blocking (packed, little-endian):

    uint8   Lora_ADDRH   (0x7B)  -- only present on the wire if the receiving
    uint8   Lora_ADDRL   (0xD3)  -- E220 is in transparent mode; a receiver in
    uint8   Lora_CH      (0x2B)  -- fixed-transmission mode strips these 3
    uint8   header       (0xAB)    <- frame search key is anchored here
    uint32  timestamp_ms
    uint8   flight_state
    float   ax, ay, az
    float   gy
    float   pitch
    float   baro_alt_raw
    float   baro_alt
    float   gps_lat
    float   gps_lon
    uint16  crc          (CRC-16/CCITT-FALSE over timestamp..gps_lon)
    uint8   footer       (0x0A)

Since whether the 3 address/channel bytes show up on this computer's UART
depends on how the local E220 is configured (transparent vs. fixed mode),
this script searches for the header/footer pair directly (45-byte body,
ignoring whatever precedes it) instead of assuming a fixed frame length.

Also opens a live matplotlib plot of baro_alt (fused) and baro_alt_raw vs.
time as frames come in.

Usage:
    python3 lora_receiver.py [port] [baud]

Defaults: port=/dev/ttyUSB0, baud=115200 (matches USART1 config in usart.c)

Requires: pyserial, matplotlib
"""

import sys
import struct
from collections import deque

import serial
import matplotlib.pyplot as plt

HISTORY_LEN = 500  # rolling window of points shown on the plot

# Body = header..footer, i.e. TelemetryPacket minus the 3 leading
# Lora_ADDRH/ADDRL/CH routing bytes (which may or may not reach this UART).
BODY_FORMAT = "<BIB9fHB"
BODY_SIZE = struct.calcsize(BODY_FORMAT)
assert BODY_SIZE == 45

HEADER_BYTE = 0xAB
FOOTER_BYTE = 0x0A

FLIGHT_STATES = [
    "PAD", "BOOST", "COAST", "MIN_ALTITUDE_REACHED",
    "APOGEE", "DESCENT", "MAIN", "LANDED",
]


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def decode_packet(body: bytes):
    fields = struct.unpack(BODY_FORMAT, body)
    (header, timestamp, flight_state,
     ax, ay, az, gy, pitch, baro_alt_raw, baro_alt, gps_lat, gps_lon,
     crc, footer) = fields

    # CRC covers timestamp..gps_lon: bytes[1:42] of the body (i.e. everything
    # between the header byte and the crc field).
    calc_crc = crc16_ccitt(body[1:42])

    return {
        "timestamp": timestamp,
        "flight_state": flight_state,
        "ax": ax, "ay": ay, "az": az,
        "gy": gy, "pitch": pitch,
        "baro_alt_raw": baro_alt_raw, "baro_alt": baro_alt,
        "gps_lat": gps_lat, "gps_lon": gps_lon,
        "crc_ok": calc_crc == crc and header == HEADER_BYTE and footer == FOOTER_BYTE,
    }


def find_next_frame(buf: bytearray):
    """Search buf for HEADER_BYTE with FOOTER_BYTE exactly BODY_SIZE-1 bytes later.
    Returns (body_bytes, remaining_buf) or (None, buf) if not enough data yet."""
    while len(buf) >= BODY_SIZE:
        idx = buf.find(HEADER_BYTE)
        if idx < 0:
            buf.clear()
            break
        if len(buf) - idx < BODY_SIZE:
            break  # need more bytes to confirm this candidate
        if buf[idx + BODY_SIZE - 1] == FOOTER_BYTE:
            body = bytes(buf[idx: idx + BODY_SIZE])
            del buf[: idx + BODY_SIZE]
            return body, buf
        # False positive (0xAB appeared in payload data) - resync past it
        del buf[: idx + 1]
    return None, buf


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print(f"Opening {port} @ {baud} baud...")
    ser = serial.Serial(port, baud, timeout=0.1)

    buf = bytearray()
    count = 0
    t0 = None
    t_hist = deque(maxlen=HISTORY_LEN)
    alt_hist = deque(maxlen=HISTORY_LEN)
    alt_raw_hist = deque(maxlen=HISTORY_LEN)

    plt.ion()
    fig, ax = plt.subplots()
    line_alt, = ax.plot([], [], label="baro_alt (fused)")
    line_raw, = ax.plot([], [], label="baro_alt_raw", alpha=0.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Live Altitude")
    ax.legend(loc="upper left")
    ax.grid(True)
    fig.show()

    print("Waiting for telemetry frames (Ctrl+C to quit)...\n")
    try:
        while True:
            chunk = ser.read(256)
            if chunk:
                buf.extend(chunk)

            got_new = False
            while True:
                frame, buf = find_next_frame(buf)
                if frame is None:
                    break

                count += 1
                pkt = decode_packet(frame)
                state_name = (
                    FLIGHT_STATES[pkt["flight_state"]]
                    if pkt["flight_state"] < len(FLIGHT_STATES)
                    else f"UNKNOWN({pkt['flight_state']})"
                )
                status = "OK" if pkt["crc_ok"] else "CRC FAIL"

                print(
                    f"[{count:5d}] t={pkt['timestamp']:>8d}ms  "
                    f"state={state_name:<22s}  "
                    f"acc=({pkt['ax']:+6.2f},{pkt['ay']:+6.2f},{pkt['az']:+6.2f})  "
                    f"gy={pkt['gy']:+7.2f}  pitch={pkt['pitch']:+6.1f}  "
                    f"alt={pkt['baro_alt']:7.1f}m (raw={pkt['baro_alt_raw']:7.1f})  "
                    f"gps=({pkt['gps_lat']:.5f},{pkt['gps_lon']:.5f})  [{status}]"
                )

                if t0 is None:
                    t0 = pkt["timestamp"]
                t_hist.append((pkt["timestamp"] - t0) / 1000.0)
                alt_hist.append(pkt["baro_alt"])
                alt_raw_hist.append(pkt["baro_alt_raw"])
                got_new = True

            if got_new:
                line_alt.set_data(t_hist, alt_hist)
                line_raw.set_data(t_hist, alt_raw_hist)
                ax.relim()
                ax.autoscale_view()

            plt.pause(0.001)  # keeps the plot window responsive between frames
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
