#!/usr/bin/env python3
"""
Stand-in for the TEKNOFEST UKB ground test device (EK-7 / EK-15).

Sends SIT/SUT/Stop commands over RS232 and decodes what the flight computer
sends back, so the link can be exercised without the official Windows test
software. Everything here matches what the real software was observed to do
on the bench, which is NOT always what EK-7's tables say:

  * checksum  = sum of header through last data byte, mod 256
                (Tablo 1 and Tablo 6's worked examples are both wrong)
  * commands  = AA <cmd> <(0xAA+cmd)&0xFF> 0D 0A
                (Tablo 1 lists 0x8C/0x8E/0x90; the software sends CA/CC/CE)
  * FLOAT32   = big-endian, MSB first
                (section 4.2's union example implies little-endian; it isn't)
  * sensor packet = 36 bytes, not the 34 implied by Tablo 3's numbering

Usage:
    ./rs232_test_device.py --list
    ./rs232_test_device.py listen              # just decode whatever arrives
    ./rs232_test_device.py sit --duration 15   # start SIT, decode, then stop
    ./rs232_test_device.py stop
"""

import argparse
import struct
import sys
import time
from collections import Counter

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")

HEADER_CMD = 0xAA
HEADER_DATA = 0xAB
FOOTER1, FOOTER2 = 0x0D, 0x0A

CMD_SIT_START, CMD_SUT_START, CMD_STOP = 0x20, 0x22, 0x24

CMD_FRAME_LEN = 5
STATUS_PACKET_LEN = 6
SENSOR_PACKET_LEN = 36

FIELDS = ("irtifa_m", "basinc_hPa", "ivme_x", "ivme_y", "ivme_z",
          "aci_x", "aci_y", "aci_z")

# EK-15 Tablo 3 abbreviations for the Tablo 5 status bits.
STATUS_BITS = ("KTE liftoff", "YSD burn-time", "IEA min-alt", "GAA body-angle",
               "ATE descent", "SPE drogue", "BIT alt-threshold", "APE main")

# EK-15 Sekil 1 acceptance ranges; values outside count as "Hatali".
LIMITS = {"irtifa_m": (0.0, 10000.0), "basinc_hPa": (600.0, 1100.0),
          "ivme_x": (-100.0, 100.0), "ivme_y": (-100.0, 100.0),
          "ivme_z": (-100.0, 100.0), "aci_x": (-180.0, 180.0),
          "aci_y": (-180.0, 180.0), "aci_z": (-180.0, 180.0)}


def checksum(payload: bytes) -> int:
    """Header through last data byte, mod 256."""
    return sum(payload) & 0xFF


def build_command(cmd: int) -> bytes:
    body = bytes((HEADER_CMD, cmd))
    return body + bytes((checksum(body), FOOTER1, FOOTER2))


class Decoder:
    """Byte-wise frame assembler, mirroring the firmware's own approach so it
    is immune to however the OS happens to chunk the stream."""

    def __init__(self):
        self.buf = bytearray()
        self.expect = 0
        self.stats = Counter()
        self.junk = bytearray()

    def feed(self, data: bytes):
        for b in data:
            if not self.buf:
                if b == HEADER_CMD:
                    self.expect = STATUS_PACKET_LEN
                elif b == HEADER_DATA:
                    self.expect = SENSOR_PACKET_LEN
                else:
                    self.junk.append(b)
                    self.stats["junk_bytes"] += 1
                    continue
            self.buf.append(b)
            if len(self.buf) == self.expect:
                frame = bytes(self.buf)
                self.buf.clear()
                yield from self._decode(frame)

    def _decode(self, f: bytes):
        if f[-2] != FOOTER1 or f[-1] != FOOTER2:
            self.stats["bad_footer"] += 1
            return
        want = checksum(f[:-3])
        if f[-3] != want:
            self.stats["bad_checksum"] += 1
            print(f"  ! checksum mismatch: expected 0x{want:02X}, "
                  f"got 0x{f[-3]:02X}   raw={f.hex(' ')}")
            return
        if f[0] == HEADER_DATA:
            self.stats["sensor"] += 1
            yield ("sensor", dict(zip(FIELDS, struct.unpack(">8f", f[1:33]))))
        else:
            self.stats["status"] += 1
            yield ("status", (f[2] << 8) | f[1])


def describe_status(bits: int) -> str:
    active = [n for i, n in enumerate(STATUS_BITS) if bits & (1 << i)]
    return ", ".join(active) if active else "no events"


def out_of_range(sample: dict):
    return [k for k, v in sample.items()
            if not (LIMITS[k][0] <= v <= LIMITS[k][1])]


def drain(ser, quiet_for: float = 0.3, cap: float = 5.0) -> int:
    """Discard anything already queued until the line has been quiet.

    reset_input_buffer() alone is NOT enough here: a backlog builds up in the
    driver and reads back at many times the wire rate, which makes the first
    seconds of any capture look like an impossible flood. Read and throw away
    until nothing arrives for `quiet_for` seconds.
    """
    start = time.monotonic()
    ser.reset_input_buffer()
    dropped = 0
    last_data = time.monotonic()
    while time.monotonic() - start < cap:
        chunk = ser.read(4096)
        if chunk:
            dropped += len(chunk)
            last_data = time.monotonic()
        elif time.monotonic() - last_data >= quiet_for:
            break
    return dropped


def run(port: str, baud: int, command: str, duration: float):
    with serial.Serial(port, baud, timeout=0.1) as ser:
        stale = drain(ser)
        if stale:
            print(f"(drained {stale} stale bytes before measuring)")

        if command in ("sit", "sut", "stop"):
            code = {"sit": CMD_SIT_START, "sut": CMD_SUT_START,
                    "stop": CMD_STOP}[command]
            frame = build_command(code)
            ser.write(frame)
            ser.flush()
            print(f"sent {command.upper()}: {frame.hex(' ')}")
            if command != "stop":
                print("(spec allows 1 s before data starts)\n")

        dec = Decoder()
        first = last = None
        started = time.monotonic()
        bad_range = Counter()

        try:
            while time.monotonic() - started < duration:
                chunk = ser.read(256)
                if not chunk:
                    continue
                for kind, payload in dec.feed(chunk):
                    now = time.monotonic()
                    if kind == "sensor":
                        if first is None:
                            first = now
                        last = now
                        for f in out_of_range(payload):
                            bad_range[f] += 1
                        if dec.stats["sensor"] % 10 == 1:
                            print("  " + "  ".join(
                                f"{k}={v:.2f}" for k, v in payload.items()))
                    else:
                        print(f"  status 0x{payload:04X} -> "
                              f"{describe_status(payload)}")
        except KeyboardInterrupt:
            print("\ninterrupted")

        if command == "sit":
            stop = build_command(CMD_STOP)
            ser.write(stop)
            ser.flush()
            print(f"\nsent STOP: {stop.hex(' ')}")

    print("\n--- summary ---")
    n = dec.stats["sensor"]
    print(f"sensor packets : {n}")
    if n > 1 and first is not None and last > first:
        print(f"rate           : {(n - 1) / (last - first):.2f} Hz  "
              f"(spec requires 10 Hz)")
    print(f"status packets : {dec.stats['status']}")
    print(f"checksum errors: {dec.stats['bad_checksum']}")
    print(f"footer errors  : {dec.stats['bad_footer']}")
    print(f"junk bytes     : {dec.stats['junk_bytes']}")
    if dec.junk:
        print(f"  first junk   : {bytes(dec.junk[:16]).hex(' ')}")
    if bad_range:
        print("out-of-range (would count as 'Hatali'):")
        for k, c in bad_range.items():
            print(f"  {k}: {c}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", nargs="?", default="listen",
                   choices=("listen", "sit", "sut", "stop"))
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument("--list", action="store_true", help="list serial ports")
    a = p.parse_args()

    if a.list:
        for pt in list_ports.comports():
            print(f"{pt.device:15} {pt.description}")
        return

    run(a.port, a.baud, a.command, a.duration)


if __name__ == "__main__":
    main()
