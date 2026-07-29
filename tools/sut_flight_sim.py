#!/usr/bin/env python3
"""
Stand-in for the TEKNOFEST UKB test device's SUT (Sentetik Ucus Testi) side.

Sends "SUT Baslat", then streams Tablo 4 synthetic sensor packets at 10 Hz
while decoding the Tablo 6 status packets the board sends back, reporting each
status bit the moment it lights up.

The default profile is shaped to walk flight_sm.c through every state, so all
eight Tablo 5 bits should fire in order. See EK-7 2.1.3 / 2.1.4.

Wire format notes, all matching tools/rs232_test_device.py:
  * command   = AA 22 CC 0D 0A   (header-inclusive checksum, not Tablo 1's 0x8E)
  * data pkt  = 36 bytes: AB + 8 big-endian FLOAT32 + checksum + 0D 0A
  * checksum  = sum of header through last data byte, mod 256
  * status    = 6 bytes: AA + Data1 + Data2 + checksum + 0D 0A,
                Data1 = bits 0-7, Data2 = bits 8-15

Usage:
    ./sut_flight_sim.py                 # run the full synthetic flight
    ./sut_flight_sim.py --port /dev/ttyUSB0
    ./sut_flight_sim.py --dry-run       # print the profile, touch no hardware
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")

HEADER_CMD, HEADER_DATA = 0xAA, 0xAB
FOOTER1, FOOTER2 = 0x0D, 0x0A
CMD_SUT_START, CMD_STOP = 0x22, 0x24
SENSOR_PACKET_LEN, STATUS_PACKET_LEN = 36, 6

# Tablo 5 / EK-15 Tablo 3
STATUS_BITS = (
    ("KTE", "Kalkis Tespit Edildi      (liftoff detected)"),
    ("YSD", "Yanma Suresi Doldu        (burn time elapsed)"),
    ("IEA", "Min. Irtifa Esigi Asildi  (min altitude passed)"),
    ("GAA", "Govde Acisi Algilandi     (body angle detected)"),
    ("ATE", "Alcalma Tespit Edildi     (descent detected)"),
    ("SPE", "Suruklenme Parasutu Emri  (DROGUE FIRED)"),
    ("BIT", "Belirlenen Irtifa Tespit  (below main altitude)"),
    ("APE", "Ana Parasut Emri          (MAIN FIRED)"),
)

RATE_HZ = 10.0
DT = 1.0 / RATE_HZ


def checksum(b: bytes) -> int:
    return sum(b) & 0xFF


def build_command(cmd: int) -> bytes:
    body = bytes((HEADER_CMD, cmd))
    return body + bytes((checksum(body), FOOTER1, FOOTER2))


def pressure_hpa(alt_m: float) -> float:
    """ISA, purely cosmetic here - the board drives its algorithm from the
    altitude field. Sent anyway so the ground software's graph looks sane."""
    return 1013.25 * (1.0 - alt_m / 44330.0) ** 5.255


def build_data_packet(alt, ax, ay, az, angx, angy, angz) -> bytes:
    pkt = bytearray([HEADER_DATA])
    for v in (alt, pressure_hpa(alt), ax, ay, az, angx, angy, angz):
        pkt += struct.pack(">f", round(v, 2))   # MSB first, 2 dp per 2.1.1
    pkt.append(checksum(pkt))                   # header..last data byte
    pkt += bytes((FOOTER1, FOOTER2))
    assert len(pkt) == SENSOR_PACKET_LEN, len(pkt)
    return bytes(pkt)


def lerp(a, b, t):
    return a + (b - a) * t


def build_profile():
    """(label, alt, ax, ay, az, angx, angy, angz) at 10 Hz.

    Tuned against flight_sm.c's thresholds, where `lon` below is the value the
    board treats as LONGITUDINAL after its own SUT remap:
      PAD  -> BOOST : lon > 24.5 m/s^2
      BOOST-> COAST : lon < 2.0 m/s^2
      COAST-> MINALT: alt > 500
      MINALT->APOGEE: alt < peak-1.5  AND one of (net_g < 3.92, pitch > 45),
                      sustained for DESCENT_CONFIRM (5) samples
      DESCENT->MAIN : alt < 800
      MAIN -> LANDED: alt steady within 0.5 m for 3 s

    Three things here follow the REAL test device rather than EK-7, because
    standing in for that device is this tool's whole job. All three were
    measured on the wire; each names the app.c flag that encodes it.

    1. ACCELERATIONS ARE m/s^2, exactly as Tablo 2/4 say (UKB_SUT_ACCEL_IN_G
       is 0). This file used to send g, on the strength of a single packet
       whose acceleration magnitude was 1.10 - read as "1 g". It was ~0.11 g
       in m/s^2, sampled just after apogee, where a rocket in ballistic
       descent genuinely is near-weightless. Never infer units from one sample
       taken at an unknown flight phase.

    2. THE LONGITUDINAL AXIS IS Z, not the X of EK-7 section 1.2
       (UKB_SUT_SWAP_XZ is 1). The device puts thrust and drag on Z - +247
       m/s^2 at boost, negative through the coast - and keeps X inside
       +/-3 m/s^2 for an entire flight.

    3. ANGLES ARE MEASURED FROM VERTICAL: 0 = nose up, 90 = horizontal,
       180 = nose down. This file used to start at 90 on the pad and fall to
       15 under parachute, i.e. inverted on both ends - asserting the rocket
       lay horizontal on the pad and pointed nose-up while descending. That
       latches GAA on the first sample and inverts the GAA/ATE order the
       judges read, even where deployment itself is unaffected.

    Combined, 1 and 2 meant the board was fed a 6.1 m/s^2 nudge on a lateral
    axis and correctly concluded that nothing ever launched: every packet
    valid, no checksum errors, not one status bit set.
    """
    S = []

    def add(n, label, fn):
        for i in range(n):
            S.append((label,) + fn(i / max(n - 1, 1)))

    # Tuples below are (alt, lon, lat1, lat2, angx, angy, angz) with `lon` the
    # longitudinal acceleration in m/s^2; the axis placement happens at the end.
    #
    # Pad: standing on its tail, so the longitudinal axis reads +1 g, nose up.
    add(15, "PAD",    lambda t: (0.0, 9.81, 0.0, 0.0, 0.0, 0.0, 0.0))
    # Boost: ~6 g, well clear of the 24.5 m/s^2 liftoff gate.
    add(20, "BOOST",  lambda t: (lerp(0, 400, t * t), 60.0, 0.5, 0.5, 0.0, 2.0, 0.0))
    # Coast: motor out, so drag DECELERATES and the longitudinal value goes
    # negative - the real device does this too, and it trips the `< 2.0` gate
    # the moment the motor quits rather than waiting for drag to decay.
    add(50, "COAST",  lambda t: (lerp(400, 2000, t), lerp(-18.0, -0.6, t), 0.3, 0.3,
                                 0.0, lerp(5.0, 35.0, t), 0.0))
    # Apogee dwell: flat top so the peak is well established before descent,
    # and the pitch-over crosses 45 deg here, so GAA latches before ATE.
    add(5,  "APOGEE", lambda t: (2000.0, -0.5, 0.2, 0.3, 0.0, lerp(40.0, 50.0, t), 0.0))
    # Descent: falling, near weightless, pitched well past 45 deg.
    # Passes 800 m on the way down, which is the main-chute gate.
    add(80, "DESCENT", lambda t: (lerp(2000, 5, t), 0.5, 0.2, 0.3, 0.0,
                                  lerp(60.0, 100.0, t), 0.0))
    # On the ground: altitude pinned so the 3 s stability timer can expire.
    add(50, "LANDED", lambda t: (5.0, 9.81, 0.2, 0.3, 0.0, 90.0, 0.0))

    # Place the longitudinal value on Z, where the real device puts it, and
    # leave the two small lateral values on X and Y.
    return [(label, alt, lat2, lat1, lon, angx, angy, angz)
            for label, alt, lon, lat1, lat2, angx, angy, angz in S]


def describe(bits, prev):
    out = []
    for i, (abbr, desc) in enumerate(STATUS_BITS):
        m = 1 << i
        if (bits & m) and not (prev & m):
            out.append(f"bit{i} {abbr}  {desc}")
    return out


class StatusDecoder:
    def __init__(self):
        self.buf = bytearray()
        self.bad_cks = 0
        self.count = 0

    def feed(self, data):
        self.buf += data
        while True:
            i = self.buf.find(bytes((HEADER_CMD,)))
            if i < 0:
                self.buf.clear()
                return
            if len(self.buf) - i < STATUS_PACKET_LEN:
                del self.buf[:i]
                return
            f = bytes(self.buf[i:i + STATUS_PACKET_LEN])
            if f[4] != FOOTER1 or f[5] != FOOTER2:
                del self.buf[:i + 1]
                continue
            del self.buf[:i + STATUS_PACKET_LEN]
            if f[3] != checksum(f[:3]):
                self.bad_cks += 1
                continue
            self.count += 1
            yield (f[2] << 8) | f[1]


def run(port, baud, dry_run):
    profile = build_profile()
    if dry_run:
        print(f"{len(profile)} samples, {len(profile) * DT:.1f} s at {RATE_HZ:g} Hz")
        last = None
        for label, alt, ax, ay, az, angx, angy, angz in profile:
            if label != last:
                # az is the longitudinal axis (see build_profile note 2).
                print(f"  {label:8} alt={alt:7.1f}  az={az:7.2f}  angY={angy:5.1f}")
                last = label
        return 0

    with serial.Serial(port, baud, timeout=0) as ser:
        ser.reset_input_buffer()

        frame = build_command(CMD_SUT_START)
        ser.write(frame); ser.flush()
        print(f"sent SUT START: {frame.hex(' ')}")
        print("waiting 1.3 s for the board to arm (spec 2.1.4c allows 1 s)\n")
        time.sleep(1.3)
        ser.reset_input_buffer()

        dec = StatusDecoder()
        bits = 0
        fired = {}
        t0 = time.monotonic()
        next_tx = t0
        last_label = None

        for idx, (label, alt, ax, ay, az, angx, angy, angz) in enumerate(profile):
            next_tx += DT
            ser.write(build_data_packet(alt, ax, ay, az, angx, angy, angz))
            ser.flush()

            if label != last_label:
                print(f"  [{time.monotonic() - t0:5.1f}s] -- profile phase: {label} "
                      f"(alt {alt:.0f} m)")
                last_label = label

            # Drain whatever came back during this slot.
            while time.monotonic() < next_tx:
                chunk = ser.read(256)
                if chunk:
                    for new in dec.feed(chunk):
                        for line in describe(new, bits):
                            el = time.monotonic() - t0
                            print(f"  [{el:5.1f}s]  ** {line}")
                            fired[line.split()[1]] = el
                        bits = new
                else:
                    time.sleep(0.002)

        stop = build_command(CMD_STOP)
        ser.write(stop); ser.flush()
        print(f"\nsent STOP: {stop.hex(' ')}")
        time.sleep(0.3)

    dur = time.monotonic() - t0
    print("\n--- summary ---")
    print(f"data packets sent : {len(profile)}  in {dur:.1f} s "
          f"({len(profile) / dur:.2f} Hz)")
    print(f"status packets in : {dec.count}  ({dec.count / dur:.2f} Hz, spec wants 10)")
    print(f"status checksum errors: {dec.bad_cks}")
    print(f"final status word : 0x{bits:04X}")
    print("\nTablo 5 events:")
    missing = 0
    for i, (abbr, desc) in enumerate(STATUS_BITS):
        if bits & (1 << i):
            print(f"  [x] bit{i} {abbr}  {desc}   @ {fired.get(abbr, float('nan')):.1f}s")
        else:
            print(f"  [ ] bit{i} {abbr}  {desc}   NOT SET")
            missing += 1
    return 1 if missing else 0


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--dry-run", action="store_true")
    a = p.parse_args()
    sys.exit(run(a.port, a.baud, a.dry_run))


if __name__ == "__main__":
    main()
