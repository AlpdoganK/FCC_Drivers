#!/usr/bin/env python3
"""
Live 3D orientation view for the flight computer.

Reads attitude off the board and draws the rocket in a world frame, so the
IMU can be checked by picking the board up and turning it. Two sources:

  console  USART2 @ 9600 - parses the IMU_BENCH_MODE line printed by app.c.
           Needs DEBUG_PRINTS_ENABLED = 1 and GPS_ENABLED = 0 (they share the
           UART). Updates at 4 Hz and carries no yaw - see below.

  sit      USART6 @ 115200 - starts a SIT run and decodes the 36-byte Tablo 3
           sensor packets. 10 Hz and carries all three angles, but needs the
           RS232 level converter and fires no pyros (SIT is sensor-only; SUT
           is the one that fires them, and this script never sends SUT).

Angle conventions, which are NOT the usual aerospace ones:

  pitch  is measured FROM VERTICAL: 0 = nose up, 90 = horizontal, 180 = nose
         down. app.c adds the +90 offset that puts it on this scale.
  roll   is atan2(ay, az), so 0 = board flat with +Z up.
  yaw    is dead-reckoned from the gyro with no magnetometer, so it drifts and
         means nothing in absolute terms. The console source does not report
         it at all and this script draws yaw = 0 there; that is expected, not
         a parse failure.

Roll is UNDEFINED when the nose is near vertical - atan2(ay, az) is then
taking the arctangent of two near-zero noise values. The readout greys out and
warns instead of pretending the number means something.

Usage:
    ./orientation_view.py --list
    ./orientation_view.py                        # console source, autodetect
    ./orientation_view.py --port /dev/ttyUSB0
    ./orientation_view.py --source sit --port /dev/ttyUSB1
    ./orientation_view.py --demo                 # no hardware, synthetic sweep
"""

import argparse
import math
import re
import struct
import sys
import threading
import time

try:
    import numpy as np
except ImportError:
    sys.exit("numpy is required:  pip install numpy")

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
except ImportError:
    sys.exit("matplotlib is required:  pip install matplotlib")

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None  # only fatal if a serial source is actually requested

# ---------------------------------------------------------------------------
# Protocol / parsing
# ---------------------------------------------------------------------------

# IMU_BENCH_MODE line:
#   ax= +0.05 ay= -0.02 az= +9.79 |a|=0.998g pitch= +90.1 roll=  -0.1
# %+6.2f right-aligns inside the field, hence the \s* after every '='.
BENCH_RE = re.compile(
    r"ax=\s*(?P<ax>[-+]?[\d.]+)\s+"
    r"ay=\s*(?P<ay>[-+]?[\d.]+)\s+"
    r"az=\s*(?P<az>[-+]?[\d.]+)\s+"
    r"\|a\|=\s*(?P<g>[-+]?[\d.]+)g\s+"
    r"pitch=\s*(?P<pitch>[-+]?[\d.]+)\s+"
    r"roll=\s*(?P<roll>[-+]?[\d.]+)"
    r"(?:\s+yaw=\s*(?P<yaw>[-+]?[\d.]+))?"
)

# The IMU_BENCH_MODE = 0 status block line, so the view also works against a
# normal build (at 1 Hz).  IMU: ax=.. ay=.. az=.. gy=.. pitch=.. roll=..
STATUS_RE = re.compile(
    r"IMU:\s+ax=(?P<ax>[-+]?[\d.]+)\s+"
    r"ay=(?P<ay>[-+]?[\d.]+)\s+"
    r"az=(?P<az>[-+]?[\d.]+)\s+"
    r"gy=(?P<gy>[-+]?[\d.]+)\s+"
    r"pitch=(?P<pitch>[-+]?[\d.]+)\s+"
    r"roll=(?P<roll>[-+]?[\d.]+)"
)

HEADER_CMD = 0xAA
HEADER_DATA = 0xAB
FOOTER = b"\x0d\x0a"
SENSOR_PACKET_LEN = 36
CMD_SIT_START = 0x20
CMD_STOP = 0x24

G0 = 9.80665


def command_frame(cmd):
    """5-byte command frame. The real ground software sends (0xAA+cmd)&0xFF as
    the checksum, not the value Tablo 1 lists - see rs232_test_device.py."""
    return bytes([HEADER_CMD, cmd, (HEADER_CMD + cmd) & 0xFF]) + FOOTER


# ---------------------------------------------------------------------------
# Attitude -> rotation matrix
# ---------------------------------------------------------------------------


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# Change of basis from the body frame (+x = nose) to the world frame (+z = up)
# at zero attitude. Ry(-90) rather than a plain axis swap because swapping two
# axes of the identity has determinant -1 - that is a mirror, and it would draw
# the fins and the roll direction backwards without looking obviously wrong.
BODY_TO_WORLD = rot_y(-math.pi / 2)


def attitude_matrix(pitch_deg, roll_deg, yaw_deg):
    """Body->world rotation for this firmware's angle conventions.

    Applied right to left: roll about the nose, then pitch away from vertical,
    then the change of basis, then yaw about world up.
    """
    return (
        rot_z(math.radians(yaw_deg))
        @ BODY_TO_WORLD
        @ rot_y(math.radians(pitch_deg))
        @ rot_x(math.radians(roll_deg))
    )


# ---------------------------------------------------------------------------
# Rocket geometry, body frame, nose along +x
# ---------------------------------------------------------------------------

TUBE_R = 0.11
TAIL_X = -0.60
SHOULDER_X = 0.45
NOSE_X = 0.90
NSEG = 18


def build_rocket():
    """Return (faces, colors) where faces is a list of Nx3 body-frame polygons."""
    faces, colors = [], []
    ring = [
        (math.cos(2 * math.pi * i / NSEG), math.sin(2 * math.pi * i / NSEG))
        for i in range(NSEG)
    ]

    # Body tube
    for i in range(NSEG):
        cy0, cz0 = ring[i]
        cy1, cz1 = ring[(i + 1) % NSEG]
        faces.append(
            np.array(
                [
                    [TAIL_X, TUBE_R * cy0, TUBE_R * cz0],
                    [TAIL_X, TUBE_R * cy1, TUBE_R * cz1],
                    [SHOULDER_X, TUBE_R * cy1, TUBE_R * cz1],
                    [SHOULDER_X, TUBE_R * cy0, TUBE_R * cz0],
                ]
            )
        )
        colors.append("#c8ccd4")

    # Nose cone
    for i in range(NSEG):
        cy0, cz0 = ring[i]
        cy1, cz1 = ring[(i + 1) % NSEG]
        faces.append(
            np.array(
                [
                    [SHOULDER_X, TUBE_R * cy0, TUBE_R * cz0],
                    [SHOULDER_X, TUBE_R * cy1, TUBE_R * cz1],
                    [NOSE_X, 0.0, 0.0],
                ]
            )
        )
        colors.append("#e2574c")

    # Three fins at 120 deg. Asymmetric on purpose: with a symmetric airframe a
    # roll error of exactly 120 deg is invisible.
    for k in range(3):
        a = 2 * math.pi * k / 3
        cy, cz = math.cos(a), math.sin(a)
        faces.append(
            np.array(
                [
                    [TAIL_X, TUBE_R * cy, TUBE_R * cz],
                    [TAIL_X + 0.30, TUBE_R * cy, TUBE_R * cz],
                    [TAIL_X + 0.12, 2.6 * TUBE_R * cy, 2.6 * TUBE_R * cz],
                    [TAIL_X, 2.6 * TUBE_R * cy, 2.6 * TUBE_R * cz],
                ]
            )
        )
        colors.append("#4c7be2" if k else "#f0a500")  # one fin marked for roll

    return faces, colors


BODY_AXES = {
    "+X nose": (np.array([0.0, 0.0, 0.0]), np.array([1.25, 0.0, 0.0]), "#e2574c"),
    "+Y": (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.75, 0.0]), "#3aa76d"),
    "+Z": (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.75]), "#4c7be2"),
}


# ---------------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------------


class Sample:
    __slots__ = ("pitch", "roll", "yaw", "ax", "ay", "az", "gmag", "alt", "t", "n")

    def __init__(self):
        self.pitch = 90.0
        self.roll = 0.0
        self.yaw = 0.0
        self.ax = self.ay = self.az = 0.0
        self.gmag = 0.0
        self.alt = None
        self.t = 0.0
        self.n = 0


class Reader(threading.Thread):
    """Base: owns `latest`, guarded by `lock`, replaced wholesale each update."""

    daemon = True

    def __init__(self):
        super().__init__()
        self.lock = threading.Lock()
        self.latest = Sample()
        self.error = None
        self._stop_evt = threading.Event()

    def publish(self, s):
        s.t = time.monotonic()
        with self.lock:
            s.n = self.latest.n + 1
            self.latest = s

    def snapshot(self):
        with self.lock:
            return self.latest

    def stop(self):
        self._stop_evt.set()


class ConsoleReader(Reader):
    def __init__(self, port, baud=9600):
        super().__init__()
        self.port, self.baud = port, baud

    def run(self):
        try:
            with serial.Serial(self.port, self.baud, timeout=1.0) as ser:
                buf = b""
                while not self._stop_evt.is_set():
                    chunk = ser.read(256)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self._feed(line.decode("ascii", "replace").strip())
                    if len(buf) > 4096:  # never seen a newline: wrong baud
                        buf = buf[-512:]
        except Exception as exc:  # noqa: BLE001 - surfaced in the UI
            self.error = str(exc)

    def _feed(self, line):
        m = BENCH_RE.search(line) or STATUS_RE.search(line)
        if not m:
            return
        d = m.groupdict()
        s = Sample()
        s.ax, s.ay, s.az = float(d["ax"]), float(d["ay"]), float(d["az"])
        s.pitch, s.roll = float(d["pitch"]), float(d["roll"])
        s.yaw = float(d["yaw"]) if d.get("yaw") else 0.0
        s.gmag = (
            float(d["g"])
            if d.get("g")
            else math.sqrt(s.ax**2 + s.ay**2 + s.az**2) / G0
        )
        self.publish(s)


class SitReader(Reader):
    """Starts a SIT run and decodes Tablo 3 sensor packets.

    SIT is sensor-only. The pyro-firing scenario is SUT, and this script never
    sends that command.
    """

    def __init__(self, port, baud=115200):
        super().__init__()
        self.port, self.baud = port, baud

    def run(self):
        try:
            with serial.Serial(self.port, self.baud, timeout=0.5) as ser:
                ser.reset_input_buffer()
                ser.write(command_frame(CMD_SIT_START))
                buf = bytearray()
                while not self._stop_evt.is_set():
                    chunk = ser.read(256)
                    if chunk:
                        buf += chunk
                        self._consume(buf)
                    if len(buf) > 4096:
                        del buf[:-256]
                try:
                    ser.write(command_frame(CMD_STOP))
                    ser.flush()
                except Exception:
                    pass
        except Exception as exc:  # noqa: BLE001
            self.error = str(exc)

    def _consume(self, buf):
        while True:
            i = buf.find(bytes([HEADER_DATA]))
            if i < 0 or len(buf) - i < SENSOR_PACKET_LEN:
                if i > 0:
                    del buf[:i]
                return
            frame = bytes(buf[i : i + SENSOR_PACKET_LEN])
            if frame[-2:] != FOOTER:
                del buf[: i + 1]  # false header, resync past it
                continue
            # Checksum: header through last data byte, mod 256. Established
            # against the real ground software, not EK-7's worked examples.
            if sum(frame[:33]) & 0xFF != frame[33]:
                del buf[: i + 1]
                continue
            alt, _press, ax, ay, az, angx, angy, angz = struct.unpack(">8f", frame[1:33])
            s = Sample()
            s.alt = alt
            s.ax, s.ay, s.az = ax, ay, az
            s.roll, s.pitch, s.yaw = angx, angy, angz
            s.gmag = math.sqrt(ax * ax + ay * ay + az * az) / G0
            self.publish(s)
            del buf[: i + SENSOR_PACKET_LEN]


class DemoReader(Reader):
    """Synthetic sweep, so the view can be checked without hardware."""

    def run(self):
        t0 = time.monotonic()
        while not self._stop_evt.is_set():
            t = time.monotonic() - t0
            s = Sample()
            s.pitch = 90.0 + 90.0 * math.sin(t * 0.35)
            s.roll = (t * 40.0 + 180.0) % 360.0 - 180.0
            s.yaw = (t * 15.0 + 180.0) % 360.0 - 180.0
            p, r = math.radians(s.pitch), math.radians(s.roll)
            s.ax = G0 * math.cos(p)
            s.ay = G0 * math.sin(p) * math.sin(r)
            s.az = G0 * math.sin(p) * math.cos(r)
            s.gmag = 1.0
            self.publish(s)
            time.sleep(0.10)


# ---------------------------------------------------------------------------
# View
# ---------------------------------------------------------------------------

VERTICAL_DEADZONE = 12.0  # deg from vertical where roll stops meaning anything


def lerp_angle(cur, tgt, k):
    """Shortest-arc interpolation, for the wrapping angles only."""
    d = (tgt - cur + 180.0) % 360.0 - 180.0
    return cur + d * k


class View:
    def __init__(self, reader, source_name, smooth=True):
        self.reader = reader
        self.source_name = source_name
        self.smooth = smooth
        self.faces, self.colors = build_rocket()
        self.cur = [90.0, 0.0, 0.0]  # pitch, roll, yaw actually drawn

        self.fig = plt.figure(figsize=(9.5, 7.0))
        self.fig.canvas.manager.set_window_title("FCC orientation")
        self.ax = self.fig.add_subplot(111, projection="3d")
        self._setup_axes()

        self.body = Poly3DCollection(
            self.faces, facecolors=self.colors, edgecolors="#00000022", linewidths=0.4
        )
        self.ax.add_collection3d(self.body)
        # Seeded with the zero-attitude segments rather than []: add_collection3d
        # concatenates the segment arrays to autoscale, which raises on an empty
        # collection (matplotlib >= 3.9).
        segs0, cols0 = self._axis_segments(np.eye(3))
        self.axes3d = Line3DCollection(segs0, colors=cols0, linewidths=2.0)
        self.ax.add_collection3d(self.axes3d)

        self.txt = self.fig.text(
            0.015,
            0.975,
            "",
            va="top",
            ha="left",
            family="monospace",
            fontsize=10,
        )
        self.warn = self.fig.text(
            0.015, 0.06, "", va="bottom", ha="left", family="monospace",
            fontsize=10, color="#c8102e",
        )
        self.last_n = -1
        self.last_rx = None

    def _setup_axes(self):
        a = self.ax
        lim = 1.15  # rocket spans ~1.5 body units, so this fills the box
        a.set_xlim(-lim, lim)
        a.set_ylim(-lim, lim)
        a.set_zlim(-lim, lim)
        a.set_box_aspect([1, 1, 1])
        a.set_xlabel("east")
        a.set_ylabel("north")
        a.set_zlabel("up")
        a.set_xticklabels([])
        a.set_yticklabels([])
        a.set_zticklabels([])
        a.view_init(elev=18, azim=-58)

        # Ground plane, so "which way is up" is unambiguous.
        g = np.linspace(-lim, lim, 9)
        segs = []
        for v in g:
            segs.append([(-lim, v, -lim), (lim, v, -lim)])
            segs.append([(v, -lim, -lim), (v, lim, -lim)])
        a.add_collection3d(Line3DCollection(segs, colors="#00000018", linewidths=0.7))
        a.plot([0, 0], [0, 0], [-lim, lim], color="#00000030", linewidth=0.8, ls=":")

    @staticmethod
    def _axis_segments(R):
        segs, cols = [], []
        for _label, (p0, p1, col) in BODY_AXES.items():
            segs.append([tuple(R @ p0), tuple(R @ p1)])
            cols.append(col)
        return segs, cols

    def _target(self, s):
        return s.pitch, s.roll, s.yaw

    def update(self, _frame):
        s = self.reader.snapshot()

        if self.reader.error:
            self.warn.set_text(f"serial error: {self.reader.error}")

        tp, tr, ty = self._target(s)
        if self.smooth:
            k = 0.35
            self.cur[0] += (tp - self.cur[0]) * k  # pitch: 0..180, never wraps
            self.cur[1] = lerp_angle(self.cur[1], tr, k)
            self.cur[2] = lerp_angle(self.cur[2], ty, k)
        else:
            self.cur = [tp, tr, ty]

        R = attitude_matrix(*self.cur)
        self.body.set_verts([(R @ f.T).T for f in self.faces])

        segs, cols = self._axis_segments(R)
        self.axes3d.set_segments(segs)
        self.axes3d.set_color(cols)

        near_vertical = min(s.pitch, abs(180.0 - s.pitch)) < VERTICAL_DEADZONE
        roll_txt = "  (undefined)" if near_vertical else f"{s.roll:+7.1f}"

        if s.n != self.last_n:
            self.last_n = s.n
            self.last_rx = time.monotonic()
        stale = self.last_rx is None or (time.monotonic() - self.last_rx) > 3.0

        lines = [
            f"source  {self.source_name}",
            f"pitch  {s.pitch:+7.1f}  (0=nose up, 90=horizontal, 180=nose down)",
            f"roll   {roll_txt}",
            f"yaw    {s.yaw:+7.1f}" + ("   (not sent by console)" if self.source_name == "console" else "  (drifts)"),
            "",
            f"ax {s.ax:+7.2f}   ay {s.ay:+7.2f}   az {s.az:+7.2f}   m/s^2",
            f"|a| {s.gmag:6.3f} g" + ("   <-- should be 1.000" if abs(s.gmag - 1.0) > 0.05 else ""),
        ]
        if s.alt is not None:
            lines.append(f"alt {s.alt:8.1f} m")
        lines.append("")
        lines.append(f"samples {s.n}")
        self.txt.set_text("\n".join(lines))

        msgs = []
        if stale and s.n == 0:
            msgs.append("no data yet - check port, baud, and DEBUG_PRINTS_ENABLED")
        elif stale:
            msgs.append("stream stalled (>3 s since last sample)")
        if near_vertical:
            msgs.append("nose near vertical: roll is atan2 of two ~zero axes, ignore it")
        if not self.reader.error:
            self.warn.set_text("\n".join(msgs))

        return self.body, self.axes3d

    def run(self):
        # cache_frame_data=False: this is an unbounded live stream, not a
        # finite sequence, so there is nothing worth caching.
        self._anim = FuncAnimation(
            self.fig, self.update, interval=33, blit=False, cache_frame_data=False
        )
        plt.show()


# ---------------------------------------------------------------------------


def autodetect():
    ports = list(list_ports.comports())
    if not ports:
        return None
    for p in ports:
        if any(k in p.device for k in ("USB", "ACM")):
            return p.device
    return ports[0].device


def main():
    ap = argparse.ArgumentParser(
        description="Live 3D orientation view for the FCC.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("Usage:")[-1],
    )
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--port", help="serial port (autodetected if omitted)")
    ap.add_argument(
        "--source",
        choices=("console", "sit"),
        default="console",
        help="console = USART2 debug line (default), sit = USART6 Tablo 3 packets",
    )
    ap.add_argument("--baud", type=int, help="override baud (9600 console, 115200 sit)")
    ap.add_argument("--demo", action="store_true", help="synthetic data, no hardware")
    ap.add_argument(
        "--no-smooth",
        action="store_true",
        help="draw raw samples; default interpolates, since 4 Hz looks choppy",
    )
    args = ap.parse_args()

    if args.list:
        ports = list(list_ports.comports()) if serial else []
        if not ports:
            print("no serial ports found")
        # USB/ACM first: a PC typically enumerates dozens of legacy ttyS nodes
        # that are never the adapter, and burying the one real port in them
        # makes the listing useless.
        for p in sorted(ports, key=lambda p: ("USB" not in p.device and "ACM" not in p.device, p.device)):
            print(f"{p.device:20s} {p.description}")
        return 0

    if args.demo:
        reader, name = DemoReader(), "demo"
    else:
        if serial is None:
            sys.exit("pyserial is required:  pip install pyserial")
        port = args.port or autodetect()
        if not port:
            sys.exit("no serial port found; pass --port (see --list)")
        if args.source == "console":
            reader = ConsoleReader(port, args.baud or 9600)
            name = "console"
        else:
            reader = SitReader(port, args.baud or 115200)
            name = "sit"
        print(f"reading {name} from {port}")

    reader.start()
    view = View(reader, name, smooth=not args.no_smooth)
    try:
        view.run()
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        reader.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
