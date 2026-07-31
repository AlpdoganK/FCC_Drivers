#!/usr/bin/env python3
"""
Live dashboard for TelemetryPacket frames from an EBYTE E220-900T30S LoRa
module connected to this computer over a USB-UART adapter.

Four panels:

  orientation  3D rocket attitude, drawn from the packet's pitch/roll. The
               geometry and the angle conventions are imported from
               orientation_view.py rather than copied, so the two views can
               never drift apart.
  altitude     fused baro_alt vs. time.
  gps / link   GPS coordinates and link health (rate, CRC failures, stalls).
  byte stream  rolling hex dump of everything arriving on the port, decoded
               frames and junk alike.

The byte stream is the panel to look at first when nothing else populates. A
wrong baud shows as dense non-repeating garbage; a dead link shows as nothing
at all; LoRa corruption shows as recognisable frames with occasional mangled
bytes. The other three panels cannot tell those apart.

Wire format matches drivers/Inc/e220.h's TelemetryPacket, as transmitted by
LoRa_TransmitTelemetry_Blocking (packed, little-endian):

    uint8   Lora_ADDRH   (0x7B)  -- only present on the wire if the receiving
    uint8   Lora_ADDRL   (0xD3)  -- E220 is in transparent mode; a receiver in
    uint8   Lora_CH      (0x2B)  -- fixed-transmission mode strips these 3
    uint8   header       (0xAB)    <- frame search key is anchored here
    uint32  timestamp_ms
    uint8   flight_state
    float   ax, ay, az    -- accelerations (m/s^2)
    float   gx, gy, gz    -- body rates (deg/s): roll, pitch, yaw
    float   pitch         -- degrees FROM VERTICAL (0 = nose up, 90 = level)
    float   roll          -- degrees
    float   baro_alt      -- fused barometric altitude (m)
    float   gps_lat
    float   gps_lon
    uint16  crc          (CRC-16/CCITT-FALSE over timestamp..gps_lon)
    uint8   footer       (0x0A)

Since whether the 3 address/channel bytes show up on this computer's UART
depends on how the local E220 is configured (transparent vs. fixed mode), this
script searches for the header/footer pair directly (53-byte body, ignoring
whatever precedes it) instead of assuming a fixed frame length.

YAW IS NOT IN THE PACKET. The firmware dead-reckons rocket_yaw from the gyro
but does not transmit it, so the rocket is drawn at yaw = 0 and the readout
says so. gz (yaw RATE) is transmitted and shown as a number; it is deliberately
not integrated here, because a locally-invented heading that drifts would look
exactly like a real one.

Usage:
    ./lora_receiver.py                       # /dev/ttyUSB0 @ 115200
    ./lora_receiver.py /dev/ttyUSB1 115200
    ./lora_receiver.py --demo                # no hardware, synthetic stream

Requires: pyserial, matplotlib, numpy
"""

import argparse
import math
import os
import struct
import sys
import threading
import time
from collections import deque

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
except ImportError:
    serial = None  # only fatal if a real port is actually opened

# Rocket geometry and the firmware's angle conventions live in orientation_view.
# Importing keeps one definition of "what pitch means" across both tools. The
# path insert lets the script run from anywhere, not just from tools/.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from orientation_view import (  # noqa: E402
    BODY_AXES,
    G0,
    VERTICAL_DEADZONE,
    attitude_matrix,
    build_rocket,
    lerp_angle,
)

# ---------------------------------------------------------------------------
# Wire format
# ---------------------------------------------------------------------------

# Body = header..footer, i.e. TelemetryPacket minus the 3 leading
# Lora_ADDRH/ADDRL/CH routing bytes (which may or may not reach this UART).
BODY_FORMAT = "<BIB11fHB"
BODY_SIZE = struct.calcsize(BODY_FORMAT)
assert BODY_SIZE == 53

# CRC covers timestamp..gps_lon, i.e. the body minus the leading header byte
# and the trailing crc(2)+footer(1). Derived from BODY_SIZE rather than written
# out, so adding or removing a packet field cannot silently desync it.
CRC_SLICE = slice(1, BODY_SIZE - 3)

HEADER_BYTE = 0xAB
FOOTER_BYTE = 0x0A
ROUTING_PREFIX = bytes([0x7B, 0xD3, 0x2B])

HISTORY_LEN = 500     # rolling window of points shown on the altitude plot
RAW_KEEP = 512        # bytes retained for the hex panel
HEX_COLS = 32
HEX_ROWS = 6

# Turkish labels for the flight_sm.c state enum, in enum order. The numeric
# state is displayed alongside these, because that is what a debugger or an
# SWD read of current_flight_state actually shows - translating the name
# without the ordinal would make the UI harder to correlate with the firmware,
# not easier.
FLIGHT_STATES = [
    "RAMPA", "İTKİ", "SÜZÜLME", "MİN_İRTİFA",
    "APOJE", "ALÇALMA", "ANA_PARAŞÜT", "İNDİ",
]

# ---------------------------------------------------------------------------
# Dark theme
# ---------------------------------------------------------------------------

THEME = {
    "fig":    "#0f1117",  # figure background
    "panel":  "#161a23",  # axes background
    "edge":   "#2b3242",  # spines / pane edges
    "text":   "#e2e7f0",
    "muted":  "#8a93a8",  # axis labels, tick marks
    "grid":   "#ffffff12",
    "accent": "#4fc3f7",  # altitude trace
    "hex":    "#9ecbff",  # byte stream, tinted so it reads as raw data
    "warn":   "#ff7a7a",
    "pane":   (0.055, 0.065, 0.09, 1.0),  # 3D pane fill, as RGBA
}


def apply_theme():
    plt.rcParams.update({
        "figure.facecolor": THEME["fig"],
        "savefig.facecolor": THEME["fig"],
        "axes.facecolor": THEME["panel"],
        "axes.edgecolor": THEME["edge"],
        "axes.labelcolor": THEME["muted"],
        "axes.titlecolor": THEME["text"],
        "text.color": THEME["text"],
        "xtick.color": THEME["muted"],
        "ytick.color": THEME["muted"],
        "grid.color": THEME["grid"],
        "legend.facecolor": THEME["panel"],
        "legend.edgecolor": THEME["edge"],
    })


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
     ax, ay, az, gx, gy, gz, pitch, roll, baro_alt, gps_lat, gps_lon,
     crc, footer) = fields

    calc_crc = crc16_ccitt(body[CRC_SLICE])

    return {
        "timestamp": timestamp,
        "flight_state": flight_state,
        "ax": ax, "ay": ay, "az": az,
        "gx": gx, "gy": gy, "gz": gz,
        "pitch": pitch, "roll": roll,
        "baro_alt": baro_alt,
        "gps_lat": gps_lat, "gps_lon": gps_lon,
        "crc_ok": calc_crc == crc and header == HEADER_BYTE and footer == FOOTER_BYTE,
    }


def encode_packet(**kw) -> bytes:
    """Build a full 56-byte frame. Used by --demo, which means the demo stream
    exercises the same decoder the radio path does - a round-trip check of
    BODY_FORMAT rather than a separate mock that could agree with nothing."""
    body = struct.pack(
        BODY_FORMAT,
        HEADER_BYTE,
        kw.get("timestamp", 0),
        kw.get("flight_state", 0),
        kw.get("ax", 0.0), kw.get("ay", 0.0), kw.get("az", 0.0),
        kw.get("gx", 0.0), kw.get("gy", 0.0), kw.get("gz", 0.0),
        kw.get("pitch", 90.0), kw.get("roll", 0.0),
        kw.get("baro_alt", 0.0),
        kw.get("gps_lat", 0.0), kw.get("gps_lon", 0.0),
        0,
        FOOTER_BYTE,
    )
    crc = crc16_ccitt(body[CRC_SLICE])
    body = body[: BODY_SIZE - 3] + struct.pack("<HB", crc, FOOTER_BYTE)
    return ROUTING_PREFIX + body


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


# ---------------------------------------------------------------------------
# Readers
# ---------------------------------------------------------------------------


class Reader(threading.Thread):
    """Owns all shared state; the animation only ever calls snapshot()."""

    daemon = True

    def __init__(self):
        super().__init__()
        self.lock = threading.Lock()
        self._stop_evt = threading.Event()
        self.error = None

        self.pkt = None           # most recent decoded packet
        self.n_ok = 0
        self.n_crc_fail = 0
        self.last_rx = None       # monotonic time of last good frame
        self.raw = deque(maxlen=RAW_KEEP)
        self.t_hist = deque(maxlen=HISTORY_LEN)
        self.alt_hist = deque(maxlen=HISTORY_LEN)
        self._t0 = None
        self._rate_marks = deque(maxlen=16)

    def stop(self):
        self._stop_evt.set()

    def snapshot(self):
        with self.lock:
            return {
                "pkt": self.pkt,
                "n_ok": self.n_ok,
                "n_crc_fail": self.n_crc_fail,
                "last_rx": self.last_rx,
                "raw": bytes(self.raw),
                "t": list(self.t_hist),
                "alt": list(self.alt_hist),
                "rate": self._rate(),
                "error": self.error,
            }

    def _rate(self):
        if len(self._rate_marks) < 2:
            return 0.0
        span = self._rate_marks[-1] - self._rate_marks[0]
        return (len(self._rate_marks) - 1) / span if span > 0 else 0.0

    def _ingest(self, chunk: bytes, buf: bytearray):
        with self.lock:
            self.raw.extend(chunk)
        buf.extend(chunk)
        while True:
            frame, buf = find_next_frame(buf)
            if frame is None:
                return
            pkt = decode_packet(frame)
            with self.lock:
                if not pkt["crc_ok"]:
                    self.n_crc_fail += 1
                    continue
                self.n_ok += 1
                self.pkt = pkt
                self.last_rx = time.monotonic()
                self._rate_marks.append(self.last_rx)
                if self._t0 is None:
                    self._t0 = pkt["timestamp"]
                self.t_hist.append((pkt["timestamp"] - self._t0) / 1000.0)
                self.alt_hist.append(pkt["baro_alt"])


class SerialReader(Reader):
    def __init__(self, port, baud, echo=True):
        super().__init__()
        self.port, self.baud, self.echo = port, baud, echo

    def run(self):
        try:
            with serial.Serial(self.port, self.baud, timeout=0.1) as ser:
                buf = bytearray()
                while not self._stop_evt.is_set():
                    chunk = ser.read(256)
                    if chunk:
                        before = self.n_ok
                        self._ingest(chunk, buf)
                        if self.echo and self.n_ok != before:
                            print(format_line(self.n_ok, self.pkt))
                    if len(buf) > 4096:  # never framed: wrong baud or pure noise
                        del buf[:-256]
        except Exception as exc:  # noqa: BLE001 - surfaced in the UI
            self.error = str(exc)


class DemoReader(Reader):
    """Synthetic flight, encoded into real frames so the whole pipeline
    (framing, CRC, decode, display) is exercised without hardware."""

    def run(self):
        t0 = time.monotonic()
        while not self._stop_evt.is_set():
            t = time.monotonic() - t0
            pitch = 90.0 + 85.0 * math.sin(t * 0.30)
            roll = (t * 45.0 + 180.0) % 360.0 - 180.0
            p, r = math.radians(pitch), math.radians(roll)
            frame = encode_packet(
                timestamp=int(t * 1000),
                flight_state=min(int(t / 4) % 8, 7),
                ax=G0 * math.cos(p),
                ay=G0 * math.sin(p) * math.sin(r),
                az=G0 * math.sin(p) * math.cos(r),
                gx=45.0, gy=12.0 * math.cos(t), gz=8.0 * math.sin(t * 0.7),
                pitch=pitch, roll=roll,
                baro_alt=max(0.0, 900.0 * math.sin(t * 0.12)),
                gps_lat=39.925018 + 0.0004 * math.sin(t * 0.1),
                gps_lon=32.836956 + 0.0004 * math.cos(t * 0.1),
            )
            self._ingest(frame, bytearray())
            time.sleep(1.0)


def format_line(n, pkt):
    st = pkt["flight_state"]
    state = f"{st}:{FLIGHT_STATES[st]}" if st < len(FLIGHT_STATES) else f"{st}:BİLİNMİYOR"
    return (
        f"[{n:5d}] t={pkt['timestamp']:>8d}ms  durum={state:<16s}  "
        f"ivme=({pkt['ax']:+6.2f},{pkt['ay']:+6.2f},{pkt['az']:+6.2f})  "
        f"jiro=({pkt['gx']:+7.2f},{pkt['gy']:+7.2f},{pkt['gz']:+7.2f})  "
        f"yunuslama={pkt['pitch']:+6.1f} yatış={pkt['roll']:+6.1f}  "
        f"irtifa={pkt['baro_alt']:7.1f}m  "
        f"gps=({pkt['gps_lat']:.5f},{pkt['gps_lon']:.5f})"
    )


# ---------------------------------------------------------------------------
# Dashboard
# ---------------------------------------------------------------------------


class Dashboard:
    def __init__(self, reader, source_name, smooth=True):
        self.reader = reader
        self.source_name = source_name
        self.smooth = smooth
        self.faces, self.colors = build_rocket()
        self.cur = [90.0, 0.0]  # pitch, roll actually drawn

        apply_theme()
        self.fig = plt.figure(figsize=(15.0, 9.0))
        self.fig.canvas.manager.set_window_title("FCC LoRa Telemetri")
        gs = self.fig.add_gridspec(
            3, 2, width_ratios=[1.05, 1.0], height_ratios=[1.0, 1.0, 0.45],
            left=0.04, right=0.975, top=0.95, bottom=0.05, hspace=0.32, wspace=0.16,
        )

        self.ax3d = self.fig.add_subplot(gs[0:2, 0], projection="3d")
        self.axalt = self.fig.add_subplot(gs[0, 1])
        self.axgps = self.fig.add_subplot(gs[1, 1])
        self.axhex = self.fig.add_subplot(gs[2, :])

        self._setup_3d()
        self._setup_alt()
        self._setup_text_panels()

    # -- panel setup --------------------------------------------------------

    def _setup_3d(self):
        a = self.ax3d
        lim = 1.15
        a.set_xlim(-lim, lim)
        a.set_ylim(-lim, lim)
        a.set_zlim(-lim, lim)
        a.set_box_aspect([1, 1, 1])
        a.set_xlabel("doğu")
        a.set_ylabel("kuzey")
        a.set_zlabel("yukarı")
        a.set_xticklabels([])
        a.set_yticklabels([])
        a.set_zticklabels([])
        a.view_init(elev=18, azim=-58)
        a.set_title("YÖNELİM", fontsize=10, color=THEME["text"])

        # The 3D panes default to near-white and would blaze out of a dark
        # figure, so they are filled explicitly rather than left to rcParams
        # (axes.facecolor does not reach them).
        for axis in (a.xaxis, a.yaxis, a.zaxis):
            axis.set_pane_color(THEME["pane"])
            axis.line.set_color(THEME["edge"])
        a.tick_params(colors=THEME["muted"])

        # Ground plane and the vertical reference, light-on-dark now: the
        # original near-black grid was invisible against this background.
        g = np.linspace(-lim, lim, 9)
        segs = []
        for v in g:
            segs.append([(-lim, v, -lim), (lim, v, -lim)])
            segs.append([(v, -lim, -lim), (v, lim, -lim)])
        a.add_collection3d(Line3DCollection(segs, colors="#ffffff14", linewidths=0.7))
        a.plot([0, 0], [0, 0], [-lim, lim], color="#ffffff2e", linewidth=0.8, ls=":")

        self.body = Poly3DCollection(
            self.faces, facecolors=self.colors, edgecolors="#ffffff22", linewidths=0.4
        )
        a.add_collection3d(self.body)
        # Seeded with zero-attitude segments rather than []: add_collection3d
        # concatenates segment arrays to autoscale and raises on an empty
        # collection (matplotlib >= 3.9).
        segs0, cols0 = self._axis_segments(np.eye(3))
        self.axes3d = Line3DCollection(segs0, colors=cols0, linewidths=2.0)
        a.add_collection3d(self.axes3d)

        self.txt3d = self.fig.text(
            0.012, 0.945, "", va="top", ha="left", family="monospace",
            fontsize=9, color=THEME["text"],
        )

    def _setup_alt(self):
        a = self.axalt
        self.line_alt, = a.plot([], [], color=THEME["accent"], linewidth=1.6,
                                label="baro_alt (füzyonlu)")
        a.set_xlabel("zaman (s)", fontsize=9)
        a.set_ylabel("irtifa (m)", fontsize=9)
        a.set_title("İRTİFA", fontsize=10, color=THEME["text"])
        a.grid(True, alpha=0.35)
        leg = a.legend(loc="upper left", fontsize=8)
        for t in leg.get_texts():
            t.set_color(THEME["text"])
        a.tick_params(labelsize=8)

    def _setup_text_panels(self):
        for a, title in ((self.axgps, "GPS / BAĞLANTI"), (self.axhex, "BAYT AKIŞI")):
            a.set_title(title, fontsize=10, loc="left", color=THEME["text"])
            a.set_xticks([])
            a.set_yticks([])
            for sp in a.spines.values():
                sp.set_color(THEME["edge"])

        self.txtgps = self.axgps.text(
            0.02, 0.94, "", va="top", ha="left", family="monospace",
            fontsize=10, color=THEME["text"], transform=self.axgps.transAxes,
        )
        self.txthex = self.axhex.text(
            0.012, 0.90, "", va="top", ha="left", family="monospace",
            fontsize=8.5, color=THEME["hex"], transform=self.axhex.transAxes,
        )
        self.warn = self.fig.text(
            0.012, 0.012, "", va="bottom", ha="left", family="monospace",
            fontsize=9, color=THEME["warn"],
        )

    @staticmethod
    def _axis_segments(R):
        segs, cols = [], []
        for _label, (p0, p1, col) in BODY_AXES.items():
            segs.append([tuple(R @ p0), tuple(R @ p1)])
            cols.append(col)
        return segs, cols

    # -- rendering ----------------------------------------------------------

    def _render_hex(self, raw: bytes) -> str:
        """Rolling hex dump, newest byte at the bottom right. A '|' before a
        byte marks 0xAB, the frame header.

        Rows are phase-locked to the most recent header so the dump does not
        re-wrap on every redraw as bytes arrive - purely display stability. It
        does NOT make the headers line up in a column: a frame is 56 bytes (53
        if the local E220 strips the routing prefix) and neither divides the row
        width, so header markers are expected to walk across the rows. What a
        healthy link looks like here is a regular pattern that repeats; what a
        wrong baud looks like is dense noise with no repeat and few markers."""
        if not raw:
            return "(hiç bayt alınmadı)"
        window = raw[-(HEX_COLS * HEX_ROWS):]
        anchor = window.rfind(HEADER_BYTE)
        if anchor > 0:
            window = window[anchor % HEX_COLS:]
        lines = []
        for i in range(0, len(window), HEX_COLS):
            row = window[i: i + HEX_COLS]
            lines.append("".join(f"|{b:02x}" if b == HEADER_BYTE else f" {b:02x}"
                                 for b in row))
        return "\n".join(lines[-HEX_ROWS:]) if lines else "(toplanıyor...)"

    def update(self, _frame):
        snap = self.reader.snapshot()
        pkt = snap["pkt"]

        # -- orientation
        if pkt is not None:
            tp, tr = pkt["pitch"], pkt["roll"]
            if self.smooth:
                k = 0.35
                self.cur[0] += (tp - self.cur[0]) * k  # pitch: 0..180, never wraps
                self.cur[1] = lerp_angle(self.cur[1], tr, k)
            else:
                self.cur = [tp, tr]

        # Yaw is not transmitted; drawn at 0 rather than invented from gz.
        R = attitude_matrix(self.cur[0], self.cur[1], 0.0)
        self.body.set_verts([(R @ f.T).T for f in self.faces])
        segs, cols = self._axis_segments(R)
        self.axes3d.set_segments(segs)
        self.axes3d.set_color(cols)

        # -- text overlays
        if pkt is None:
            self.txt3d.set_text(f"kaynak  {self.source_name}\npaket bekleniyor...")
            self.txtgps.set_text("henüz paket yok")
        else:
            near_vertical = min(pkt["pitch"], abs(180.0 - pkt["pitch"])) < VERTICAL_DEADZONE
            roll_txt = "  (tanımsız)" if near_vertical else f"{pkt['roll']:+7.1f}"
            st = pkt["flight_state"]
            state = f"{st}  {FLIGHT_STATES[st]}" if st < len(FLIGHT_STATES) else f"{st}  BİLİNMİYOR"
            gmag = math.sqrt(pkt["ax"] ** 2 + pkt["ay"] ** 2 + pkt["az"] ** 2) / G0
            self.txt3d.set_text("\n".join([
                f"kaynak  {self.source_name}",
                f"durum   {state}",
                f"süre    {pkt['timestamp'] / 1000.0:8.1f} s",
                "",
                f"yunuslama {pkt['pitch']:+7.1f}  (0=burun yukarı, 90=yatay)",
                f"yatış     {roll_txt}",
                 "sapma        yok    (pakette gönderilmiyor)",
                "",
                f"ax {pkt['ax']:+7.2f}  ay {pkt['ay']:+7.2f}  az {pkt['az']:+7.2f}  m/s^2",
                f"gx {pkt['gx']:+7.2f}  gy {pkt['gy']:+7.2f}  gz {pkt['gz']:+7.2f}  °/s",
                f"|a| {gmag:6.3f} g",
            ]))

            lat, lon = pkt["gps_lat"], pkt["gps_lon"]
            no_fix = (lat == 0.0 and lon == 0.0)
            stale = snap["last_rx"] is None or (time.monotonic() - snap["last_rx"]) > 3.0
            gps_lines = [
                f"enlem    {lat:12.6f}",
                f"boylam   {lon:12.6f}",
            ]
            if no_fix:
                gps_lines.append("         KONUM YOK - alıcı gerçek bir çözüm")
                gps_lines.append("         bulana kadar yazılım 0.0 gönderir")
            gps_lines += [
                "",
                f"irtifa   {pkt['baro_alt']:9.1f} m",
                "",
                f"paket       {snap['n_ok']}",
                f"crc hatası  {snap['n_crc_fail']}",
                f"hız         {snap['rate']:.2f} Hz",
                f"bağlantı    {'DURDU' if stale else 'canlı'}",
            ]
            self.txtgps.set_text("\n".join(gps_lines))

        # -- altitude
        if snap["t"]:
            self.line_alt.set_data(snap["t"], snap["alt"])
            self.axalt.relim()
            self.axalt.autoscale_view()

        # -- byte stream
        self.txthex.set_text(self._render_hex(snap["raw"]))

        # -- warnings
        msgs = []
        if snap["error"]:
            msgs.append(f"seri port hatası: {snap['error']}")
        elif snap["n_ok"] == 0 and snap["raw"]:
            msgs.append("bayt geliyor ama geçerli paket yok - baud hızını kontrol edin "
                        "(yukarıdaki bayt akışına bakın)")
        elif snap["n_ok"] == 0:
            msgs.append("henüz veri yok - portu, baud hızını ve LORA_TX_ENABLED değerini kontrol edin")
        elif snap["last_rx"] and (time.monotonic() - snap["last_rx"]) > 3.0:
            msgs.append("akış durdu (son geçerli paketten bu yana >3 s)")
        self.warn.set_text("\n".join(msgs))

        return self.body, self.axes3d

    def run(self):
        # cache_frame_data=False: unbounded live stream, nothing worth caching.
        self._anim = FuncAnimation(
            self.fig, self.update, interval=50, blit=False, cache_frame_data=False
        )
        plt.show()


# ---------------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(
        description="Live LoRa telemetry dashboard for the FCC.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("Usage:")[-1],
    )
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0",
                    help="serial port (default /dev/ttyUSB0)")
    ap.add_argument("baud", nargs="?", type=int, default=115200,
                    help="baud (default 115200, matches USART1 in usart.c)")
    ap.add_argument("--demo", action="store_true",
                    help="synthetic telemetry, no hardware")
    ap.add_argument("--quiet", action="store_true",
                    help="do not echo decoded frames to the terminal")
    ap.add_argument("--no-smooth", action="store_true",
                    help="draw raw attitude; default interpolates, since 1 Hz looks steppy")
    args = ap.parse_args()

    if args.demo:
        reader, name = DemoReader(), "demo"
    else:
        if serial is None:
            sys.exit("pyserial is required:  pip install pyserial")
        reader = SerialReader(args.port, args.baud, echo=not args.quiet)
        name = f"{args.port} @ {args.baud}"
        print(f"Opening {args.port} @ {args.baud} baud...")

    reader.start()
    dash = Dashboard(reader, name, smooth=not args.no_smooth)
    try:
        dash.run()
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        reader.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
