#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Keya WAS test emulator  -  bench-test the Keya calibration + direction logic
# without a real Keya motor or wheel IMU.
#
# It pretends to be BOTH sensors the calibration uses, driven by ONE virtual
# steering slider and the wheel geometry (L, T) you enter:
#
#   * Keya encoder  -> CAN  : heartbeat 0x07000001 (ext), buf[0..1] = uint16
#                             encoder ticks (big-endian). Ticks are proportional
#                             to the BIKE (virtual-centre) angle: ticks = k * dBike.
#   * Wheel IMU yaw -> UDP  : PGN 0xD6 to the Teensy (:8888), the angle of the
#                             RIGHT wheel, derived from the bike angle through the
#                             Ackermann geometry (bike -> wheel). This is what the
#                             reference IMU on the right knuckle would read.
#
# So the firmware's sweep should: read the right-wheel IMU angle, convert it
# back to the bike angle with its own wheelToBike(L,T), fit it against the ticks
# and recover  k = ticks/deg(bike)  with RMS ~ 0 on both sides. If the geometry
# or the encoder-direction logic is wrong, the recovered slopes won't match or
# the sign will be off - exactly what we want to catch.
#
# Wiring: a PEAK PCAN-USB on the Teensy's Keya CAN port (matching baud), plus an
# Ethernet/UDP path to the Teensy for the PGN (same as the WheelCalib bridge).
#
# Requires:  pip install python-can     (PCAN driver installed)
#            tkinter ships with CPython on Windows.
# -----------------------------------------------------------------------------
import math
import socket
import threading
import time
import tkinter as tk
from tkinter import ttk

try:
    import can  # python-can
except Exception:
    can = None

# ── Protocol constants ───────────────────────────────────────────────────────
KEYA_HEARTBEAT_ID = 0x07000001     # extended; firmware reads encoder from buf[0..1]
TEENSY_UDP_PORT   = 8888           # PGN destination (same as the bridge)
PGN_REF           = 0xD6           # reference wheel angle PGN

HEARTBEAT_HZ = 50                  # CAN heartbeat rate (keeps keyaDetected alive)
UDP_HZ       = 20                  # reference-angle rate


def bike_to_wheel(bike_deg, L, T):
    """Bike (virtual-centre) angle -> RIGHT wheel angle, signed (deg).
    Right wheel is INNER on a right turn (bike>0), OUTER on a left turn (bike<0).
    Exact inverse of the firmware's wheelToBike(L,T)."""
    if abs(bike_deg) < 1e-3 or L < 0.1:
        return bike_deg
    b = math.radians(abs(bike_deg))
    R = L / math.tan(b)                      # rear-centre turn radius
    r = (R - T / 2.0) if bike_deg > 0 else (R + T / 2.0)   # inner / outer
    if r < 0.01:
        r = 0.01
    w = math.degrees(math.atan(L / r))
    return w if bike_deg > 0 else -w


class Emulator:
    def __init__(self):
        self.lock = threading.Lock()
        # shared state (written by GUI, read by sender thread)
        self.bike_deg      = 0.0
        self.L             = 3.20
        self.T             = 1.80
        self.ticks_per_deg = 24.0     # ticks per BIKE degree (ground truth)
        self.reverse       = False    # flip tick sign (to match firmware/HW wiring)
        self.teensy_ip     = "192.168.5.126"
        self.bus           = None
        self.udp           = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.running       = False
        self.thread        = None
        # live readback for the GUI
        self.last_wheel    = 0.0
        self.last_ticks    = 0
        self.last_enc16    = 0

    # ── derived values from the current bike angle ──────────────────────────
    def compute(self):
        with self.lock:
            bike = self.bike_deg; L = self.L; T = self.T
            k = self.ticks_per_deg; rev = self.reverse
        wheel = bike_to_wheel(bike, L, T)
        ticks = k * bike * (-1.0 if rev else 1.0)
        enc16 = int(round(ticks)) & 0xFFFF
        return bike, wheel, ticks, enc16

    # ── sender thread: CAN heartbeat @50Hz + UDP PGN @20Hz ──────────────────
    def _run(self):
        udp_every = max(1, HEARTBEAT_HZ // UDP_HZ)
        i = 0
        period = 1.0 / HEARTBEAT_HZ
        while self.running:
            t0 = time.time()
            bike, wheel, ticks, enc16 = self.compute()
            self.last_wheel = wheel
            self.last_ticks = int(round(ticks))
            self.last_enc16 = enc16

            # CAN heartbeat: enc(2 BE), speed(2 BE)=0, rest 0
            if self.bus is not None:
                data = bytes([(enc16 >> 8) & 0xFF, enc16 & 0xFF, 0, 0, 0, 0, 0, 0])
                try:
                    self.bus.send(can.Message(arbitration_id=KEYA_HEARTBEAT_ID,
                                              is_extended_id=True, data=data))
                except Exception:
                    pass

            # UDP PGN 0xD6 (reference wheel angle) every udp_every ticks
            if i % udp_every == 0:
                self._send_udp(wheel)
            i += 1

            dt = period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)

    def _send_udp(self, yaw_deg):
        a = max(-32768, min(32767, int(round(yaw_deg * 100.0))))
        pkt = bytearray([0x80, 0x81, 0x7F, PGN_REF, 0x03,
                         a & 0xFF, (a >> 8) & 0xFF, 1, 0])
        crc = 0
        for j in range(2, len(pkt) - 1):
            crc = (crc + pkt[j]) & 0xFF
        pkt[-1] = crc
        try:
            self.udp.sendto(pkt, (self.teensy_ip, TEENSY_UDP_PORT))
        except Exception:
            pass

    def start(self, channel, bitrate):
        if can is None:
            raise RuntimeError("python-can not installed (pip install python-can)")
        self.bus = can.interface.Bus(bustype="pcan", channel=channel, bitrate=int(bitrate))
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None


# ── GUI ──────────────────────────────────────────────────────────────────────
class App:
    def __init__(self, root):
        self.emu = Emulator()
        root.title("Keya WAS test emulator")
        root.geometry("520x560")
        pad = {"padx": 6, "pady": 3}

        def row(parent, label, default, width=12):
            f = ttk.Frame(parent); f.pack(fill="x", **pad)
            ttk.Label(f, text=label, width=22).pack(side="left")
            v = tk.StringVar(value=str(default))
            ttk.Entry(f, textvariable=v, width=width).pack(side="left")
            return v

        # Connection
        cf = ttk.LabelFrame(root, text="Connection"); cf.pack(fill="x", **pad)
        self.v_ip   = row(cf, "Teensy IP", self.emu.teensy_ip)
        self.v_chan = row(cf, "PCAN channel", "PCAN_USBBUS1")
        self.v_baud = row(cf, "CAN bitrate (match port)", "250000")
        bf = ttk.Frame(cf); bf.pack(fill="x", **pad)
        self.btn = ttk.Button(bf, text="Connect & run", command=self.toggle)
        self.btn.pack(side="left")
        self.status = ttk.Label(bf, text="stopped", foreground="#a00")
        self.status.pack(side="left", padx=10)

        # Geometry
        gf = ttk.LabelFrame(root, text="Wheel geometry (must match WebConfig L/T)")
        gf.pack(fill="x", **pad)
        self.v_L   = row(gf, "Wheelbase L (m)", self.emu.L)
        self.v_T   = row(gf, "Track T (m)", self.emu.T)
        self.v_max = row(gf, "Max bike angle (deg)", 40)
        self.v_k   = row(gf, "ticks/deg (bike, truth)", self.emu.ticks_per_deg)
        self.v_rev = tk.BooleanVar(value=False)
        ttk.Checkbutton(gf, text="Reverse tick direction (flip if firmware reads wrong sign)",
                        variable=self.v_rev, command=self.apply).pack(anchor="w", **pad)

        # Steering slider
        sf = ttk.LabelFrame(root, text="Virtual steering  (turn right = positive)")
        sf.pack(fill="both", expand=True, **pad)
        self.slider = tk.Scale(sf, from_=-40, to=40, resolution=0.1,
                               orient="horizontal", length=460, command=self.on_slide)
        self.slider.pack(fill="x", **pad)
        bf2 = ttk.Frame(sf); bf2.pack(**pad)
        ttk.Button(bf2, text="Centre (0)", command=lambda: self.slider.set(0)).pack(side="left", padx=4)
        ttk.Button(bf2, text="Full left",  command=lambda: self.slider.set(self.slider.cget("from"))).pack(side="left", padx=4)
        ttk.Button(bf2, text="Full right", command=lambda: self.slider.set(self.slider.cget("to"))).pack(side="left", padx=4)

        # Live readout
        lf = ttk.LabelFrame(root, text="Sending"); lf.pack(fill="x", **pad)
        self.lbl = ttk.Label(lf, text="-", font=("Consolas", 11), justify="left")
        self.lbl.pack(anchor="w", **pad)

        self.apply()
        self._tick_gui()

    def apply(self):
        try:
            self.emu.L = float(self.v_L.get()); self.emu.T = float(self.v_T.get())
            self.emu.ticks_per_deg = float(self.v_k.get())
            self.emu.teensy_ip = self.v_ip.get().strip()
            self.emu.reverse = bool(self.v_rev.get())
            mx = abs(float(self.v_max.get()))
            self.slider.config(from_=-mx, to=mx)
        except ValueError:
            pass

    def on_slide(self, _):
        self.apply()
        with self.emu.lock:
            self.emu.bike_deg = float(self.slider.get())

    def toggle(self):
        if self.emu.running:
            self.emu.stop()
            self.btn.config(text="Connect & run")
            self.status.config(text="stopped", foreground="#a00")
        else:
            self.apply()
            try:
                self.emu.start(self.v_chan.get().strip(), self.v_baud.get().strip())
                self.btn.config(text="Stop")
                self.status.config(text="running", foreground="#0a0")
            except Exception as e:
                self.status.config(text=str(e), foreground="#a00")

    def _tick_gui(self):
        bike, wheel, ticks, enc16 = self.emu.compute()
        self.lbl.config(text=(
            f"bike angle  : {bike:+7.2f} deg   (slider)\n"
            f"wheel angle : {wheel:+7.2f} deg   -> UDP PGN 0xD6 (reference IMU)\n"
            f"ticks       : {ticks:+8.0f}       -> CAN enc16 = {enc16}\n"
            f"reverse     : {'ON' if self.emu.reverse else 'off'}"
        ))
        self.lbl.after(100, self._tick_gui)


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
