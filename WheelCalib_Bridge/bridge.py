#!/usr/bin/env python3
"""
WheelCalib_Bridge - laptop bridge between the ESP32 wheel-IMU and the Teensy.

Flow:
  ESP32 (soft-AP 192.168.4.1) --WiFi UDP--> this app --Ethernet UDP PGN--> Teensy

The app:
  - listens for the ESP32 broadcast "roll,pitch,yaw,fresh" on UDP :9000
  - shows roll / pitch / yaw live (mount the sensor flat: roll & pitch near 0,
    so the steer angle is pure yaw)
  - forwards yaw to the Teensy as reference wheel angle (PGN 0xD6) on :8888

Network setup (no static IP needed on the ESP side):
  1. Connect the laptop WiFi to the ESP32 AP  "WheelCalib"  (pass "calib1234").
     The laptop gets 192.168.4.x from the ESP.
  2. Keep the laptop's Ethernet on the AgOpenGPS/Teensy network (e.g. 192.168.5.x).
  3. Enter the Teensy IP below and press "Forward to Teensy".

Requires only the Python standard library (tkinter).
Run:  python bridge.py
"""

import socket
import threading
import time
import tkinter as tk
from tkinter import ttk

ESP_UDP_PORT   = 9000          # ESP32 broadcasts here
TEENSY_PORT    = 8888          # Teensy autosteer UDP port
TEENSY_IP_DEF  = "192.168.5.126"
PGN_REF        = 0xD6          # reference wheel angle PGN

state = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "imuOk": 0, "last": 0.0}
running = True


# ── ESP32 receive thread ──────────────────────────────────────────────────────
def rx_thread():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("", ESP_UDP_PORT))
    except OSError as e:
        print("UDP bind failed:", e)
        return
    s.settimeout(1.0)
    while running:
        try:
            data, _ = s.recvfrom(128)
            p = data.decode(errors="ignore").strip().split(",")
            if len(p) >= 4:
                state["roll"]  = float(p[0])
                state["pitch"] = float(p[1])
                state["yaw"]   = float(p[2])
                state["imuOk"] = int(p[3])     # 1=real TM171, 0=simulated/no IMU
                state["last"]  = time.time()
        except socket.timeout:
            pass
        except Exception:
            pass


# ── Teensy send ───────────────────────────────────────────────────────────────
tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_to_teensy(ip, yaw_deg, valid):
    a = int(round(yaw_deg * 100.0))
    a = max(-32768, min(32767, a))
    pkt = bytearray([0x80, 0x81, 0x7F, PGN_REF, 0x03,
                     a & 0xFF, (a >> 8) & 0xFF, 1 if valid else 0, 0])
    crc = 0
    for i in range(2, len(pkt) - 1):
        crc = (crc + pkt[i]) & 0xFF
    pkt[-1] = crc
    try:
        tx_sock.sendto(pkt, (ip, TEENSY_PORT))
        return True
    except Exception:
        return False


# ── GUI ───────────────────────────────────────────────────────────────────────
class App:
    def __init__(self, root):
        self.root = root
        root.title("Wheel Calib Bridge")
        root.geometry("360x300")

        f = ttk.Frame(root, padding=12)
        f.pack(fill="both", expand=True)

        self.link = tk.Label(f, text="ESP link: --", font=("Consolas", 11))
        self.link.pack(anchor="w")
        self.imu = tk.Label(f, text="IMU: --", font=("Consolas", 11, "bold"))
        self.imu.pack(anchor="w")

        box = ttk.LabelFrame(f, text="Reference IMU (mount flat: roll & pitch ~ 0)", padding=8)
        box.pack(fill="x", pady=8)
        self.lr = tk.Label(box, text="Roll  : --", font=("Consolas", 14)); self.lr.pack(anchor="w")
        self.lp = tk.Label(box, text="Pitch : --", font=("Consolas", 14)); self.lp.pack(anchor="w")
        self.ly = tk.Label(box, text="Yaw   : --", font=("Consolas", 14, "bold")); self.ly.pack(anchor="w")

        row = ttk.Frame(f); row.pack(fill="x", pady=4)
        ttk.Label(row, text="Teensy IP:").pack(side="left")
        self.ip = ttk.Entry(row, width=16); self.ip.insert(0, TEENSY_IP_DEF); self.ip.pack(side="left", padx=6)

        self.fwd = tk.BooleanVar(value=False)
        ttk.Checkbutton(f, text="Forward yaw to Teensy (reference angle)",
                        variable=self.fwd).pack(anchor="w", pady=4)

        self.zero = 0.0
        ttk.Button(f, text="Zero yaw (set current as 0°)", command=self.set_zero).pack(anchor="w")
        self.tx = tk.Label(f, text="TX: idle", font=("Consolas", 10)); self.tx.pack(anchor="w", pady=4)

        self.update()

    def set_zero(self):
        self.zero = state["yaw"]

    def update(self):
        age = time.time() - state["last"]
        link = age < 0.5                              # ESP packets arriving
        imu  = link and state["imuOk"] == 1
        self.link.config(text="ESP link: " + ("OK" if link else "-- (connect to WheelCalib WiFi)"),
                         fg=("#0a0" if link else "#a00"))
        if not link:
            self.imu.config(text="IMU: --", fg="#a00")
        elif imu:
            self.imu.config(text="IMU: TM171 OK", fg="#0a0")
        else:
            self.imu.config(text="no IMU detected (simulated values)", fg="#c60")

        self.lr.config(text=f"Roll  : {state['roll']:+7.2f}")
        self.lp.config(text=f"Pitch : {state['pitch']:+7.2f}")
        relyaw = state["yaw"] - self.zero
        self.ly.config(text=f"Yaw   : {relyaw:+7.2f}  (ref)")

        # Forward whenever the ESP link is up (real OR simulated). Send valid=1 so
        # the AIO web GUI shows the value and the whole chain is verifiable even
        # without an IMU; the bridge label above tells you if it's real or sim.
        if self.fwd.get() and link:
            if send_to_teensy(self.ip.get().strip(), relyaw, True):
                self.tx.config(text=f"TX: {relyaw:+.2f} deg -> {self.ip.get().strip()}"
                                    + ("" if imu else "  [SIM]"))
            else:
                self.tx.config(text="TX: send error")
        else:
            self.tx.config(text="TX: idle")

        self.root.after(20, self.update)   # 50 Hz


if __name__ == "__main__":
    threading.Thread(target=rx_thread, daemon=True).start()
    root = tk.Tk()
    App(root)
    try:
        root.mainloop()
    finally:
        running = False
