#!/usr/bin/env python3
"""
AIO v4 CAN Test Suite
Tests CAN hardware and firmware: loopback, Keya emulation, IMU-WAS, steer brand guide.

Requirements:
    pip install python-can requests

Usage:
    python can_test.py [--ip <aio-ip>] [--interface <pcan|socketcan|...>] [--channel <ch>] [--bitrate <bps>]

Examples:
    python can_test.py --ip 192.168.5.126 --interface pcan --channel PCAN_USBBUS1
    python can_test.py --ip 192.168.5.126 --interface socketcan --channel can0
    python can_test.py --ip 192.168.5.126 --interface canable --channel COM3
"""

import sys
import os
import math
import json
import time
import random
import argparse
import datetime

try:
    import can
except ImportError:
    print("ERROR: python-can not installed. Run:  pip install python-can")
    sys.exit(1)

try:
    import requests
except ImportError:
    print("ERROR: requests not installed. Run:  pip install requests")
    sys.exit(1)


# ── Default config ─────────────────────────────────────────────────────────────
DEFAULT_AIO_IP  = "192.168.5.126"
DEFAULT_IFACE   = "pcan"
DEFAULT_CHANNEL = "PCAN_USBBUS1"
DEFAULT_BITRATE = 250000

# ── CAN mode values (match zConfig.h) ─────────────────────────────────────────
CAN_MODE_OFF     = 0
CAN_MODE_KEYA    = 1
CAN_MODE_IMU     = 2
CAN_MODE_VBUS    = 3
CAN_MODE_KBUS    = 4
CAN_MODE_ISO     = 5
CAN_MODE_J1939   = 6
CAN_MODE_CANTEST = 7
CAN_MODE_CUSTOM  = 8

# WAS source values (match zConfig.h)
WAS_SOURCE_ADS1115   = 0
WAS_SOURCE_KEYA      = 1
WAS_SOURCE_IMU_CAN   = 2
WAS_SOURCE_CAN_VALVE = 3

# CANtest IDs (match zCANBUS.ino)
CANTEST_TX_ID = 0x7E0   # Python → Teensy
CANTEST_RX_ID = 0x7E1   # Teensy → Python (echo, data+1)

# Keya IDs
KEYA_HEARTBEAT_ID = 0x07000001  # motor → AIO
KEYA_COMMAND_ID   = 0x06000001  # AIO → motor

# IMU WAS CAN ID (match zCAN1.ino)
IMU_WAS_CAN_ID = 0x300


# ── ANSI colors ────────────────────────────────────────────────────────────────
class C:
    RST  = "\033[0m"
    BOLD = "\033[1m"
    GRN  = "\033[92m"
    RED  = "\033[91m"
    YEL  = "\033[93m"
    CYN  = "\033[96m"
    GRY  = "\033[90m"
    BLU  = "\033[94m"


def cp(color, *args, end="\n"):
    print(f"{color}{''.join(str(a) for a in args)}{C.RST}", end=end, flush=True)


def header(title):
    print()
    cp(C.BOLD + C.CYN, "─" * 62)
    cp(C.BOLD + C.CYN, f"  {title}")
    cp(C.BOLD + C.CYN, "─" * 62)


# ── Log file ───────────────────────────────────────────────────────────────────
_LOG_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    f"can_test_{datetime.datetime.now():%Y%m%d_%H%M%S}.log"
)
# Create log file immediately so the path is always valid
with open(_LOG_FILE, "w", encoding="utf-8") as _f:
    _f.write(f"AIO v4 CAN Test Suite — {datetime.datetime.now():%Y-%m-%d %H:%M:%S}\n\n")


def log(test_id, test_name, result_lines):
    ts    = datetime.datetime.now().isoformat(timespec="seconds")
    entry = f"[{ts}] [{test_id}] {test_name}\n"
    entry += "\n".join(f"  {l}" for l in result_lines) + "\n"
    with open(_LOG_FILE, "a", encoding="utf-8") as f:
        f.write(entry + "\n")


# ── AIO web API ────────────────────────────────────────────────────────────────
def _aio_get(ip, path, timeout=4):
    url = f"http://{ip}/{path.lstrip('/')}"
    try:
        r = requests.get(url, timeout=timeout)
        r.raise_for_status()
        return r
    except requests.RequestException:
        return None


def aio_status(ip):
    r = _aio_get(ip, "/api/status")
    if r is None:
        return None
    try:
        return r.json()
    except Exception:
        return None


def aio_save(ip, params: dict):
    qs = "&".join(f"{k}={v}" for k, v in params.items())
    return _aio_get(ip, f"/api/save?{qs}") is not None


def wait_online(ip, timeout=40):
    cp(C.YEL, f"  Waiting for AIO at {ip}", end=" ")
    t0 = time.time()
    while time.time() - t0 < timeout:
        if aio_status(ip) is not None:
            cp(C.GRN, " online!")
            return True
        time.sleep(1)
        print(".", end="", flush=True)
    cp(C.RED, "\n  Timeout — AIO not responding")
    return False


# ── CAN bus helpers ────────────────────────────────────────────────────────────
def open_can(iface, channel, bitrate):
    try:
        bus = can.interface.Bus(interface=iface, channel=channel, bitrate=bitrate)
        cp(C.GRN, f"  CAN: {iface}/{channel} @ {bitrate // 1000} kbps — open")
        return bus
    except Exception as e:
        cp(C.RED, f"  Cannot open CAN bus: {e}")
        return None


def _send(bus, arb_id, data, extended=True):
    msg = can.Message(arbitration_id=arb_id, data=bytes(data), is_extended_id=extended)
    try:
        bus.send(msg)
        return True
    except can.CanError:
        return False


def _recv(bus, timeout=0.05):
    try:
        return bus.recv(timeout=timeout)
    except Exception:
        return None


# ── Console prompts ────────────────────────────────────────────────────────────
def ask_port():
    while True:
        s = input("  Which CAN port? (1 / 2 / 3): ").strip()
        if s in ("1", "2", "3"):
            return int(s)
        cp(C.RED, "  Enter 1, 2 or 3")


def ask_yn(prompt):
    while True:
        s = input(f"  {prompt} [y/n]: ").strip().lower()
        if s in ("y", "yes"):
            return True
        if s in ("n", "no"):
            return False
        cp(C.RED, "  Enter y or n")


def pause(msg="Press Enter to continue ..."):
    input(f"  {msg}")


# ── Set CAN port mode via API + restart guide ──────────────────────────────────
_MODE_NAMES = ["Off", "Keya", "IMU-WAS", "V_Bus", "K_Bus", "ISO",
               "J1939", "CANtest", "Custom"]


def set_can_mode(ip, port, mode, baud=250000, guide_restart=True):
    ok = aio_save(ip, {f"can{port}Mode": mode, f"can{port}Baud": baud})
    name = _MODE_NAMES[mode] if mode < len(_MODE_NAMES) else str(mode)
    if ok:
        cp(C.GRN, f"  AIO: CAN{port} = {name} @ {baud // 1000} kbps — saved")
    else:
        cp(C.YEL, f"  API save failed — set CAN{port} = '{name}' manually in web UI")

    if guide_restart:
        cp(C.YEL, "  Power-cycle or reset Teensy now")
        pause()
        return wait_online(ip)
    return True


# ═══════════════════════════════════════════════════════════════════════════════
# TEST 1 — CAN loopback (CANtest mode)
# ═══════════════════════════════════════════════════════════════════════════════
def test_cantest_loopback(cfg):
    header("Test 1 — CAN Loopback  (CANtest mode)")
    ip, iface, channel, bitrate = cfg["ip"], cfg["interface"], cfg["channel"], cfg["bitrate"]
    cp(C.GRY, "  Firmware echoes 0x7E0 frames back as 0x7E1 with each data byte +1.")
    cp(C.GRY, "  Verifies CAN TX, RX, and Teensy firmware path.")

    while True:
        port = ask_port()

        # Configure + restart
        if not set_can_mode(ip, port, CAN_MODE_CANTEST, bitrate, guide_restart=True):
            return

        bus = open_can(iface, channel, bitrate)
        if bus is None:
            if not ask_yn("Retry?"):
                return
            continue

        cp(C.CYN, "  Running 20-second loopback test at 10 msg/s ...")

        sent, recv = 0, 0
        latencies  = []
        seq        = 0
        pending    = {}   # seq -> send_time_s
        last_send  = time.monotonic()
        t_end      = time.monotonic() + 20.0

        while time.monotonic() < t_end:
            now = time.monotonic()

            # Transmit at 10 Hz
            if now - last_send >= 0.1:
                last_send = now
                lo, hi = seq & 0xFF, (seq >> 8) & 0xFF
                _send(bus, CANTEST_TX_ID,
                      [lo, hi, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF],
                      extended=False)
                pending[seq] = now
                sent += 1
                seq = (seq + 1) & 0xFFFF

            # Receive echo
            msg = _recv(bus, timeout=0.008)
            if msg is not None and msg.arbitration_id == CANTEST_RX_ID and len(msg.data) >= 2:
                # echo: data[0]=sent_lo+1, data[1]=sent_hi+1
                s = ((msg.data[0] - 1) & 0xFF) | (((msg.data[1] - 1) & 0xFF) << 8)
                if s in pending:
                    latencies.append((now - pending.pop(s)) * 1000)
                    recv += 1

        bus.shutdown()

        lost        = sent - recv
        success_pct = 100 * recv / max(sent, 1)
        avg_lat     = sum(latencies) / len(latencies) if latencies else 0
        max_lat     = max(latencies)                  if latencies else 0

        print()
        cp(C.BOLD, "  ── Results ──")
        print(f"  Sent:      {sent}")
        print(f"  Received:  {recv}  ({success_pct:.1f} %)")
        print(f"  Lost:      {lost}")
        if latencies:
            print(f"  Latency:   avg {avg_lat:.1f} ms   max {max_lat:.1f} ms")

        status = "PASS" if success_pct >= 95 else "FAIL"
        cp(C.GRN if status == "PASS" else C.RED, f"  Result: {status}")

        log("T1", f"CAN{port} Loopback",
            [f"sent={sent} recv={recv} lost={lost} success={success_pct:.1f}%",
             f"latency avg={avg_lat:.1f}ms max={max_lat:.1f}ms",
             f"result={status}"])

        if not ask_yn("Test another port?"):
            break

    if ask_yn("Restore port to Off and save?"):
        set_can_mode(ip, port, CAN_MODE_OFF, bitrate, guide_restart=False)
        cp(C.YEL, "  Restart Teensy to apply")


# ═══════════════════════════════════════════════════════════════════════════════
# TEST 2 — Keya motor emulation
# ═══════════════════════════════════════════════════════════════════════════════
def test_keya_emulation(cfg):
    header("Test 2 — Keya Motor Emulation")
    ip, iface, channel, bitrate = cfg["ip"], cfg["interface"], cfg["channel"], cfg["bitrate"]
    cp(C.GRY, "  Emulates a Keya motor: sends heartbeat, reads AIO steer speed commands.")
    cp(C.GRY, "  AIO must be configured with Keya mode on the port being tested.")

    port = ask_port()
    set_can_mode(ip, port, CAN_MODE_KEYA, bitrate, guide_restart=True)
    aio_save(ip, {"wasSource": WAS_SOURCE_KEYA})

    bus = open_can(iface, channel, bitrate)
    if bus is None:
        return

    # Interactive steer guide: 5 left + 5 right, 3 s each
    STEP_DUR = 3.0
    guide = (["LEFT"] * 5 + ["RIGHT"] * 5 + ["STOP"])
    step_idx   = 0
    step_start = time.monotonic()

    cp(C.YEL, f"\n  Step 1/11: Steer LEFT")

    position      = 0       # simulated encoder ticks
    speed_sim     = 0       # simulated actual speed
    last_set_speed = 0
    last_send     = time.monotonic()
    last_print    = time.monotonic()

    dir_ticks = {"LEFT": [], "RIGHT": []}

    try:
        while step_idx < len(guide):
            now = time.monotonic()
            dir_now = guide[step_idx]

            # Advance step
            if now - step_start >= STEP_DUR:
                step_idx += 1
                step_start = now
                if step_idx < len(guide):
                    label = f"Steer {guide[step_idx]}" if guide[step_idx] != "STOP" else "Release"
                    cp(C.YEL, f"\n  Step {step_idx + 1}/{len(guide)}: {label}")

            # Update simulated state
            if dir_now == "LEFT":
                position -= 2;  speed_sim = -100
            elif dir_now == "RIGHT":
                position += 2;  speed_sim = 100
            else:
                speed_sim = 0

            # Send Keya heartbeat at 100 Hz
            if now - last_send >= 0.01:
                last_send = now
                enc = ((-position) & 0xFFFF)
                hb  = [(enc >> 8) & 0xFF, enc & 0xFF,
                       (speed_sim >> 8) & 0xFF, speed_sim & 0xFF,
                       0, 0, 0, 0]
                _send(bus, KEYA_HEARTBEAT_ID, hb, extended=True)

            # Read AIO speed command
            msg = _recv(bus, timeout=0.001)
            if msg is not None and msg.arbitration_id == KEYA_COMMAND_ID and len(msg.data) >= 6:
                last_set_speed = int.from_bytes(msg.data[4:6], "big", signed=True)

            # Print at 2 Hz
            if now - last_print >= 0.5:
                last_print = now
                print(f"\r  pos={position:6d} tks  simSpd={speed_sim:5d}  AIO setSpd={last_set_speed:5d}",
                      end="", flush=True)
                if dir_now in dir_ticks:
                    dir_ticks[dir_now].append(position)

    except KeyboardInterrupt:
        pass

    bus.shutdown()
    print()

    left_range  = (max(dir_ticks["LEFT"])  - min(dir_ticks["LEFT"]))  if dir_ticks["LEFT"]  else 0
    right_range = (max(dir_ticks["RIGHT"]) - min(dir_ticks["RIGHT"])) if dir_ticks["RIGHT"] else 0
    status = "PASS" if left_range > 5 and right_range > 5 else "WARN"

    print()
    cp(C.BOLD, "  ── Results ──")
    print(f"  Encoder range LEFT:  {left_range} ticks")
    print(f"  Encoder range RIGHT: {right_range} ticks")
    cp(C.GRN if status == "PASS" else C.YEL, f"  Result: {status}")

    log("T2", f"CAN{port} Keya Emulation",
        [f"left_range={left_range}tks right_range={right_range}tks", f"result={status}"])


# ═══════════════════════════════════════════════════════════════════════════════
# TEST 3 — IMU as WAS
# ═══════════════════════════════════════════════════════════════════════════════
def test_imu_was(cfg):
    header("Test 3 — IMU as WAS  (CAN wheel-angle source)")
    ip, iface, channel, bitrate = cfg["ip"], cfg["interface"], cfg["channel"], cfg["bitrate"]
    cp(C.GRY, "  Sends synthetic yaw angles via CAN (ID 0x300, format: int16 centideg big-endian).")
    cp(C.GRY, "  Reads actualAngle from AIO status and verifies it follows the input.")

    port = ask_port()
    set_can_mode(ip, port, CAN_MODE_IMU, bitrate, guide_restart=False)
    aio_save(ip, {"wasSource": WAS_SOURCE_IMU_CAN})
    cp(C.YEL, "  Restart Teensy to activate IMU-WAS mode on this port")
    pause()
    if not wait_online(ip):
        return

    bus = open_can(iface, channel, bitrate)
    if bus is None:
        return

    cp(C.CYN, "  Sending ±45° sine wave at 50 Hz for 20 s — reading WAS from AIO status ...")

    sent_yaw   = []
    actual_yaw = []
    last_send  = time.monotonic()
    last_poll  = time.monotonic()
    t0         = time.monotonic()
    DURATION   = 20.0

    while time.monotonic() - t0 < DURATION:
        now = time.monotonic()
        t   = now - t0
        yaw = 45.0 * math.sin(2 * math.pi * t / 4.0)   # ±45° at 0.25 Hz

        # Send at 50 Hz
        if now - last_send >= 0.02:
            last_send = now
            cdeg = int(yaw * 100)
            data = [(cdeg >> 8) & 0xFF, cdeg & 0xFF,
                    0x00, 0x00,
                    0x01,               # status bit0 = valid
                    0x00, 0x00, 0x00]
            _send(bus, IMU_WAS_CAN_ID, data, extended=False)

        # Poll AIO at 2 Hz
        if now - last_poll >= 0.5:
            last_poll = now
            st  = aio_status(ip)
            ang = float(st["live"]["actualAngle"]) if st and "live" in st else None
            if ang is not None:
                sent_yaw.append(yaw)
                actual_yaw.append(ang)
                print(f"\r  sent={yaw:6.1f}°   WAS={ang:6.2f}°", end="", flush=True)

    bus.shutdown()
    print()

    was_range = (max(actual_yaw) - min(actual_yaw)) if actual_yaw else 0
    status    = "PASS" if was_range > 10.0 else "FAIL"

    print()
    cp(C.BOLD, "  ── Results ──")
    print(f"  Status samples: {len(actual_yaw)}")
    print(f"  WAS range:      {was_range:.1f}°  (need > 10°)")
    cp(C.GRN if status == "PASS" else C.RED, f"  Result: {status}")

    log("T3", f"CAN{port} IMU-WAS",
        [f"samples={len(actual_yaw)} was_range={was_range:.1f}deg", f"result={status}"])


# ═══════════════════════════════════════════════════════════════════════════════
# TEST 4 — CAN steer brand guide
# ═══════════════════════════════════════════════════════════════════════════════
_BRAND_INFO = {
    0: ("Claas",                    True,  False, False),
    1: ("Valtra / Massey / McCormick / MF", True, False, False),
    2: ("CaseIH / New Holland",     True,  False, True ),
    3: ("Fendt",                    True,  True,  True ),
    4: ("JCB",                      True,  False, False),
    5: ("FendtOne",                 False, False, True ),
    6: ("Lindner",                  True,  False, False),
    7: ("AgOpenGPS",                True,  False, False),
    8: ("Cat / Challenger MT Late", True,  False, False),
    9: ("Cat / Challenger MT Early",True,  False, False),
}
# (name, vbus, iso, kbus)

_KNOWN_IDS = {
    0x0CAC1E13: "Claas V_Bus RX (curve)",
    0x0CAD131E: "Claas V_Bus TX",
    0x18EF1CD2: "Claas engage",
    0x0CAC1C13: "Valtra/MF/AGO V_Bus RX",
    0x0CAD131C: "Valtra/MF/AGO V_Bus TX",
    0x18EF1C32: "Valtra engage",
    0x18EF1CFC: "McCormick engage",
    0x18EF1C00: "MF engage",
    0x18FF8306: "McCormick joystick",
    0x0CACAA08: "CaseIH/NH V_Bus RX",
    0x0CAD08AA: "CaseIH/NH V_Bus TX",
    0x14FF7706: "CaseIH K_Bus engage",
    0x18FE4523: "CaseIH rear hitch",
    0x0CEF2CF0: "Fendt V_Bus RX",
    0x0CEFF02C: "Fendt V_Bus TX",
    0x18EF2CF0: "Fendt ISO/K_Bus engage",
    0x0CFFD899: "FendtOne K_Bus engage",
    0x0CACAB13: "JCB V_Bus RX",
    0x0CAD13AB: "JCB V_Bus TX",
    0x18EFAB27: "JCB engage",
    0x0CACF013: "Lindner V_Bus RX",
    0x0CAD13F0: "Lindner V_Bus TX",
    0x0CEFF021: "Lindner engage",
    0x18EF1CF0: "Cat MT Late V_Bus",
    0x1CEFF01C: "Cat MT Late TX",
    0x0CEFFF76: "Cat MT Early V_Bus",
    0x0CEF762C: "Cat MT Early TX",
    0x18EAFFFE: "PVED param read request",
    0x1CEFFF1E: "PVED write / commit",
}


# ── Traffic generator helpers ──────────────────────────────────────────────────
def _curve_le(t, center=32128, amp=8000):
    """Oscillating valve curve, little-endian [lo, hi]."""
    v = int(center + amp * math.sin(t * 0.4)) & 0xFFFF
    return [v & 0xFF, (v >> 8) & 0xFF]

def _curve_be(t, center=32128, amp=8000):
    """Oscillating valve curve, big-endian [hi, lo]."""
    v = int(center + amp * math.sin(t * 0.4)) & 0xFFFF
    return [(v >> 8) & 0xFF, v & 0xFF]

def _fendt_curve(t, amp=8000):
    """Fendt signed delta from center, little-endian [lo, hi]."""
    v = int(amp * math.sin(t * 0.4))
    v16 = v & 0xFFFF
    return [v16 & 0xFF, (v16 >> 8) & 0xFF]

# Brand traffic table: {brand: [(interval_ms, arb_id, extended, data_or_fn), ...]}
# data_or_fn: list of 8 bytes OR callable(t_sec) -> list
_BRAND_TRAFFIC = {
    0: [  # Claas
        (100, 0x0CAC1E13, True,  lambda t: _curve_le(t) + [0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]),
        (500, 0x18EF1CD2, True,  [39, 0, 241, 0, 0, 0, 0, 0]),   # engage MR
    ],
    1: [  # Valtra / Massey / McCormick / MF
        (100, 0x0CAC1C13, True,  lambda t: _curve_le(t) + [0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]),
        (500, 0x18EF1C32, True,  [15, 96, 1, 0, 0, 0, 0, 0]),    # Valtra engage
        (500, 0x18EF1CFC, True,  [15, 96, 0, 255, 0, 0, 0, 0]),  # McCormick engage
        (500, 0x18EF1C00, True,  [15, 96, 1, 0, 0, 0, 0, 0]),    # MF engage
        (500, 0x18FF8306, True,  [0, 0, 0, 0, 0, 8, 0, 0]),      # McCormick joystick
    ],
    2: [  # CaseIH / New Holland
        (100, 0x0CACAA08, True,  lambda t: _curve_le(t) + [0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]),
        (500, 0x14FF7706, True,  [130, 1, 0, 0, 0, 0, 0, 0]),    # K_Bus engage
        (500, 0x18FE4523, True,  [100, 0, 0, 0, 0, 0, 0, 0]),    # K_Bus rear hitch 40%
    ],
    3: [  # Fendt
        (100, 0x0CEF2CF0, True,  lambda t: [5, 10, 3, 10] + _fendt_curve(t) + [0xFF, 0xFF]),
        (500, 0x18EF2CF0, True,  [0x0F, 0x60, 0x01, 0, 0, 0, 0, 0]),  # ISO engage
        (500, 0x18FEBB00, True,  [0, 100, 0, 0, 0, 0, 0, 0]),          # rear hitch
        (500, 0x18EF2CF0, True,  [0x0F, 0x60, 0x01, 0, 0, 0, 0, 0]),  # K_Bus SCR engage
    ],
    4: [  # JCB
        (100, 0x0CACAB13, True,  lambda t: _curve_le(t) + [0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]),
        (500, 0x18EFAB27, True,  [15, 96, 1, 0, 0, 0, 0, 0]),    # engage
    ],
    5: [  # FendtOne
        (500, 0x0CFFD899, True,  [0, 0, 0, 0xF6, 0, 0, 0, 0]),   # K_Bus engage
    ],
    6: [  # Lindner
        (100, 0x0CACF013, True,  lambda t: _curve_le(t) + [0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]),
        (500, 0x0CEFF021, True,  [0, 0, 0, 0, 0, 0, 0, 0]),      # engage
    ],
    7: [  # AgOpenGPS
        (100, 0x0CAC1C13, True,  lambda t: _curve_le(t) + [0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]),
    ],
    8: [  # Cat MT Late
        (100, 0x18EF1CF0, True,  lambda t: [0xF0, 0x20] + _curve_be(t) + [5, 0xFF, 0xFF, 0xFF]),
        (500, 0x18EF1CF0, True,  [0x0F, 0x60, 0x01, 0, 0, 0, 0, 0]),  # engage
    ],
    9: [  # Cat MT Early
        (100, 0x0CEFFF76, True,  lambda t: [0xF0, 0x20] + _curve_be(t) + [5, 0xFF, 0xFF, 0xFF]),
        (500, 0x0CEFFF76, True,  [0x0F, 0x60, 0x01, 0, 0, 0, 0, 0]),  # engage
    ],
}

# Random IDs that look like plausible J1939 tractor background traffic
_RANDOM_POOL = [
    0x18FEF100, 0x18FEF200, 0x18FEDF00, 0x18FEE000,
    0x18FF1234, 0x0CF00400, 0x18F00131, 0x18FED900,
]


def _can_traffic_gen(cfg, brand):
    """Continuously send brand-specific + random CAN traffic until Ctrl+C."""
    header("Test 4b — CAN Traffic Generator")
    iface, channel, bitrate = cfg["interface"], cfg["channel"], cfg["bitrate"]

    msgs = _BRAND_TRAFFIC.get(brand, [])
    name = _BRAND_INFO[brand][0]

    cp(C.GRY,  f"  Brand: {name}")
    cp(C.GRY,   "  Sends brand-specific messages + random J1939 background traffic.")
    cp(C.GRY,   "  Watch the CAN plot / Live status in web UI.  Ctrl+C to stop.")
    print()
    cp(C.YEL,   "  Scheduled messages:")
    for interval_ms, arb_id, ext, _ in msgs:
        desc = _KNOWN_IDS.get(arb_id, "")
        ext_flag = "EXT" if ext else "STD"
        cp(C.GRY, f"    {interval_ms:4d} ms  0x{arb_id:08X} [{ext_flag}]  {desc}")
    cp(C.GRY,   f"   1000 ms  random J1939 background (pool of {len(_RANDOM_POOL)} IDs)")

    bus = open_can(iface, channel, bitrate)
    if bus is None:
        return

    # per-message last-send timestamps
    last_sent  = [time.monotonic() - (i / max(len(msgs), 1)) for i in range(len(msgs))]
    last_rand  = time.monotonic()
    rand_idx   = 0
    t0         = time.monotonic()
    counts     = [0] * len(msgs)
    rand_count = 0

    cp(C.CYN, "  Running — press Ctrl+C to stop ...\n")

    try:
        while True:
            now = time.monotonic()
            t   = now - t0

            # Brand-specific messages
            for i, (interval_ms, arb_id, ext, data_or_fn) in enumerate(msgs):
                if (now - last_sent[i]) * 1000 >= interval_ms:
                    last_sent[i] = now
                    data = data_or_fn(t) if callable(data_or_fn) else list(data_or_fn)
                    if _send(bus, arb_id, data, extended=ext):
                        counts[i] += 1

            # Random background at 1 Hz (cycle through pool)
            if now - last_rand >= 1.0:
                last_rand = now
                rid  = _RANDOM_POOL[rand_idx % len(_RANDOM_POOL)]
                rand_idx += 1
                rdata = [random.randint(0, 255) for _ in range(8)]
                _send(bus, rid, rdata, extended=True)
                rand_count += 1

            # Status line every second
            total = sum(counts)
            elapsed = int(now - t0)
            curve_v = int(32128 + 8000 * math.sin(t * 0.4))
            print(f"\r  [{elapsed:4d}s]  brand msgs={total:5d}  rand={rand_count:4d}"
                  f"  curve=0x{curve_v:04X} ({curve_v})", end="", flush=True)

            time.sleep(0.005)

    except KeyboardInterrupt:
        pass

    bus.shutdown()
    elapsed = time.monotonic() - t0
    print()
    print()
    cp(C.BOLD, "  ── Traffic Summary ──")
    print(f"  Duration: {elapsed:.1f} s")
    for i, (interval_ms, arb_id, _, _) in enumerate(msgs):
        desc = _KNOWN_IDS.get(arb_id, "")
        print(f"  0x{arb_id:08X}  sent={counts[i]:5d}  ({desc})")
    print(f"  Random background: {rand_count} msgs")

    log("T4b", f"CAN traffic gen brand={brand} {name}",
        [f"duration={elapsed:.1f}s total_brand={sum(counts)} total_rand={rand_count}"] +
        [f"0x{arb_id:08X} n={counts[i]}" for i, (_, arb_id, _, _) in enumerate(msgs)])


def test_can_steer_guide(cfg):
    header("Test 4 — CAN Steer Brand Guide")
    ip = cfg["ip"]

    print()
    print("  ID  Brand")
    print("  ─" * 35)
    for bid, (name, vbus, iso, kbus) in _BRAND_INFO.items():
        buses = ", ".join(b for b, on in [("V_Bus", vbus), ("ISO", iso), ("K_Bus", kbus)] if on)
        print(f"  {bid:2d}  {name:38s}  [{buses}]")

    print()
    while True:
        try:
            brand = int(input("  Select brand (0-9): ").strip())
            if 0 <= brand <= 9:
                break
        except ValueError:
            pass
        cp(C.RED, "  Enter a number 0-9")

    name, vbus, iso, kbus = _BRAND_INFO[brand]
    cp(C.BOLD, f"\n  Brand {brand}: {name}")

    print()
    cp(C.YEL, "  Required CAN bus connections:")
    if vbus:
        cp(C.GRY, "    V_Bus  — steering valve TX/RX (mode 3, 250 kbps)")
    if iso:
        cp(C.GRY, "    ISO    — engage + hitch + PVED  (mode 5, 250 kbps)")
    if kbus:
        cp(C.GRY, "    K_Bus  — tractor engage bus     (mode 4, 250 or 500 kbps)")

    print()
    cp(C.YEL, "  Web UI setup steps:")
    cp(C.GRY, f"    1. Open  http://{ip}  in your browser")
    cp(C.GRY,  "    2. CAN port assignment: assign each port to its function above")
    cp(C.GRY,  "    3. WAS source: choose 'V_Bus (tractor valve estCurve)'")
    cp(C.GRY,  "    4. Steer brand: select the matching brand")
    cp(C.GRY,  "    5. Save and restart Teensy")

    # PVED test (brand 3 = Fendt with ISO valve)
    if iso and brand == 3:
        print()
        if ask_yn("Run PVED (Danfoss valve) read test?"):
            _pved_test(ip)

    # CAN scan
    print()
    if ask_yn("Run 5-second CAN scan to identify tractor messages?"):
        _can_scan(ip)

    # Traffic generator
    print()
    if ask_yn("Run CAN traffic generator (send brand msgs + random until Ctrl+C)?"):
        _can_traffic_gen(cfg, brand)

    log("T4", f"Brand {brand} {name}", ["guide completed"])


def _pved_test(ip):
    cp(C.YEL, "\n  Triggering PVED param 64007 read via API ...")
    r = _aio_get(ip, "/api/pved?cmd=read64007")
    if r is None:
        cp(C.RED, "  API call failed — check AIO connection")
        return

    cp(C.GRY, "  Waiting 2 s for valve response ...")
    time.sleep(2.0)

    st = aio_status(ip)
    if not st:
        cp(C.RED, "  Cannot read AIO status")
        return

    pved     = st.get("pved", {})
    detected = pved.get("detected", False)
    last_val = pved.get("last64007", 0xFFFF)
    factory  = pved.get("factory64007", 0xFFFF)

    if detected:
        cp(C.GRN, f"  PVED detected!  param 64007 = 0x{last_val:04X}", end="")
        if factory != 0xFFFF:
            print(f"  (factory = 0x{factory:04X})")
        else:
            print()
        if last_val == 0x1E:
            cp(C.GRN, "  Address already set to AIO (0x1E) — OK")
        else:
            cp(C.YEL, "  Address not AIO (0x1E) — use 'Write param 64007' in web UI to program it")
    else:
        cp(C.RED, "  PVED not detected — no response received")
        cp(C.GRY, "  Check: ISO_Bus wired? PVED powered? Correct CAN baud rate?")

    log("T4-PVED", "PVED read",
        [f"detected={detected} last64007=0x{last_val:04X} factory=0x{factory:04X}"])


def _can_scan(ip):
    cp(C.YEL, "\n  Starting CAN scan ...")
    if _aio_get(ip, "/api/canscan?cmd=start") is None:
        cp(C.RED, "  Scan start failed — AIO not reachable")
        return

    for i in range(5, 0, -1):
        print(f"\r  Scanning ... {i} s remaining", end="", flush=True)
        time.sleep(1)
    print()

    _aio_get(ip, "/api/canscan?cmd=stop")
    time.sleep(0.3)
    r = _aio_get(ip, "/api/canscan?cmd=data")
    if r is None:
        cp(C.RED, "  Scan data fetch failed")
        return

    try:
        entries = r.json().get("entries", [])
    except Exception:
        cp(C.RED, "  Could not parse scan response")
        return

    if not entries:
        cp(C.YEL, "  No CAN messages captured during scan")
        return

    known_count = 0
    print()
    cp(C.BOLD, f"  {len(entries)} unique CAN IDs found:")

    known_lines   = []
    unknown_lines = []
    for e in entries:
        cid   = e.get("id", 0)
        bus   = e.get("bus", "?")
        count = e.get("count", 0)
        desc  = _KNOWN_IDS.get(cid, "")
        line  = f"    [{bus}] 0x{cid:08X}  ×{count:4d}"
        if desc:
            known_count += 1
            known_lines.append((line, desc))
        else:
            unknown_lines.append(line)

    if known_lines:
        cp(C.BOLD, "\n  Known IDs:")
        for line, desc in known_lines:
            cp(C.GRN, f"{line}   {desc}")

    if unknown_lines:
        cp(C.BOLD, "\n  Unknown IDs:")
        for line in unknown_lines:
            cp(C.GRY, line)

    print()
    cp(C.CYN, f"  Summary: {known_count} known / {len(entries)} total")

    log("T4-scan", "CAN scan",
        [f"total={len(entries)} known={known_count}"] +
        [f"0x{e.get('id', 0):08X} [{e.get('bus', '?')}] ×{e.get('count', 0)}"
         f"  {_KNOWN_IDS.get(e.get('id', 0), '')}" for e in entries])


# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="AIO v4 CAN Test Suite")
    parser.add_argument("--ip",        default=DEFAULT_AIO_IP,   help="AIO IP address")
    parser.add_argument("--interface", default=DEFAULT_IFACE,    help="python-can interface (pcan, socketcan, vector, ...)")
    parser.add_argument("--channel",   default=DEFAULT_CHANNEL,  help="CAN channel (PCAN_USBBUS1, can0, COM3, ...)")
    parser.add_argument("--bitrate",   default=DEFAULT_BITRATE,  type=int, help="CAN bitrate in bps")
    args = parser.parse_args()

    cfg = {"ip": args.ip, "interface": args.interface,
           "channel": args.channel, "bitrate": args.bitrate}

    # Enable ANSI on Windows cmd
    if sys.platform == "win32":
        os.system("")

    cp(C.BOLD + C.CYN, "\n  AIO v4 CAN Test Suite")
    cp(C.GRY,          f"  AIO:  {cfg['ip']}")
    cp(C.GRY,          f"  CAN:  {cfg['interface']}/{cfg['channel']}  {cfg['bitrate'] // 1000} kbps")
    cp(C.GRY,          f"  Log:  {_LOG_FILE}\n")

    st = aio_status(cfg["ip"])
    if st is None:
        cp(C.YEL, f"  AIO not reachable at {cfg['ip']} — API-dependent steps will prompt manually")
    else:
        cp(C.GRN, "  AIO online")

    TESTS = [
        ("CAN Loopback  (CANtest mode)",      test_cantest_loopback),
        ("Keya Motor Emulation",              test_keya_emulation),
        ("IMU as WAS",                        test_imu_was),
        ("CAN Steer Brand Guide + PVED scan", test_can_steer_guide),
    ]

    while True:
        print()
        cp(C.BOLD, "  ── Menu ──")
        for i, (name, _) in enumerate(TESTS, 1):
            print(f"  {i}. {name}")
        print(f"  {len(TESTS) + 1}. Exit")
        print()

        choice = input("  Select: ").strip()
        if choice in (str(len(TESTS) + 1), "q", "exit", "quit"):
            break
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(TESTS):
                TESTS[idx][1](cfg)
            else:
                cp(C.RED, f"  Enter 1–{len(TESTS) + 1}")
        except (ValueError, KeyboardInterrupt):
            cp(C.YEL, "\n  Interrupted")

    print()
    cp(C.GRY, f"  Results saved: {_LOG_FILE}")
    cp(C.CYN,  "  Done.")


if __name__ == "__main__":
    main()
