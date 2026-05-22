#!/usr/bin/env python3
"""
Tractor CAN Bus Scanner
Snima 30 s CAN prometa, sprema sve u log file.
U konzoli odmah ističe poruke koje su vezane uz upravljanje (steer-ready PGN-ovi).

Koristi se za pronalaženje CAN pinova na traktoru — priključi adapter na
potencijalne CAN pinove i pokreni skriptu. Ako vidiš poznate steer ID-ove,
pravi si na pravoj sabirnici.

Requirements:
    pip install python-can

Usage:
    python can_scan_tractor.py
    python can_scan_tractor.py --interface pcan --channel PCAN_USBBUS1 --bitrate 250000
    python can_scan_tractor.py --duration 60
"""

import sys
import os
import math
import time
import argparse
import datetime

try:
    import can
except ImportError:
    print("ERROR: python-can nije instaliran. Pokreni:  pip install python-can")
    sys.exit(1)

# ── ANSI boje ──────────────────────────────────────────────────────────────────
class C:
    RST  = "\033[0m"
    BOLD = "\033[1m"
    GRN  = "\033[92m"
    RED  = "\033[91m"
    YEL  = "\033[93m"
    CYN  = "\033[96m"
    GRY  = "\033[90m"
    MAG  = "\033[95m"

def cp(color, text, end="\n"):
    print(f"{color}{text}{C.RST}", end=end, flush=True)

# ── Poznati steer ID-ovi (brand-specific) ─────────────────────────────────────
STEER_IDS = {
    # Claas
    0x0CAC1E13: "Claas  V_Bus RX — valve curve + state",
    0x0CAD131E: "Claas  V_Bus TX — steer command",
    0x18EF1CD2: "Claas  engage signal",
    # Valtra / Massey / McCormick / MF
    0x0CAC1C13: "Valtra/MF/AGO  V_Bus RX — valve curve",
    0x0CAD131C: "Valtra/MF/AGO  V_Bus TX — steer command",
    0x18EF1C32: "Valtra  engage",
    0x18EF1CFC: "McCormick  engage",
    0x18EF1C00: "MF  engage",
    0x18FF8306: "McCormick  joystick engage",
    # CaseIH / New Holland
    0x0CACAA08: "CaseIH/NH  V_Bus RX — valve curve",
    0x0CAD08AA: "CaseIH/NH  V_Bus TX — steer command",
    0x14FF7706: "CaseIH  K_Bus engage",
    0x18FE4523: "CaseIH  K_Bus rear hitch",
    # Fendt
    0x0CEF2CF0: "Fendt  V_Bus RX — valve curve",
    0x0CEFF02C: "Fendt  V_Bus TX — steer command",
    0x18EF2CF0: "Fendt  ISO/K_Bus engage",
    # FendtOne
    0x0CFFD899: "FendtOne  K_Bus engage",
    # JCB
    0x0CACAB13: "JCB  V_Bus RX — valve curve",
    0x0CAD13AB: "JCB  V_Bus TX — steer command",
    0x18EFAB27: "JCB  engage",
    # Lindner
    0x0CACF013: "Lindner  V_Bus RX — valve curve",
    0x0CAD13F0: "Lindner  V_Bus TX — steer command",
    0x0CEFF021: "Lindner  engage",
    # Cat / Challenger
    0x18EF1CF0: "Cat MT Late  V_Bus (curve + engage)",
    0x1CEFF01C: "Cat MT Late  V_Bus TX",
    0x0CEFFF76: "Cat MT Early  V_Bus (curve + engage)",
    0x0CEF762C: "Cat MT Early  V_Bus TX",
    # PVED (Danfoss electrohydraulic valve)
    0x18EAFFFE: "PVED  param read request",
    0x1CEFFF1E: "PVED  param write / commit",
    # Rear hitch (generic J1939 PGN 65093 = 0xFEBB, any source)
    # — matched via PGN extraction below, not exact ID
}

# ── J1939 PGN-ovi vezani uz upravljanje / hidrauliku ──────────────────────────
# PGN = bits 17-8 of 29-bit ID (za PDU2, PF>=0xF0) ili (PF<<8) za PDU1
STEER_PGNS = {
    0x00AC00: "PGN 0xAC** — steer valve command (Claas/Valtra family)",
    0x00EF00: "PGN 0xEF** — proprietary peer-to-peer (engage / PVED)",
    0x00FF00: "PGN 0xFF** — proprietary broadcast (hitch, joystick...)",
    0xFEBB00: "PGN 65211 / 0xFEBB — rear hitch position",
    0xFEC100: "PGN 65217 / 0xFEC1 — front hitch / implement",
    0xFEBF00: "PGN 65215 / 0xFEBF — steering wheel angle sensor",
    0xF00900: "PGN 61449 / 0xF009 — vehicle direction + steering angle",
    0xF00500: "PGN 61445 / 0xF005 — transmission / ground speed",
    0xFF1300: "PGN 65299 / 0xFF13 — GPS position broadcast (J1939)",
}


def extract_pgn(arb_id):
    """Extract J1939 PGN from 29-bit extended CAN ID."""
    pf  = (arb_id >> 16) & 0xFF
    ps  = (arb_id >>  8) & 0xFF
    dp  = (arb_id >> 24) & 0x01
    if pf < 0xF0:
        return (dp << 16) | (pf << 8)          # PDU1: PS is destination address
    else:
        return (dp << 16) | (pf << 8) | ps     # PDU2: PS is group extension


def pgn_label(arb_id):
    """Return description if PGN matches a known steer-related PGN, else ''."""
    pgn = extract_pgn(arb_id)
    # Exact match
    if pgn in STEER_PGNS:
        return STEER_PGNS[pgn]
    # Wildcard match on upper byte (PF family)
    pf_key = pgn & 0xFF00
    if pf_key in STEER_PGNS:
        return STEER_PGNS[pf_key]
    return ""


def classify(arb_id):
    """Return (label, is_steer) for a CAN ID."""
    if arb_id in STEER_IDS:
        return STEER_IDS[arb_id], True
    lbl = pgn_label(arb_id)
    if lbl:
        return lbl, True
    return "", False


# ── Formatiranje ───────────────────────────────────────────────────────────────
def fmt_data(data, maxbytes=8):
    return " ".join(f"{b:02X}" for b in data[:maxbytes])

def fmt_id(arb_id, extended):
    if extended:
        return f"0x{arb_id:08X}"
    return f"0x{arb_id:04X}     "   # pad to same width


# ── Glavni scan ────────────────────────────────────────────────────────────────
def run_scan(iface, channel, bitrate, duration, log_path):
    try:
        bus = can.interface.Bus(interface=iface, channel=channel, bitrate=bitrate)
        cp(C.GRN, f"  CAN otvoren: {iface}/{channel} @ {bitrate // 1000} kbps")
    except Exception as e:
        cp(C.RED, f"  Ne mogu otvoriti CAN bus: {e}")
        sys.exit(1)

    cp(C.YEL, f"  Snimljem {duration} sekundi ...  (log: {log_path})")
    cp(C.GRY,  "  Zeleno = poznati steer ID / PGN    Sivo = ostalo")
    print()

    # Zaglavlje u konzoli
    cp(C.BOLD, f"  {'Vrijeme':>8}  {'ID':12}  {'DLC'}  {'Podaci (hex)':24}  Opis")
    cp(C.GRY,   "  " + "─" * 80)

    t0         = time.monotonic()
    total      = 0
    unique     = {}      # arb_id -> {count, first_ts, last_data, label, is_steer}
    steer_found = {}     # arb_id -> label (samo steer ID-ovi)

    with open(log_path, "w", encoding="utf-8") as f:
        # Log zaglavlje
        f.write(f"# Tractor CAN Scan  {datetime.datetime.now():%Y-%m-%d %H:%M:%S}\n")
        f.write(f"# Interface: {iface}/{channel}  Bitrate: {bitrate}\n")
        f.write(f"# Duration: {duration} s\n")
        f.write(f"# Format: elapsed_ms, ID (hex), EXT, DLC, data_hex, description\n#\n")

        deadline = time.monotonic() + duration

        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            msg = bus.recv(timeout=min(remaining, 0.1))
            if msg is None:
                continue

            elapsed_ms = int((time.monotonic() - t0) * 1000)
            total += 1
            arb_id = msg.arbitration_id
            data   = bytes(msg.data)
            label, is_steer = classify(arb_id)

            # Log file — sve poruke
            id_str   = f"0x{arb_id:08X}" if msg.is_extended_id else f"0x{arb_id:04X}"
            ext_flag = "EXT" if msg.is_extended_id else "STD"
            data_hex = fmt_data(data)
            f.write(f"{elapsed_ms:8d}, {id_str}, {ext_flag}, {msg.dlc}, {data_hex:<24}, {label}\n")

            # Konzola — ispiši samo kad se pojavi novi ID ili steer ID
            if arb_id not in unique:
                unique[arb_id] = {"count": 0, "data": data, "label": label, "steer": is_steer}
                t_str = f"{elapsed_ms / 1000:7.2f}s"
                line  = f"  {t_str}  {fmt_id(arb_id, msg.is_extended_id)}  {msg.dlc}  {data_hex:<24}"
                if is_steer:
                    steer_found[arb_id] = label
                    cp(C.GRN + C.BOLD, line + f"  *** {label}")
                else:
                    cp(C.GRY, line)

            entry = unique[arb_id]
            entry["count"] += 1
            entry["data"]   = data

            # Progress svake 5 sekunde
            elapsed = time.monotonic() - t0
            if int(elapsed) % 5 == 0 and int(elapsed) > 0:
                pct = elapsed / duration * 100
                print(f"\r  [{elapsed:4.0f}s / {duration}s  {pct:.0f}%]"
                      f"  primljeno={total:6d}  jedinstvenih={len(unique):4d}"
                      f"  steer={len(steer_found):2d}",
                      end="", flush=True)

    bus.shutdown()
    print()

    # ── Sažetak u konzoli ──────────────────────────────────────────────────────
    print()
    cp(C.BOLD + C.CYN, "  ── Rezultati skeniranja ──")
    print(f"  Trajanje:        {duration} s")
    print(f"  Ukupno poruka:   {total}")
    print(f"  Jedinstvenih ID: {len(unique)}")
    print(f"  Steer ID-ovi:    {len(steer_found)}")

    if steer_found:
        print()
        cp(C.BOLD + C.GRN, "  ── Pronađeni steer PGN-ovi / ID-ovi ──")
        cp(C.GRY, f"  {'ID':12}  {'Broj':>6}  {'Zadnji podaci':24}  Opis")
        cp(C.GRY,  "  " + "─" * 78)
        # Sortiraj po broju poruka (najfrekventniji prvi)
        for arb_id in sorted(steer_found, key=lambda x: unique[x]["count"], reverse=True):
            entry    = unique[arb_id]
            id_str   = f"0x{arb_id:08X}"
            data_hex = fmt_data(entry["data"])
            cp(C.GRN,
               f"  {id_str:12}  {entry['count']:6d}  {data_hex:<24}  {steer_found[arb_id]}")
        print()
        cp(C.BOLD + C.YEL, "  Savjet: Priključi adapter na iste pinove i provjeri brand u AIO web UI.")
    else:
        print()
        cp(C.YEL, "  Nisu pronađeni poznati steer ID-ovi.")
        cp(C.GRY,  "  Moguće: pogrešna sabirnica, pogrešan baud rate (pokušaj 500k),")
        cp(C.GRY,  "  ili traktor ima nestandardni protokol.")

    # ── Sve jedinstvene ID-ove u log file ─────────────────────────────────────
    with open(log_path, "a", encoding="utf-8") as f:
        f.write("\n# ── Sažetak jedinstvenih ID-ova ──\n")
        f.write(f"# {'ID':12}  {'Broj':>6}  {'Steer':5}  Opis\n")
        for arb_id in sorted(unique, key=lambda x: unique[x]["count"], reverse=True):
            e      = unique[arb_id]
            id_str = f"0x{arb_id:08X}"
            steer  = "DA" if e["steer"] else "-"
            f.write(f"  {id_str:12}  {e['count']:6d}  {steer:5}  {e['label']}\n")

    cp(C.GRY, f"\n  Kompletan log: {log_path}")


# ── Entry point ────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Tractor CAN Bus Scanner")
    parser.add_argument("--interface", default="pcan",         help="python-can interface")
    parser.add_argument("--channel",   default="PCAN_USBBUS1", help="CAN kanal")
    parser.add_argument("--bitrate",   default=250000, type=int, help="Baud rate (250000 / 500000)")
    parser.add_argument("--duration",  default=30,     type=int, help="Trajanje skeniranja u sekundama")
    args = parser.parse_args()

    if sys.platform == "win32":
        os.system("")  # ANSI na Windows cmd

    log_dir  = os.path.dirname(os.path.abspath(__file__))
    log_path = os.path.join(log_dir, f"tractor_scan_{datetime.datetime.now():%Y%m%d_%H%M%S}.log")

    print()
    cp(C.BOLD + C.CYN, "  Tractor CAN Bus Scanner")
    cp(C.GRY,          f"  {args.interface}/{args.channel}  {args.bitrate // 1000} kbps  {args.duration} s")
    print()

    run_scan(args.interface, args.channel, args.bitrate, args.duration, log_path)


if __name__ == "__main__":
    main()
