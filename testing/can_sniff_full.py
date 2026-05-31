#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pun CAN sniffer s faznim markerima
===================================
Snima SAV promet (bez filtriranja), bilježi raw .log i sazetak po PGN/SA.
Operater pritisne ENTER za prelazak u sljedecu fazu eksperimenta.

Pokretanje: python can_sniff_full.py
"""

import sys
import os
import time
import threading
from collections import Counter, defaultdict
from datetime import datetime

# === KONFIGURACIJA ===
PCAN_CHANNEL = "PCAN_USBBUS1"
BITRATE      = 250_000          # promijeni na 500_000 ako treba
DURATION_MAX = 300              # max snimanja u sekundama (5 min)


def j1939_decode(arb_id):
    priority = (arb_id >> 26) & 0x7
    pf       = (arb_id >> 16) & 0xFF
    ps       = (arb_id >>  8) & 0xFF
    sa       =  arb_id        & 0xFF
    if pf < 240:
        pgn = pf << 8
    else:
        pgn = (pf << 8) | ps
    return priority, pgn, sa


def have_python_can():
    try:
        import can  # noqa
        return True
    except ImportError:
        return False


def main():
    print("=" * 64)
    print("  Pun CAN sniffer - identifikacija steering")
    print("=" * 64)
    print()
    print(f"Brzina: {BITRATE} bps")
    print(f"Kanal:  {PCAN_CHANNEL}")
    print()
    print("FAZE EKSPERIMENTA - pritisni ENTER za prelazak u sljedecu:")
    print("  1. Mirno (motor radi, niko ne dira nista)        -> 10-15s")
    print("  2. Volan okreni lijevo do kraja                  -> 5s")
    print("  3. Volan okreni desno do kraja                   -> 5s")
    print("  4. Volan na centar                               -> 5s")
    print("  5. Ugasi i upali kontakt (force Address Claim)   -> 15s")
    print("  6. Pritisni autosteer/engage gumb (ako postoji)  -> 10s")
    print("  7. VRC ON, blagi pokreti volana                  -> 15s")
    print("  KRAJ - pritisni ENTER zadnji put za stop")
    print()
    input("Spoji PCAN na pin, pa pritisni ENTER za start...")

    if not have_python_can():
        print("FALI python-can. Instaliraj: pip install python-can")
        sys.exit(1)

    import can

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path     = os.path.abspath(f"sniff_{ts}.log")
    summary_path = os.path.abspath(f"sniff_{ts}_summary.txt")
    phases_path  = os.path.abspath(f"sniff_{ts}_phases.txt")

    print(f"\nLog:     {log_path}")
    print(f"Sazetak: {summary_path}")
    print(f"Faze:    {phases_path}\n")

    try:
        bus = can.Bus(interface="pcan", channel=PCAN_CHANNEL,
                      bitrate=BITRATE, receive_own_messages=False)
    except Exception as e:
        print(f"GRESKA: ne mogu otvoriti CAN: {e}")
        input("ENTER za izlaz...")
        sys.exit(1)

    print("Sniffam... (faza 1)")

    # statistika
    id_counter   = Counter()
    pgn_counter  = Counter()
    sa_counter   = Counter()
    pgn_examples = {}            # PGN -> zadnji primljeni payload (hex)
    pgn_by_phase = defaultdict(set)   # phase -> set of PGN-ova
    id_by_phase  = defaultdict(set)
    total        = 0
    start_time   = time.time()
    current_phase = 1
    phase_start   = start_time

    phases_log = []

    # flag za prelazak u sljedecu fazu (postavlja se iz threada)
    advance_phase = threading.Event()
    stop_sniff    = threading.Event()

    def keyboard_listener():
        while not stop_sniff.is_set():
            try:
                input()           # blokira dok user ne pritisne ENTER
                advance_phase.set()
            except (EOFError, KeyboardInterrupt):
                stop_sniff.set()
                break

    kb_thread = threading.Thread(target=keyboard_listener, daemon=True)
    kb_thread.start()

    log_fp = open(log_path, "w", encoding="utf-8")
    log_fp.write(f"# CAN sniff {datetime.now()}, bitrate {BITRATE}\n")
    log_fp.write(f"# Phase 1 start\n")

    try:
        while not stop_sniff.is_set() and (time.time() - start_time) < DURATION_MAX:
            # prelazak u sljedecu fazu?
            if advance_phase.is_set():
                advance_phase.clear()
                phase_elapsed = time.time() - phase_start
                phases_log.append({
                    "phase":    current_phase,
                    "duration": phase_elapsed,
                    "pgns":     sorted(pgn_by_phase[current_phase]),
                    "ids":      sorted(id_by_phase[current_phase]),
                })
                current_phase += 1
                phase_start = time.time()
                log_fp.write(f"# === Phase {current_phase} start @ {phase_start - start_time:.1f}s ===\n")
                print(f"  >>> Faza {current_phase} (prethodna: {phase_elapsed:.1f}s, "
                      f"{len(pgn_by_phase[current_phase-1])} PGN-ova)")
                if current_phase > 7:
                    print("Kraj zadnje faze. Pritisni ENTER za STOP.")

            msg = bus.recv(timeout=0.2)
            if msg is None:
                continue

            total += 1
            id_counter[msg.arbitration_id] += 1
            data_hex = msg.data.hex().upper()
            t_rel = msg.timestamp - start_time if msg.timestamp > start_time else (time.time() - start_time)

            log_fp.write(f"({msg.timestamp:.6f}) bus "
                         f"{msg.arbitration_id:08X}"
                         f"{'X' if msg.is_extended_id else ''}"
                         f"#{data_hex} p{current_phase}\n")

            if msg.is_extended_id:
                _, pgn, sa = j1939_decode(msg.arbitration_id)
                pgn_counter[pgn] += 1
                sa_counter[sa]   += 1
                pgn_by_phase[current_phase].add(pgn)
                id_by_phase[current_phase].add(msg.arbitration_id)
                if pgn not in pgn_examples:
                    pgn_examples[pgn] = (msg.arbitration_id, data_hex)

            # status na ekran svakih 5s
            if total % 500 == 0:
                elapsed = time.time() - start_time
                print(f"  ... {total} okvira, "
                      f"{len(id_counter)} ID, "
                      f"{len(pgn_counter)} PGN, "
                      f"faza {current_phase} ({elapsed:.0f}s)")

    except KeyboardInterrupt:
        print("\nPrekid s Ctrl+C")
    finally:
        # zadnja faza
        phase_elapsed = time.time() - phase_start
        phases_log.append({
            "phase":    current_phase,
            "duration": phase_elapsed,
            "pgns":     sorted(pgn_by_phase[current_phase]),
            "ids":      sorted(id_by_phase[current_phase]),
        })
        stop_sniff.set()
        log_fp.close()
        try:
            bus.shutdown()
        except Exception:
            pass

    # ===== SAZETAK =====
    print(f"\nGotovo. Ukupno {total} okvira, {len(id_counter)} ID, {len(pgn_counter)} PGN.\n")

    with open(summary_path, "w", encoding="utf-8") as f:
        f.write(f"CAN sniff sazetak  {datetime.now()}\n")
        f.write(f"Brzina: {BITRATE} bps\n")
        f.write(f"Trajanje: {time.time()-start_time:.1f}s\n")
        f.write(f"Ukupno: {total} okvira, {len(id_counter)} jedinstvenih ID, {len(pgn_counter)} PGN-ova\n\n")

        f.write("=" * 50 + "\n")
        f.write("TOP 30 SOURCE ADDRESSES (J1939 SA)\n")
        f.write("=" * 50 + "\n")
        for sa, n in sa_counter.most_common(30):
            hint = ""
            if sa == 0x00: hint = " (motor)"
            elif sa == 0x03: hint = " (transmisija)"
            elif sa == 0x05: hint = " (steering controller, std J1939)"
            elif sa == 0x0B: hint = " (brakes)"
            elif sa == 0x13: hint = " (moguce Danfoss PVED)"
            elif sa == 0x82: hint = " (moguce Danfoss PVED)"
            elif sa == 0xE5: hint = " (moguce Danfoss PVED)"
            elif sa == 0xF0: hint = " (fleet mgmt)"
            elif sa == 0xF9: hint = " (off-board diag tool)"
            elif 0x80 <= sa <= 0x8F: hint = " (ISOBUS slot range)"
            elif 0x9C <= sa <= 0x9F: hint = " (navigation - guidance)"
            f.write(f"  SA 0x{sa:02X} ({sa:3d})  x{n:6d}{hint}\n")

        f.write("\n" + "=" * 50 + "\n")
        f.write("TOP 40 PGN-OVA\n")
        f.write("=" * 50 + "\n")
        for pgn, n in pgn_counter.most_common(40):
            hint = ""
            if pgn == 0xAC00: hint = " *** GUIDANCE MACHINE STATUS ***"
            elif pgn == 0xAD00: hint = " *** GUIDANCE SYSTEM COMMAND ***"
            elif pgn == 0xEE00: hint = " (Address Claim)"
            elif pgn == 0xEA00: hint = " (Request)"
            elif pgn == 0xF004: hint = " (Engine Controller 1)"
            elif pgn == 0xFEF1: hint = " (Cruise/Vehicle Speed)"
            elif pgn == 0xFE48: hint = " (Wheel Speed)"
            elif pgn == 0xFEEE: hint = " (Engine Temperature)"
            elif 0xFF00 <= pgn <= 0xFFFF: hint = " (proprietary)"
            example_id, example_data = pgn_examples.get(pgn, (0, ""))
            f.write(f"  PGN 0x{pgn:04X} ({pgn:5d})  x{n:6d}{hint}\n")
            f.write(f"          primjer: ID 0x{example_id:08X}  data {example_data}\n")

        f.write("\n" + "=" * 50 + "\n")
        f.write("SVI CAN ID-OVI (top 50 po broju)\n")
        f.write("=" * 50 + "\n")
        for cid, n in id_counter.most_common(50):
            f.write(f"  0x{cid:08X}  x{n}\n")

    # ===== PHASE COMPARISON =====
    with open(phases_path, "w", encoding="utf-8") as f:
        f.write(f"Faze eksperimenta  {datetime.now()}\n")
        f.write(f"Bitrate: {BITRATE} bps\n\n")

        for p in phases_log:
            f.write(f"--- FAZA {p['phase']}  ({p['duration']:.1f}s)  "
                    f"{len(p['pgns'])} PGN-ova, {len(p['ids'])} ID-ova ---\n")
            for pgn in p['pgns']:
                f.write(f"  PGN 0x{pgn:04X}\n")
            f.write("\n")

        # razlika faza
        f.write("=" * 50 + "\n")
        f.write("RAZLIKE IZMEDU FAZA (novi PGN-ovi u fazi)\n")
        f.write("=" * 50 + "\n")
        seen_pgns = set()
        for p in phases_log:
            new_pgns = set(p['pgns']) - seen_pgns
            if new_pgns:
                f.write(f"\nFaza {p['phase']}: novi PGN-ovi koji se ranije nisu vidjeli:\n")
                for pgn in sorted(new_pgns):
                    f.write(f"  PGN 0x{pgn:04X}\n")
            seen_pgns |= set(p['pgns'])

        # ID-ovi koji su PROMIJENILI PAYLOAD izmedu faza
        # (to bi pokazalo na steering komande koje reagiraju na volan)
        # Za to bi trebao history - ova verzija samo ima zadnji payload
        # ali raw .log ima sve, mozemo to naknadno analizirati

    print(f"Snimljeno:")
    print(f"  Raw log:  {log_path}")
    print(f"  Sazetak:  {summary_path}")
    print(f"  Faze:     {phases_path}")
    print(f"\nPosalji sva tri fajla Nikoli.")
    input("\nENTER za izlaz...")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\nGRESKA: {e}")
        import traceback
        traceback.print_exc()
        input("\nENTER za izlaz...")