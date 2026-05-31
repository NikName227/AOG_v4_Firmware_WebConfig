#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAN sniffer - eksperiment volana
=================================
Faze se prikazuju jedna po jedna, velikim slovima.
Operater pritisne ENTER za prelazak na sljedecu fazu.
"""

import sys
import os
import time
import threading
from collections import Counter, defaultdict
from datetime import datetime

PCAN_CHANNEL = "PCAN_USBBUS1"
BITRATE      = 250_000


# Definicija faza - lako za izmjeniti
PHASES = [
    ("MIRNO - VOLAN NA CENTRU",            "Ne diraj nista. Cekaj."),
    ("OKRENI VOLAN POLAKO LIJEVO DO KRAJA","Polako okreci u lijevu stranu."),
    ("DRZI VOLAN LIJEVO",                  "Volan ostaje na lijevom end-stopu."),
    ("POLAKO VRATI VOLAN NA CENTAR",       "Polako vrati u sredinu."),
    ("OKRENI VOLAN POLAKO DESNO DO KRAJA", "Polako okreci u desnu stranu."),
    ("DRZI VOLAN DESNO",                   "Volan ostaje na desnom end-stopu."),
    ("POLAKO VRATI VOLAN NA CENTAR",       "Polako vrati u sredinu."),
    ("PRITISNI VRC GUMB 5 PUTA",           "Srednji narancasti gumb, jednom svakih 1-2s."),
    ("KRAJ",                               "Pritisni ENTER za izlaz."),
]


def clear_screen():
    """Ocisti ekran."""
    os.system('cls' if os.name == 'nt' else 'clear')


def show_phase_banner(phase_num, total_phases, title, instruction, msg_count):
    """Veliki banner s instrukcijama trenutne faze."""
    clear_screen()
    print()
    print("=" * 70)
    print(f"   FAZA {phase_num} / {total_phases}")
    print("=" * 70)
    print()
    print()
    print(f"   >>>  {title}  <<<")
    print()
    print()
    print(f"   {instruction}")
    print()
    print()
    print("-" * 70)
    print(f"   Snimljeno do sad: {msg_count} poruka")
    print()
    print("   ==> KAD SI GOTOV/A SA OVOM FAZOM, PRITISNI ENTER <==")
    print()
    print("=" * 70)


def j1939_decode(arb_id):
    pf = (arb_id >> 16) & 0xFF
    ps = (arb_id >>  8) & 0xFF
    sa =  arb_id        & 0xFF
    if pf < 240:
        pgn = pf << 8
    else:
        pgn = (pf << 8) | ps
    return pgn, sa


def main():
    clear_screen()
    print("=" * 70)
    print("   CAN sniffer - eksperiment volana (PVED-CLS identifikacija)")
    print("=" * 70)
    print()
    print(f"   Brzina:  {BITRATE} bps")
    print(f"   Kanal:   {PCAN_CHANNEL}")
    print(f"   Faza ukupno: {len(PHASES)}")
    print()
    print("   FAZE EKSPERIMENTA:")
    for i, (title, _) in enumerate(PHASES, 1):
        print(f"      {i}. {title}")
    print()
    print("=" * 70)
    print()
    input("   Spoji PCAN na pin 1/3, pritisni ENTER za start sniffa...")

    try:
        import can
    except ImportError:
        print("\nFALI python-can. Instaliraj: pip install python-can")
        sys.exit(1)

    # Otvori CAN
    try:
        bus = can.Bus(interface="pcan", channel=PCAN_CHANNEL,
                      bitrate=BITRATE, receive_own_messages=False)
    except Exception as e:
        print(f"\nGRESKA pri otvaranju CAN: {e}")
        input("ENTER za izlaz...")
        sys.exit(1)

    # File outputs
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path    = os.path.abspath(f"steer_test_{ts}.log")
    summary_path = os.path.abspath(f"steer_test_{ts}_summary.txt")
    phases_path  = os.path.abspath(f"steer_test_{ts}_phases.txt")

    log_fp = open(log_path, "w", encoding="utf-8")
    log_fp.write(f"# CAN steer test  {datetime.now()}, bitrate {BITRATE}\n")

    # State
    total_msgs    = 0
    id_counter    = Counter()
    pgn_counter   = Counter()
    sa_counter    = Counter()
    pgn_examples  = {}
    phase_msgs    = defaultdict(list)   # phase_idx -> list of (timestamp, id, data)

    current_phase = 0
    phase_start   = time.time()
    start_time    = time.time()

    # Keyboard listener
    advance_event = threading.Event()
    stop_event    = threading.Event()

    def keyboard_thread():
        while not stop_event.is_set():
            try:
                input()
                advance_event.set()
            except (EOFError, KeyboardInterrupt):
                stop_event.set()
                break

    kb = threading.Thread(target=keyboard_thread, daemon=True)
    kb.start()

    # Show first phase
    title, instruction = PHASES[current_phase]
    show_phase_banner(current_phase + 1, len(PHASES), title, instruction, total_msgs)
    log_fp.write(f"# Phase {current_phase + 1}: {title}\n")

    last_redraw = time.time()

    try:
        while not stop_event.is_set():
            # Provjeri ENTER
            if advance_event.is_set():
                advance_event.clear()
                # zabiljezi trajanje
                phase_dur = time.time() - phase_start
                log_fp.write(f"# Phase {current_phase + 1} END after {phase_dur:.1f}s, "
                             f"{len(phase_msgs[current_phase])} msgs\n")

                current_phase += 1
                if current_phase >= len(PHASES):
                    break  # zadnja faza je "KRAJ", izlazak

                phase_start = time.time()
                title, instruction = PHASES[current_phase]
                show_phase_banner(current_phase + 1, len(PHASES), title, instruction, total_msgs)
                log_fp.write(f"\n# Phase {current_phase + 1}: {title}\n")

            # Refresh banner svakih 2s da osvjezi msg counter
            if time.time() - last_redraw > 2.0:
                title, instruction = PHASES[current_phase]
                show_phase_banner(current_phase + 1, len(PHASES), title, instruction, total_msgs)
                last_redraw = time.time()

            # Recv
            msg = bus.recv(timeout=0.3)
            if msg is None:
                continue

            total_msgs += 1
            id_counter[msg.arbitration_id] += 1
            data_hex = msg.data.hex().upper()

            log_fp.write(f"({msg.timestamp:.6f}) bus "
                         f"{msg.arbitration_id:08X}"
                         f"{'X' if msg.is_extended_id else ''}"
                         f"#{data_hex} p{current_phase + 1}\n")

            if msg.is_extended_id:
                pgn, sa = j1939_decode(msg.arbitration_id)
                pgn_counter[pgn] += 1
                sa_counter[sa]   += 1
                if pgn not in pgn_examples:
                    pgn_examples[pgn] = (msg.arbitration_id, data_hex)

            # spremi za phase analysis (svaku 10. poruku da ne raste previse)
            if total_msgs % 5 == 0:
                phase_msgs[current_phase].append(
                    (time.time() - start_time, msg.arbitration_id, data_hex)
                )

    except KeyboardInterrupt:
        clear_screen()
        print("Prekinuto Ctrl+C.")
    finally:
        stop_event.set()
        log_fp.close()
        try:
            bus.shutdown()
        except Exception:
            pass

    clear_screen()
    print()
    print("=" * 70)
    print("   SNIFF GOTOV - zapisujem sazetke...")
    print("=" * 70)
    print()
    print(f"   Ukupno: {total_msgs} okvira, {len(id_counter)} jedinstvenih ID-ova")
    print(f"   Faza odradeno: {current_phase + 1 if current_phase >= len(PHASES) else current_phase}")
    print()

    # ===== SAZETAK =====
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write(f"CAN steer test sazetak  {datetime.now()}\n")
        f.write(f"Bitrate: {BITRATE} bps\n")
        f.write(f"Trajanje: {time.time()-start_time:.1f}s\n")
        f.write(f"Ukupno: {total_msgs} okvira\n\n")

        f.write("=" * 50 + "\n")
        f.write("TOP 30 SOURCE ADDRESSES\n")
        f.write("=" * 50 + "\n")
        for sa, n in sa_counter.most_common(30):
            hint = ""
            if sa == 0xF7: hint = " *** PVED-CLS Steering ***"
            elif sa == 0xF0: hint = " (Engine ECU / Fleet)"
            elif sa == 0x00: hint = " (Engine std)"
            elif sa == 0x9F: hint = " (Navigation)"
            f.write(f"  SA 0x{sa:02X} ({sa:3d})  x{n}{hint}\n")

        f.write("\n" + "=" * 50 + "\n")
        f.write("TOP 40 PGN-OVA s primjerima\n")
        f.write("=" * 50 + "\n")
        for pgn, n in pgn_counter.most_common(40):
            example_id, example_data = pgn_examples.get(pgn, (0, ""))
            f.write(f"  PGN 0x{pgn:04X}  x{n:5d}  primjer: ID 0x{example_id:08X} data {example_data}\n")

        f.write("\n" + "=" * 50 + "\n")
        f.write("SVI CAN ID-OVI (top 60 po broju)\n")
        f.write("=" * 50 + "\n")
        for cid, n in id_counter.most_common(60):
            f.write(f"  0x{cid:08X}  x{n}\n")

    # ===== PHASE COMPARISON =====
    with open(phases_path, "w", encoding="utf-8") as f:
        f.write(f"Faze eksperimenta  {datetime.now()}\n")
        f.write(f"Bitrate: {BITRATE} bps\n\n")

        # Za svaki ID, koliko vrijednosti payload-a smo vidjeli po fazi
        # (da identificiramo koji ID-ovi reagiraju na volan)
        id_payloads_per_phase = defaultdict(lambda: defaultdict(set))
        for phase_idx, msgs in phase_msgs.items():
            for ts, cid, data in msgs:
                id_payloads_per_phase[cid][phase_idx].add(data)

        f.write("=" * 60 + "\n")
        f.write("ID-OVI KOJI MIJENJAJU PAYLOAD IZMEDU FAZA\n")
        f.write("(kandidati za WAS / steering position)\n")
        f.write("=" * 60 + "\n\n")

        for cid in sorted(id_payloads_per_phase.keys()):
            phases_data = id_payloads_per_phase[cid]
            total_unique = set()
            for p_unique in phases_data.values():
                total_unique |= p_unique

            if len(total_unique) > 1:
                f.write(f"\nID 0x{cid:08X}: {len(total_unique)} razlicitih payload-a ukupno\n")
                for phase_idx in sorted(phases_data.keys()):
                    title = PHASES[phase_idx][0] if phase_idx < len(PHASES) else "?"
                    payloads = phases_data[phase_idx]
                    if len(payloads) <= 3:
                        f.write(f"  Faza {phase_idx+1} ({title}): {len(payloads)} unikatnih: "
                                f"{', '.join(sorted(payloads))}\n")
                    else:
                        sample = sorted(payloads)[:3]
                        f.write(f"  Faza {phase_idx+1} ({title}): {len(payloads)} unikatnih, primjer: "
                                f"{', '.join(sample)}...\n")

        f.write("\n\n")
        f.write("=" * 60 + "\n")
        f.write("ID-OVI KOJI SU STATICNI (isti payload uvijek)\n")
        f.write("=" * 60 + "\n")
        for cid in sorted(id_payloads_per_phase.keys()):
            phases_data = id_payloads_per_phase[cid]
            total_unique = set()
            for p_unique in phases_data.values():
                total_unique |= p_unique
            if len(total_unique) == 1:
                payload = list(total_unique)[0]
                f.write(f"  0x{cid:08X}: {payload}\n")

    print(f"   Snimljeno:")
    print(f"     Raw log:  {log_path}")
    print(f"     Sazetak:  {summary_path}")
    print(f"     Faze:     {phases_path}")
    print()
    print("   Posalji sva tri fajla Nikoli.")
    print()
    input("   ENTER za izlaz...")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\nGRESKA: {e}")
        import traceback
        traceback.print_exc()
        input("\nENTER za izlaz...")
