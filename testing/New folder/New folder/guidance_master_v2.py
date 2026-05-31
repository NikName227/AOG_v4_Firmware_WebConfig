#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Guidance master simulator s POTPUNIM logiranjem
================================================
Predstavlja se kao guidance master, salje Address Claim + nulti Curvature,
i logira SVE poruke s kompletnim podatkom za naknadnu analizu.

Output fajlovi:
   guidance_test_YYYYMMDD_HHMMSS.log         - raw candump format, sve poruke
   guidance_test_YYYYMMDD_HHMMSS.csv         - strukturirano: ts,dir,id,pgn,sa,ps,data,b0..b7
   guidance_test_YYYYMMDD_HHMMSS_pved.txt    - sažetak PVED komunikacije
   guidance_test_YYYYMMDD_HHMMSS_summary.txt - top SA/PGN, before/after usporedba
"""

import sys
import os
import time
import threading
from datetime import datetime
from collections import defaultdict, Counter

PCAN_CHANNEL = "PCAN_USBBUS1"
BITRATE      = 250_000

OUR_SA       = 0x9C    # Navigation
PVED_SA      = 0xF7    # PVED-CLS

DURATION             = 60      # sekundi ukupno
BASELINE_DURATION    = 10      # prvih 10s SAMO slusamo (bez emisije)
ADDR_CLAIM_PERIOD    = 5.0
CURVATURE_PERIOD     = 0.1


def make_j1939_id(priority, pgn, sa, ps=0xFF):
    pf = (pgn >> 8) & 0xFF
    if pf < 240:
        return (priority << 26) | (pf << 16) | (ps << 8) | sa
    else:
        ps_byte = pgn & 0xFF
        return (priority << 26) | (pf << 16) | (ps_byte << 8) | sa


def j1939_decode(arb_id):
    pf = (arb_id >> 16) & 0xFF
    ps = (arb_id >>  8) & 0xFF
    sa =  arb_id        & 0xFF
    if pf < 240:
        pgn = pf << 8
    else:
        pgn = (pf << 8) | ps
    return pgn, sa, ps


def make_address_claim_name():
    identity      = 0x12345
    manufacturer  = 0
    function      = 0x9C        # Navigation
    industry_grp  = 2           # Agriculture
    arb_addr      = 1

    name = identity
    name |= manufacturer  << 21
    name |= 0             << 32   # ECU instance
    name |= 0             << 35   # function instance
    name |= function      << 40
    name |= 0             << 49   # vehicle system
    name |= 0             << 56   # vehicle inst
    name |= industry_grp  << 60
    name |= arb_addr      << 63

    return name.to_bytes(8, 'little')


def make_curvature_command_payload(curvature=0.0, engage=False):
    curv_raw = int(curvature * 4) + 8032
    curv_raw = max(0, min(65535, curv_raw))
    status = 0b01 if engage else 0b00

    payload = bytearray(8)
    payload[0] = curv_raw & 0xFF
    payload[1] = (curv_raw >> 8) & 0xFF
    payload[2] = status | 0xFC
    for i in range(3, 8):
        payload[i] = 0xFF
    return bytes(payload)


def main():
    print("=" * 70)
    print("   GUIDANCE MASTER SIMULATOR  (s POTPUNIM logiranjem)")
    print("=" * 70)
    print()
    print(f"   Plan ({DURATION}s ukupno):")
    print(f"     0 -  {BASELINE_DURATION}s:  SAMO slusam (baseline, bez emisije)")
    print(f"    {BASELINE_DURATION} - {DURATION}s:  Slusam + saljem Address Claim + Curvature")
    print()
    print("   PROVJERI:")
    print("   - Traktor STOJI")
    print("   - Rucna kocnica POVUCENA")
    print("   - Motor moze biti upaljen ili ugasen")
    print("   - PCAN na pin 1/3 MCD")
    print()
    print(f"   Nasa SA: 0x{OUR_SA:02X}")
    print(f"   PVED na: 0x{PVED_SA:02X}")
    print()
    input("   ENTER za start...")

    try:
        import can
    except ImportError:
        print("FALI python-can"); sys.exit(1)

    try:
        bus = can.Bus(interface="pcan", channel=PCAN_CHANNEL,
                      bitrate=BITRATE, receive_own_messages=False)
    except Exception as e:
        print(f"GRESKA: {e}"); input(); sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path     = os.path.abspath(f"guidance_test_{ts}.log")
    csv_path     = os.path.abspath(f"guidance_test_{ts}.csv")
    pved_path    = os.path.abspath(f"guidance_test_{ts}_pved.txt")
    summary_path = os.path.abspath(f"guidance_test_{ts}_summary.txt")

    log_fp = open(log_path, "w", encoding="utf-8")
    csv_fp = open(csv_path, "w", encoding="utf-8")
    
    log_fp.write(f"# Guidance master test  {datetime.now()}\n")
    log_fp.write(f"# Bitrate: {BITRATE}, our SA: 0x{OUR_SA:02X}, PVED SA: 0x{PVED_SA:02X}\n")
    log_fp.write(f"# Plan: 0-{BASELINE_DURATION}s baseline (RX only), then RX+TX\n")
    
    csv_fp.write("timestamp,phase,direction,id_hex,pgn_hex,sa,ps,data_hex,"
                 "b0,b1,b2,b3,b4,b5,b6,b7\n")

    # State
    pved_messages = defaultdict(list)
    pved_unique_payloads = defaultdict(set)
    baseline_pved_payloads = defaultdict(set)   # za "before" usporedbu
    active_pved_payloads = defaultdict(set)     # za "after" usporedbu
    
    all_msgs = 0
    sent_addr = 0
    sent_curv = 0
    
    sa_counter = Counter()
    pgn_counter = Counter()
    id_counter = Counter()
    
    stop_flag = threading.Event()
    tx_enabled = threading.Event()    # postaje True nakon BASELINE_DURATION

    # ----- TX setup -----
    addr_claim_id   = make_j1939_id(priority=6, pgn=0xEE00, sa=OUR_SA, ps=0xFF)
    addr_claim_name = make_address_claim_name()
    curv_cmd_id     = make_j1939_id(priority=3, pgn=0xAD00, sa=OUR_SA, ps=0xFF)
    curv_payload    = make_curvature_command_payload(curvature=0.0, engage=False)
    
    print()
    print(f"   TX Address Claim: ID 0x{addr_claim_id:08X}  data {addr_claim_name.hex().upper()}")
    print(f"   TX Curvature:     ID 0x{curv_cmd_id:08X}  data {curv_payload.hex().upper()}")
    print()

    def log_message(timestamp, direction, msg):
        """Zapisi poruku u log i CSV."""
        data_hex = msg.data.hex().upper()
        log_fp.write(f"({timestamp:.6f}) {direction} "
                     f"{msg.arbitration_id:08X}"
                     f"{'X' if msg.is_extended_id else ''}"
                     f"#{data_hex}\n")
        
        if msg.is_extended_id:
            pgn, sa, ps = j1939_decode(msg.arbitration_id)
        else:
            pgn, sa, ps = 0, 0, 0
        
        # padded bytes to 8
        bytes_padded = list(msg.data) + [0] * (8 - len(msg.data))
        bytes_padded = bytes_padded[:8]
        
        phase = "ACTIVE" if tx_enabled.is_set() else "BASELINE"
        
        csv_fp.write(f"{timestamp:.6f},{phase},{direction},"
                     f"0x{msg.arbitration_id:08X},0x{pgn:04X},0x{sa:02X},0x{ps:02X},"
                     f"{data_hex},"
                     f"{bytes_padded[0]},{bytes_padded[1]},{bytes_padded[2]},{bytes_padded[3]},"
                     f"{bytes_padded[4]},{bytes_padded[5]},{bytes_padded[6]},{bytes_padded[7]}\n")

    # ----- TX thread -----
    def tx_thread():
        nonlocal sent_addr, sent_curv
        
        # Cekaj da BASELINE prode
        while not tx_enabled.is_set() and not stop_flag.is_set():
            time.sleep(0.1)
        
        if stop_flag.is_set():
            return
        
        # Initial Address Claim cim postanemo aktivni
        try:
            msg = can.Message(arbitration_id=addr_claim_id,
                              data=addr_claim_name,
                              is_extended_id=True)
            bus.send(msg)
            sent_addr += 1
            log_message(time.time(), "TX", msg)
            log_fp.write(f"# === ACTIVE phase starts ===\n")
        except Exception as e:
            print(f"TX greska: {e}")

        last_addr = time.time()
        last_curv = time.time()

        while not stop_flag.is_set():
            now = time.time()
            
            if now - last_addr >= ADDR_CLAIM_PERIOD:
                try:
                    msg = can.Message(arbitration_id=addr_claim_id,
                                      data=addr_claim_name,
                                      is_extended_id=True)
                    bus.send(msg)
                    sent_addr += 1
                    log_message(now, "TX", msg)
                    last_addr = now
                except Exception:
                    pass

            if now - last_curv >= CURVATURE_PERIOD:
                try:
                    msg = can.Message(arbitration_id=curv_cmd_id,
                                      data=curv_payload,
                                      is_extended_id=True)
                    bus.send(msg)
                    sent_curv += 1
                    log_message(now, "TX", msg)
                    last_curv = now
                except Exception:
                    pass

            time.sleep(0.02)

    tx = threading.Thread(target=tx_thread, daemon=True)
    tx.start()

    # ----- Main RX loop -----
    start_time = time.time()
    last_display = 0
    activated_logged = False

    print(f"   Pokrenuto. Cekam {DURATION}s...")
    print()
    print("   PVED odgovori (samo NOVI payloadovi se prikazuju):")
    print()

    try:
        while (time.time() - start_time) < DURATION:
            now = time.time()
            elapsed = now - start_time
            
            # Aktiviraj TX nakon baseline-a
            if elapsed >= BASELINE_DURATION and not tx_enabled.is_set():
                tx_enabled.set()
                if not activated_logged:
                    print(f"\n   [{elapsed:5.1f}s]  >>> AKTIVNI <<< saljem Address Claim + Curvature\n")
                    activated_logged = True

            msg = bus.recv(timeout=0.1)
            if msg is None:
                # progres
                if now - last_display >= 5.0:
                    last_display = now
                    phase = "ACTIVE" if tx_enabled.is_set() else "BASELINE"
                    print(f"   ... {int(elapsed)}s ({phase}) "
                          f"RX={all_msgs} TX_a={sent_addr} TX_c={sent_curv} "
                          f"PVED_PGN={len(pved_messages)}")
                continue

            all_msgs += 1
            log_message(now, "RX", msg)

            if msg.is_extended_id:
                pgn, sa, ps = j1939_decode(msg.arbitration_id)
                sa_counter[sa] += 1
                pgn_counter[pgn] += 1
                id_counter[msg.arbitration_id] += 1

                # PVED-related?
                if sa == PVED_SA or ps == PVED_SA:
                    data_hex = msg.data.hex().upper()
                    direction = "PVED->" if sa == PVED_SA else "->PVED"
                    new_payload = data_hex not in pved_unique_payloads[pgn]
                    
                    if new_payload:
                        marker = " <-- NEW"
                        print(f"   [{elapsed:5.1f}s] {direction}  "
                              f"PGN 0x{pgn:04X}  data {data_hex}{marker}")
                    
                    pved_messages[pgn].append((elapsed, data_hex, "active" if tx_enabled.is_set() else "baseline"))
                    pved_unique_payloads[pgn].add(data_hex)
                    
                    if tx_enabled.is_set():
                        active_pved_payloads[pgn].add(data_hex)
                    else:
                        baseline_pved_payloads[pgn].add(data_hex)

    except KeyboardInterrupt:
        print("\n   Prekid sa Ctrl+C")
    finally:
        stop_flag.set()
        time.sleep(0.3)
        log_fp.close()
        csv_fp.close()
        try:
            bus.shutdown()
        except Exception:
            pass

    # ===== ANALIZA =====
    total_time = time.time() - start_time
    print()
    print("=" * 70)
    print("   GOTOVO")
    print("=" * 70)
    print(f"   Trajanje:        {total_time:.1f}s")
    print(f"   Primljeno:       {all_msgs} okvira")
    print(f"   Poslato AC:      {sent_addr}")
    print(f"   Poslato Curv:    {sent_curv}")
    print(f"   PVED PGN-ova:    {len(pved_messages)}")
    print()

    # Pokazi BEFORE/AFTER usporedbu
    print("   PVED komunikacija - BASELINE vs ACTIVE:")
    print("   " + "-" * 65)
    
    all_pved_pgns = set(baseline_pved_payloads.keys()) | set(active_pved_payloads.keys())
    for pgn in sorted(all_pved_pgns):
        baseline_n = len(baseline_pved_payloads[pgn])
        active_n   = len(active_pved_payloads[pgn])
        new_in_active = active_pved_payloads[pgn] - baseline_pved_payloads[pgn]
        marker = " *** NEW PAYLOADS ***" if new_in_active else ""
        print(f"     PGN 0x{pgn:04X}: baseline={baseline_n} unique, "
              f"active={active_n} unique{marker}")
        if new_in_active:
            for payload in sorted(new_in_active)[:5]:
                print(f"        NEW: {payload}")
    print()

    # ===== PVED REPORT =====
    with open(pved_path, "w", encoding="utf-8") as f:
        f.write(f"PVED komunikacija report  {datetime.now()}\n")
        f.write(f"Trajanje: {total_time:.1f}s\n")
        f.write(f"Baseline (samo RX): 0-{BASELINE_DURATION}s\n")
        f.write(f"Active (RX+TX):     {BASELINE_DURATION}-{DURATION}s\n")
        f.write(f"Nasa SA: 0x{OUR_SA:02X}\n")
        f.write(f"PVED SA: 0x{PVED_SA:02X}\n\n")
        f.write(f"Address Claims poslano: {sent_addr}\n")
        f.write(f"Curvature Commands poslano: {sent_curv}\n")
        f.write(f"Ukupno primljeno: {all_msgs}\n\n")

        f.write("=" * 60 + "\n")
        f.write("USPOREDBA: BASELINE vs ACTIVE\n")
        f.write("=" * 60 + "\n\n")
        
        if not all_pved_pgns:
            f.write("NEMA NIKAKVE PVED komunikacije.\n")
            f.write("PVED ili nije na sabirnici, ili je u dubokom sleep modu.\n\n")
        else:
            for pgn in sorted(all_pved_pgns):
                baseline_uniq = baseline_pved_payloads[pgn]
                active_uniq   = active_pved_payloads[pgn]
                new_in_active = active_uniq - baseline_uniq
                lost_in_active = baseline_uniq - active_uniq
                
                f.write(f"\n--- PGN 0x{pgn:04X} ({pgn}) ---\n")
                f.write(f"Baseline unique payloads: {len(baseline_uniq)}\n")
                f.write(f"Active unique payloads:   {len(active_uniq)}\n")
                f.write(f"NEW in active phase:      {len(new_in_active)}\n")
                f.write(f"LOST in active phase:     {len(lost_in_active)}\n\n")
                
                if baseline_uniq:
                    f.write("Baseline payloads:\n")
                    for p in sorted(baseline_uniq):
                        f.write(f"  {p}\n")
                
                if new_in_active:
                    f.write("\n*** NEW payloads in ACTIVE phase ***:\n")
                    for p in sorted(new_in_active):
                        f.write(f"  {p}\n")

                # Show kronologija (sve poruke)
                f.write(f"\nKronologija (sve {len(pved_messages[pgn])} poruka):\n")
                for t, data, phase in pved_messages[pgn]:
                    marker = "*" if phase == "active" and data in new_in_active else " "
                    f.write(f"  [{phase:8}] t={t:6.2f}s  {marker} {data}\n")

        f.write("\n" + "=" * 60 + "\n")
        f.write("INTERPRETACIJA\n")
        f.write("=" * 60 + "\n")
        f.write("NEW payloads in active -> PVED nas DETEKTIRA i reagira\n")
        f.write("Same payloads -> PVED nas IGNORIRA\n")
        f.write("Novi PGN-ovi (npr. 0xAC00) -> PVED je usao u 'aware' mod\n")
        f.write("PGN 0xEEFF (Address Claim with our SA) iz drugog ECU-a -> SA collision\n")

    # ===== GLOBAL SUMMARY =====
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write(f"Summary  {datetime.now()}\n")
        f.write(f"Trajanje: {total_time:.1f}s, bitrate: {BITRATE}\n\n")
        
        f.write("=" * 50 + "\n")
        f.write("TOP 20 SOURCE ADDRESSES\n")
        f.write("=" * 50 + "\n")
        for sa, n in sa_counter.most_common(20):
            hint = " (PVED)" if sa == PVED_SA else (" (us)" if sa == OUR_SA else "")
            f.write(f"  SA 0x{sa:02X}  x{n}{hint}\n")
        
        f.write("\n" + "=" * 50 + "\n")
        f.write("TOP 30 PGN-OVA\n")
        f.write("=" * 50 + "\n")
        for pgn, n in pgn_counter.most_common(30):
            f.write(f"  PGN 0x{pgn:04X}  x{n}\n")
        
        f.write("\n" + "=" * 50 + "\n")
        f.write("TOP 40 ID-OVA\n")
        f.write("=" * 50 + "\n")
        for cid, n in id_counter.most_common(40):
            f.write(f"  0x{cid:08X}  x{n}\n")

    print()
    print("   FAJLOVI:")
    print(f"     Raw log:        {log_path}")
    print(f"     CSV strukturiran: {csv_path}")
    print(f"     PVED report:    {pved_path}")
    print(f"     Summary:        {summary_path}")
    print()
    print("   Posalji sva cetiri fajla.")
    input("\n   ENTER za izlaz...")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\nGRESKA: {e}")
        import traceback
        traceback.print_exc()
        input("\nENTER za izlaz...")
