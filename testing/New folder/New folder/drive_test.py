#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Guidance master simulator - DRIVE TEST verzija
================================================
Za test dok traktor polako vozi (2-5 km/h).

Plan:
   0-15s    BASELINE - samo slusam (vozac moze biti u praznom hodu ili lagano krenuti)
   15-75s   ACTIVE - saljem Address Claim + nulti Curvature, gledam PVED
   
Live prikaz:
   - Trenutna brzina vozila (PGN 0xFE48)
   - PVED status (PGN 0xCB00)
   - Broj poslanih komandi
   - Da li PVED reagira

SIGURNOST:
   - Curvature = 0 (ravno)
   - Engage = FALSE
   - Vozac DRZI VOLAN ruke obje
   - Otvoreno polje, prazno u radijusu 30m
"""

import sys
import os
import time
import threading
from datetime import datetime
from collections import defaultdict, Counter

PCAN_CHANNEL = "PCAN_USBBUS1"
BITRATE      = 250_000

OUR_SA       = 0x9C
PVED_SA      = 0xF7

DURATION             = 90       # ukupno (15 baseline + 75 active)
BASELINE_DURATION    = 15
ADDR_CLAIM_PERIOD    = 5.0
CURVATURE_PERIOD     = 0.1


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')


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
    name = 0x12345
    name |= 0 << 21          # mfg
    name |= 0 << 32
    name |= 0 << 35
    name |= 0x9C << 40       # function = Navigation
    name |= 0 << 49
    name |= 0 << 56
    name |= 2 << 60          # industry agriculture
    name |= 1 << 63          # arb addr capable
    return name.to_bytes(8, 'little')


def make_curvature_payload(curvature=0.0, engage=False):
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


def decode_speed_kmh(data):
    """PGN 0xFE48 Wheel Speed: bytes 0-1 little-endian, unit = 1/256 km/h."""
    if len(data) < 2:
        return None
    raw = data[0] | (data[1] << 8)
    if raw == 0xFFFF:
        return None
    return raw / 256.0


def main():
    clear_screen()
    print("=" * 70)
    print("   GUIDANCE MASTER - DRIVE TEST")
    print("=" * 70)
    print()
    print("   SIGURNOSNA LISTA - OBAVEZNO PROVJERI:")
    print("     [ ] Otvoreno polje/dvoriste, prazno 30m u krug")
    print("     [ ] Vozac u kabini, obje ruke na volanu")
    print("     [ ] Telefonska veza s vozacem aktivna")
    print("     [ ] Vozac zna: pri bilo cemu neobicnom -> KOČNICA + NEUTRAL")
    print("     [ ] VRC iskljucen")
    print("     [ ] Curvature = 0 (ravno), engage = FALSE")
    print()
    print(f"   Plan ({DURATION}s ukupno):")
    print(f"     0 - {BASELINE_DURATION}s:   BASELINE - vozac moze krenuti polako (2-3 km/h)")
    print(f"     {BASELINE_DURATION} - {DURATION}s:  ACTIVE - ja emitiram, vozac VOZI cijelo vrijeme")
    print()
    input("   Vozac potvrdio da je spreman? ENTER za start...")

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
    log_path     = os.path.abspath(f"drive_test_{ts}.log")
    csv_path     = os.path.abspath(f"drive_test_{ts}.csv")
    pved_path    = os.path.abspath(f"drive_test_{ts}_pved.txt")

    log_fp = open(log_path, "w", encoding="utf-8")
    csv_fp = open(csv_path, "w", encoding="utf-8")
    log_fp.write(f"# Drive test  {datetime.now()}, bitrate {BITRATE}\n")
    csv_fp.write("ts,phase,dir,id,pgn,sa,ps,data,b0,b1,b2,b3,b4,b5,b6,b7\n")

    # State
    pved_baseline = defaultdict(set)
    pved_active   = defaultdict(set)
    pved_all      = []

    speed_history = []   # (ts, kmh)
    current_speed = None
    current_pved_cb00 = None
    current_pved_fecb = None
    
    all_msgs = 0
    sent_addr = 0
    sent_curv = 0
    
    stop_flag = threading.Event()
    tx_enabled = threading.Event()

    addr_claim_id   = make_j1939_id(priority=6, pgn=0xEE00, sa=OUR_SA, ps=0xFF)
    addr_claim_name = make_address_claim_name()
    curv_cmd_id     = make_j1939_id(priority=3, pgn=0xAD00, sa=OUR_SA, ps=0xFF)
    curv_payload    = make_curvature_payload(curvature=0.0, engage=False)

    def log_msg(timestamp, direction, msg):
        data_hex = msg.data.hex().upper()
        log_fp.write(f"({timestamp:.6f}) {direction} "
                     f"{msg.arbitration_id:08X}"
                     f"{'X' if msg.is_extended_id else ''}#{data_hex}\n")
        if msg.is_extended_id:
            pgn, sa, ps = j1939_decode(msg.arbitration_id)
        else:
            pgn, sa, ps = 0, 0, 0
        b = list(msg.data) + [0]*(8-len(msg.data))
        b = b[:8]
        phase = "ACTIVE" if tx_enabled.is_set() else "BASELINE"
        csv_fp.write(f"{timestamp:.6f},{phase},{direction},0x{msg.arbitration_id:08X},"
                     f"0x{pgn:04X},0x{sa:02X},0x{ps:02X},{data_hex},"
                     f"{b[0]},{b[1]},{b[2]},{b[3]},{b[4]},{b[5]},{b[6]},{b[7]}\n")

    # TX thread
    def tx_thread():
        nonlocal sent_addr, sent_curv
        while not tx_enabled.is_set() and not stop_flag.is_set():
            time.sleep(0.1)
        if stop_flag.is_set():
            return

        # Initial AC
        try:
            msg = can.Message(arbitration_id=addr_claim_id,
                              data=addr_claim_name, is_extended_id=True)
            bus.send(msg)
            sent_addr += 1
            log_msg(time.time(), "TX", msg)
        except Exception:
            pass

        last_addr = time.time()
        last_curv = time.time()

        while not stop_flag.is_set():
            now = time.time()
            if now - last_addr >= ADDR_CLAIM_PERIOD:
                try:
                    msg = can.Message(arbitration_id=addr_claim_id,
                                      data=addr_claim_name, is_extended_id=True)
                    bus.send(msg)
                    sent_addr += 1
                    log_msg(now, "TX", msg)
                    last_addr = now
                except Exception:
                    pass

            if now - last_curv >= CURVATURE_PERIOD:
                try:
                    msg = can.Message(arbitration_id=curv_cmd_id,
                                      data=curv_payload, is_extended_id=True)
                    bus.send(msg)
                    sent_curv += 1
                    log_msg(now, "TX", msg)
                    last_curv = now
                except Exception:
                    pass

            time.sleep(0.02)

    tx = threading.Thread(target=tx_thread, daemon=True)
    tx.start()

    start_time = time.time()
    last_display = 0
    pved_new_events = []   # za pamcenje kad je PVED prvi put pokazao nesto novo

    try:
        while (time.time() - start_time) < DURATION:
            now = time.time()
            elapsed = now - start_time

            # aktivacija TX
            if elapsed >= BASELINE_DURATION and not tx_enabled.is_set():
                tx_enabled.set()
                log_fp.write(f"# === ACTIVE phase starts @ t={elapsed:.2f}s ===\n")

            msg = bus.recv(timeout=0.05)
            if msg is not None:
                all_msgs += 1
                log_msg(now, "RX", msg)
                
                if msg.is_extended_id:
                    pgn, sa, ps = j1939_decode(msg.arbitration_id)
                    data_hex = msg.data.hex().upper()
                    
                    # Brzina
                    if pgn == 0xFE48:
                        sp = decode_speed_kmh(msg.data)
                        if sp is not None:
                            current_speed = sp
                            speed_history.append((elapsed, sp))
                    
                    # PVED status
                    if sa == PVED_SA:
                        if pgn == 0xCB00:
                            current_pved_cb00 = data_hex
                        elif pgn == 0xFECB:
                            current_pved_fecb = data_hex

                    if sa == PVED_SA or ps == PVED_SA:
                        pved_all.append((elapsed, msg.arbitration_id, pgn, sa, ps, data_hex,
                                         "active" if tx_enabled.is_set() else "baseline"))
                        if tx_enabled.is_set():
                            if data_hex not in pved_active[pgn]:
                                # NEW payload u active fazi!
                                if data_hex in pved_baseline[pgn]:
                                    pved_active[pgn].add(data_hex)
                                else:
                                    pved_active[pgn].add(data_hex)
                                    pved_new_events.append((elapsed, pgn, data_hex))
                        else:
                            pved_baseline[pgn].add(data_hex)

            # Refresh ekran 4x u sekundi
            if now - last_display >= 0.25:
                last_display = now
                clear_screen()
                phase = "ACTIVE" if tx_enabled.is_set() else "BASELINE"
                phase_color = "*** ACTIVE ***" if tx_enabled.is_set() else "BASELINE"
                
                print("=" * 70)
                print(f"   DRIVE TEST   t={elapsed:5.1f}s / {DURATION}s   [{phase_color}]")
                print("=" * 70)
                print()
                
                # Brzina prominent
                if current_speed is not None:
                    sp_bar = '#' * int(current_speed * 5)
                    print(f"   BRZINA:   {current_speed:5.2f} km/h   {sp_bar}")
                else:
                    print(f"   BRZINA:   --- (nema PGN 0xFE48)")
                print()
                
                # PVED status
                print(f"   PVED 0xCB00 (status):  {current_pved_cb00 or '(jos ne)'}")
                print(f"   PVED 0xFECB (DTC):     {current_pved_fecb or '(jos ne)'}")
                print()
                
                # Counters
                print(f"   Primljeno:        {all_msgs} okvira")
                print(f"   Poslano AC:       {sent_addr}")
                print(f"   Poslano Curv:     {sent_curv}")
                print()
                
                # PVED reakcija
                print("   PVED komunikacija (po PGN-u):")
                all_pved_pgns = set(pved_baseline.keys()) | set(pved_active.keys())
                for pgn in sorted(all_pved_pgns):
                    b = len(pved_baseline[pgn])
                    a = len(pved_active[pgn])
                    new = pved_active[pgn] - pved_baseline[pgn]
                    marker = f"  *** {len(new)} NOVI ***" if new else ""
                    print(f"     PGN 0x{pgn:04X}:  baseline={b} unique, active={a} unique{marker}")
                print()
                
                # New events log (zadnja 3)
                if pved_new_events:
                    print("   NEW PVED reakcije (zadnje):")
                    for t, pgn, data in pved_new_events[-3:]:
                        print(f"     t={t:5.1f}s  PGN 0x{pgn:04X}  {data}")
                    print()
                
                # Status na dnu
                if not tx_enabled.is_set():
                    print(f"   << BASELINE faza ({BASELINE_DURATION-elapsed:.0f}s do TX) - vozac neka pocne polako voziti >>")
                else:
                    print(f"   >> ACTIVE - vozim, drzi volan obje ruke, {DURATION-elapsed:.0f}s do kraja <<")
                print()
                print("   Ctrl+C za hitno zaustavljanje")

    except KeyboardInterrupt:
        clear_screen()
        print("\n  PREKID! Vozac neka stane sigurno.")
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
    clear_screen()
    print("=" * 70)
    print("   DRIVE TEST GOTOV")
    print("=" * 70)
    print()
    print(f"   Trajanje:    {time.time()-start_time:.1f}s")
    print(f"   Poruka RX:   {all_msgs}")
    print(f"   Poslano AC:  {sent_addr}")
    print(f"   Poslano Curv: {sent_curv}")
    print()
    
    # Brzina statistika
    if speed_history:
        speeds_active = [s for t,s in speed_history if t >= BASELINE_DURATION]
        if speeds_active:
            print(f"   Brzina tokom ACTIVE:  min={min(speeds_active):.2f} max={max(speeds_active):.2f} "
                  f"avg={sum(speeds_active)/len(speeds_active):.2f} km/h")
    print()
    
    # PVED reakcija
    print("   PVED reakcija:")
    all_pved_pgns = set(pved_baseline.keys()) | set(pved_active.keys())
    any_reaction = False
    for pgn in sorted(all_pved_pgns):
        b = pved_baseline[pgn]
        a = pved_active[pgn]
        new = a - b
        if new:
            any_reaction = True
            print(f"     PGN 0x{pgn:04X}: *** {len(new)} NOVI payloadovi u ACTIVE fazi:")
            for p in sorted(new):
                print(f"        {p}")
    
    if not any_reaction:
        print("     PVED se ponaša ISTO u baseline i active fazama.")
        print("     (Ili nas ne primjećuje, ili odgovara isto bez obzira na nas)")
    
    # Write PVED report
    with open(pved_path, "w", encoding="utf-8") as f:
        f.write(f"Drive Test PVED Report  {datetime.now()}\n")
        f.write(f"Trajanje: {time.time()-start_time:.1f}s\n\n")
        
        if speed_history:
            speeds_baseline = [s for t,s in speed_history if t < BASELINE_DURATION]
            speeds_active = [s for t,s in speed_history if t >= BASELINE_DURATION]
            f.write("BRZINA VOZILA:\n")
            if speeds_baseline:
                f.write(f"  Baseline: min={min(speeds_baseline):.2f} max={max(speeds_baseline):.2f} km/h\n")
            if speeds_active:
                f.write(f"  Active:   min={min(speeds_active):.2f} max={max(speeds_active):.2f} avg={sum(speeds_active)/len(speeds_active):.2f} km/h\n")
            f.write("\n")
        
        f.write("PVED REAKCIJA (baseline vs active):\n")
        for pgn in sorted(all_pved_pgns):
            b = pved_baseline[pgn]
            a = pved_active[pgn]
            new = a - b
            f.write(f"\nPGN 0x{pgn:04X}:\n")
            f.write(f"  Baseline ({len(b)} unique):\n")
            for p in sorted(b):
                f.write(f"    {p}\n")
            f.write(f"  Active ({len(a)} unique):\n")
            for p in sorted(a):
                marker = " *** NEW ***" if p in new else ""
                f.write(f"    {p}{marker}\n")
        
        f.write("\n\nKRONOLOGIJA SVIH PVED PORUKA:\n")
        for t, arb_id, pgn, sa, ps, data, phase in pved_all:
            f.write(f"  [{phase:8}] t={t:6.2f}s  ID 0x{arb_id:08X}  PGN 0x{pgn:04X}  {data}\n")

    print()
    print(f"   Log:     {log_path}")
    print(f"   CSV:     {csv_path}")
    print(f"   PVED:    {pved_path}")
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
