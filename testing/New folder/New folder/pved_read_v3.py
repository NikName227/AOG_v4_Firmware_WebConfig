#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PVED Param Read v3 - pokusaj drugih SA
=======================================
Saljemo isti request, ali s razlicitim source adresama.
Mozda neka SA je autorizirana za parameter read u tvornickoj konfiguraciji.

Naprije sniffat cemo dali se neka SA vec koristi za dijagnostiku,
pa cemo pokusati s tih + drugim standardnim SA-ovima.
"""

import sys
import os
import time
from datetime import datetime

PCAN_CHANNEL = "PCAN_USBBUS1"
BITRATE      = 250_000

PVED_SA = 0xF7
PARAM   = 64007

# Source adrese za pokusaj
# Standardne J1939 SA i autorizirane za diagnostiku
TRY_SOURCES = [
    (0xFE, "off-board diag tool 1 (Tony default)"),
    (0xFB, "off-board diag tool 2"),
    (0xFA, "off-board service tool"),
    (0xF9, "off-board service tool 2"),
    (0xF8, "lighting controller (vec aktivna SA na busu)"),
    (0xF0, "fleet management (engine ECU)"),
    (0xDE, "vec aktivna SA"),
    (0xD3, "vec aktivna SA"),
    (0x80, "navigation generic"),
    (0x1C, "SASA-style"),
    (0x2B, "vec aktivna SA"),
    (0x26, "vec aktivna SA"),
    (0x9C, "Navigation reserved 1"),
    (0x9F, "Navigation reserved 2"),
    (0x1E, "Tony AIO board default"),
]


def main():
    print("=" * 70)
    print("   PVED Param Read v3 - try different source addresses")
    print("=" * 70)
    print()
    print(f"   Parametar:  {PARAM} ({hex(PARAM)})")
    print(f"   Dest:       0x{PVED_SA:02X} (PVED)")
    print(f"   Probati cu  {len(TRY_SOURCES)} razlicitih SA")
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
    log_path = os.path.abspath(f"pved_sa_{ts}.log")
    rep_path = os.path.abspath(f"pved_sa_{ts}_report.txt")
    log_fp = open(log_path, "w", encoding="utf-8")

    param_lo = PARAM & 0xFF
    param_hi = (PARAM >> 8) & 0xFF
    payload = bytes([param_lo, param_hi, 0xFF])

    results = []

    for our_sa, desc in TRY_SOURCES:
        # Build TX ID: PGN 0xEA00, dest=F7, source=our_sa, priority 6
        tx_id = (6 << 26) | (0xEA << 16) | (PVED_SA << 8) | our_sa
        
        print(f"\n   --- SA 0x{our_sa:02X}  ({desc}) ---")
        print(f"   TX  0x{tx_id:08X}  {payload.hex().upper()}")
        
        # Pre-test clear
        t = time.time()
        while time.time() - t < 0.1:
            m = bus.recv(timeout=0.05)
        
        # Send
        try:
            msg = can.Message(arbitration_id=tx_id, data=payload, is_extended_id=True)
            bus.send(msg)
            send_ts = time.time()
            log_fp.write(f"\n# === SA 0x{our_sa:02X}: {desc} ===\n")
            log_fp.write(f"({send_ts:.6f}) TX {tx_id:08X}#{payload.hex().upper()}\n")
        except Exception as e:
            print(f"   GRESKA TX: {e}")
            results.append((our_sa, desc, "TX FAILED", []))
            continue

        # Listen 1.5s for response
        responses = []
        start = time.time()
        while time.time() - start < 1.5:
            msg = bus.recv(timeout=0.1)
            if msg is None:
                continue
            log_fp.write(f"({msg.timestamp:.6f}) RX {msg.arbitration_id:08X}"
                         f"{'X' if msg.is_extended_id else ''}#{msg.data.hex().upper()}\n")
            
            sa = msg.arbitration_id & 0xFF
            ps = (msg.arbitration_id >> 8) & 0xFF
            pf = (msg.arbitration_id >> 16) & 0xFF
            pgn = (pf << 8) if pf < 240 else ((pf << 8) | ps)
            
            # Samo poruke iz PVED-a prema nama
            if sa == PVED_SA and ps == our_sa:
                # Dekodiraj ACK
                if pgn == 0xE800 and len(msg.data) >= 1:
                    ack_type = msg.data[0]
                    ack_names = {
                        0x00: "POSITIVE ACK",
                        0x01: "NEGATIVE ACK (Access Denied)",
                        0x02: "Access Denied",
                        0x03: "Cannot Respond"
                    }
                    ack_name = ack_names.get(ack_type, f"unknown 0x{ack_type:02X}")
                    print(f"     [{time.time()-send_ts:5.2f}s] RX  PGN 0xE800  {ack_name}  data {msg.data.hex().upper()}")
                    responses.append((time.time()-send_ts, "ACK", ack_type, msg.data.hex().upper()))
                else:
                    # Mozda pravi parameter response!
                    print(f"     [{time.time()-send_ts:5.2f}s] *** RX  PGN 0x{pgn:04X}  data {msg.data.hex().upper()} ***")
                    responses.append((time.time()-send_ts, "RESPONSE", pgn, msg.data.hex().upper()))

        if not responses:
            print(f"     (nema odgovora)")
            results.append((our_sa, desc, "NO RESPONSE", []))
        else:
            # Check if any positive
            has_positive = any(r[1] == "RESPONSE" or (r[1] == "ACK" and r[2] == 0) for r in responses)
            status = "*** POSITIVE ***" if has_positive else "NACK (denied)"
            results.append((our_sa, desc, status, responses))

    log_fp.close()
    try:
        bus.shutdown()
    except Exception:
        pass

    # Report
    print()
    print("=" * 70)
    print("   REZIME")
    print("=" * 70)
    with open(rep_path, "w", encoding="utf-8") as f:
        f.write(f"PVED Read v3 - source address test  {datetime.now()}\n")
        f.write(f"Parametar: {PARAM}\n")
        f.write(f"Dest SA: 0x{PVED_SA:02X}\n\n")
        f.write(f"{'SA':>5}  {'Opis':40} {'Status':35}\n")
        f.write("-" * 80 + "\n")
        
        any_positive = False
        for sa, desc, status, responses in results:
            line = f"  0x{sa:02X}  {desc[:38]:38} {status[:33]:33}"
            f.write(line + "\n")
            print(line)
            if "POSITIVE" in status:
                any_positive = True
                for r in responses:
                    detail = f"      response: type={r[1]}  code/pgn={r[2]}  data={r[3]}"
                    f.write(detail + "\n")
                    print(detail)
        
        f.write("\n")
        if any_positive:
            f.write("*** NEKE SA SU AUTORIZIRANE! ***\n")
        else:
            f.write("Nijedna SA nije autorizirana za parameter read.\n")
            f.write("PVED treba ili Diagnostic Session Start, ili je read u tvornici disabled.\n")

    print()
    print(f"   Log:    {log_path}")
    print(f"   Report: {rep_path}")
    input("\n   ENTER za izlaz...")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"GRESKA: {e}")
        import traceback
        traceback.print_exc()
        input()
