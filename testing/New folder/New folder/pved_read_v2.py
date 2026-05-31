#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PVED Parameter Read v2 - DIRECTED request
==========================================
Sad probaj nekoliko varijanti formata:
   1. Tony's original: ID 0x18EAFFFE, payload 3 bytes
   2. Directed: ID 0x18EAF7FE (dest=PVED), payload 3 bytes
   3. Directed s padding: 8 bytes payload
   4. Manji parametar broj prvi (508 je relativno mali)

Za svaki test čekamo 1s odgovor, pa sljedeci.
"""

import sys
import os
import time
from datetime import datetime

PCAN_CHANNEL = "PCAN_USBBUS1"
BITRATE      = 250_000

PVED_SA      = 0xF7
OUR_SA       = 0xFE   # off-board diagnostic, kao Tony

# Test cases: (description, can_id, payload)
TEST_CASES = []

def add_test_case(desc, can_id, payload):
    TEST_CASES.append((desc, can_id, payload))

# === Varijante za parametar 64007 (Setpoint Controller Address) ===
PARAM = 64007
param_lo = PARAM & 0xFF
param_hi = (PARAM >> 8) & 0xFF

# Var 1: Tony's original (broadcast, 3 bytes)
add_test_case(
    "Tony original (broadcast, 3B)",
    0x18EAFFFE,
    bytes([param_lo, param_hi, 0xFF])
)

# Var 2: Directed to PVED (3 bytes)
add_test_case(
    "Directed to PVED (3B)",
    0x18EAF7FE,    # PGN EA00, dest=F7 (PVED), src=FE
    bytes([param_lo, param_hi, 0xFF])
)

# Var 3: Directed s 8 byte padding
add_test_case(
    "Directed to PVED (8B padded)",
    0x18EAF7FE,
    bytes([param_lo, param_hi, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
)

# Var 4: Danfoss proprietary direct - moze biti drugaciji format
# Tony pvedWriteParam koristi ID 0x1CEFFF1E s payload [0x1C, 0xFA, ...]
# Mozda za READ koristi slican format ali drugi sub-command:
# Pokusajmo s [0x1C, 0xFB, param_lo, param_hi, ...] (0xFB = read sub-command?)
add_test_case(
    "Danfoss PEFFF1E format (read sub-cmd FB)",
    0x18EFF7FE,    # PGN EF00 (proprietary), dest=F7, src=FE
    bytes([0x1C, 0xFB, param_lo, param_hi, 0xFF, 0xFF, 0xFF, 0xFF])
)

# Var 5: Drugi Danfoss read variant
add_test_case(
    "Danfoss PEFFF1E format (read sub-cmd 1A)",
    0x18EFF7FE,
    bytes([0x1A, 0xFA, param_lo, param_hi, 0xFF, 0xFF, 0xFF, 0xFF])
)

# Var 6: Standard J1939 PGN Request - request specific PGN
# Request PGN 0xCB00 (PVED status)
add_test_case(
    "Standard J1939 Request PGN 0xCB00",
    0x18EAF7FE,
    bytes([0x00, 0xCB, 0x00])
)

# Var 7: Test 65080 (manji ID, mozda lakse)
PARAM2 = 65080
add_test_case(
    "Request 65080 directed",
    0x18EAF7FE,
    bytes([PARAM2 & 0xFF, (PARAM2 >> 8) & 0xFF, 0xFF])
)


def main():
    print("=" * 70)
    print("   PVED Parameter Read v2 - directed requests")
    print("=" * 70)
    print()
    print(f"   Test cases: {len(TEST_CASES)}")
    for i, (desc, id, payload) in enumerate(TEST_CASES, 1):
        print(f"     {i}. {desc}")
        print(f"        ID=0x{id:08X}  payload={payload.hex().upper()}")
    print()
    print("   Za svaki: salje, ceka 2s odgovor, ide na sljedeci.")
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
    log_path = os.path.abspath(f"pved_read_v2_{ts}.log")
    rep_path = os.path.abspath(f"pved_read_v2_{ts}_report.txt")
    log_fp = open(log_path, "w", encoding="utf-8")

    results = []

    print()
    for i, (desc, tx_id, payload) in enumerate(TEST_CASES, 1):
        print(f"\n   --- Test {i}: {desc} ---")
        print(f"   TX  0x{tx_id:08X}  {payload.hex().upper()}")
        
        # Pre-test: očisti bus 100ms
        cleared = 0
        t = time.time()
        while time.time() - t < 0.1:
            m = bus.recv(timeout=0.05)
            if m: cleared += 1
        
        # Send
        try:
            msg = can.Message(arbitration_id=tx_id, data=payload, is_extended_id=True)
            bus.send(msg)
            send_ts = time.time()
            log_fp.write(f"\n# === TEST {i}: {desc} ===\n")
            log_fp.write(f"({send_ts:.6f}) TX {tx_id:08X}#{payload.hex().upper()}\n")
        except Exception as e:
            print(f"   GRESKA TX: {e}")
            results.append((desc, "TX FAILED", []))
            continue

        # Slusaj 2 sekunde
        responses = []
        start = time.time()
        while time.time() - start < 2.0:
            msg = bus.recv(timeout=0.1)
            if msg is None:
                continue
            log_fp.write(f"({msg.timestamp:.6f}) RX {msg.arbitration_id:08X}"
                         f"{'X' if msg.is_extended_id else ''}#{msg.data.hex().upper()}\n")
            
            # Filter samo poruke iz PVED-a (SA = 0xF7) ILI prema nama (PS = 0xFE)
            sa = msg.arbitration_id & 0xFF
            ps = (msg.arbitration_id >> 8) & 0xFF
            
            if sa == PVED_SA or ps == OUR_SA:
                # Iskljuci poznate periodicke poruke
                # 0xCB00 (status) i 0xFECB (DTC) su standardni periodicki
                pf = (msg.arbitration_id >> 16) & 0xFF
                pgn = (pf << 8) if pf < 240 else ((pf << 8) | ps)
                
                # Pokupi samo "interesantne" odgovore
                if pgn not in (0xCB00, 0xFECB, 0xEE00, 0xEA00):
                    responses.append((time.time() - send_ts, msg.arbitration_id, 
                                      pgn, sa, ps, msg.data.hex().upper()))
                    print(f"     [{time.time()-send_ts:5.2f}s] RX  "
                          f"0x{msg.arbitration_id:08X}  PGN 0x{pgn:04X}  {msg.data.hex().upper()}")

        if not responses:
            print(f"     (nema relevantnih odgovora)")
        results.append((desc, "OK" if responses else "NO RESPONSE", responses))

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
        f.write(f"PVED Read v2 report  {datetime.now()}\n\n")
        for i, (desc, status, responses) in enumerate(results, 1):
            f.write(f"=== Test {i}: {desc} ===\n")
            f.write(f"Status: {status}\n")
            print(f"\n   Test {i}: {desc}: {status}")
            if responses:
                for t, arb_id, pgn, sa, ps, data in responses:
                    line = f"  t={t:5.2f}s  ID 0x{arb_id:08X}  PGN 0x{pgn:04X}  SA=0x{sa:02X} PS=0x{ps:02X}  {data}"
                    f.write(line + "\n")
                    print(f"      {line}")
            f.write("\n")

    print()
    print(f"   Log:    {log_path}")
    print(f"   Report: {rep_path}")
    print()
    input("   ENTER za izlaz...")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"GRESKA: {e}")
        import traceback
        traceback.print_exc()
        input()
