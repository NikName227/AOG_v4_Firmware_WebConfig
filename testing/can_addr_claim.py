#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Address Claim hunter
====================
Snima Address Claim poruke (PGN 0xEE00) pri paljenju traktora.
Cilj: pronaci PVED-CLS na sabirnici.

UPUTSTVA:
  1. UGASI KONTAKT traktora kompletno, cekaj 30s
  2. Pokreni ovu skriptu
  3. Kad pise "ZAPOCNJEM SNIFF - SAD UPALI KONTAKT", upali ga
  4. Skripta snima 60s i ispisuje sve Address Claim-ove

Pokretanje: python can_addr_claim.py
"""

import sys
import os
import time
from datetime import datetime

PCAN_CHANNEL = "PCAN_USBBUS1"
BITRATE      = 250_000          # promijeni za drugu brzinu
DURATION     = 60               # sekundi


def j1939_decode(arb_id):
    pf = (arb_id >> 16) & 0xFF
    ps = (arb_id >>  8) & 0xFF
    sa =  arb_id        & 0xFF
    if pf < 240:
        pgn = pf << 8
    else:
        pgn = (pf << 8) | ps
    return pgn, sa


def decode_addr_claim(data):
    """Dekodira Address Claim payload (8 bajta)."""
    if len(data) < 8:
        return None
    # Format (J1939-81):
    #   bytes 0-1: Identity Number (21 bits) + ECU Instance (3 bits) + Function Instance (5 bits)
    #   byte 2:    Function (8 bits)
    #   byte 3:    bit 7=reserved, bits 6-1=Vehicle System (7 bits), bit 0=Vehicle System Instance
    #   byte 4:    Industry Group (3 bits) + Arbitrary Address bit
    #   bytes 5-7: Manufacturer Code (11 bits) + Identity high bits

    identity = data[0] | (data[1] << 8) | ((data[2] & 0x1F) << 16)
    function = data[3]
    vehicle_system = (data[4] >> 1) & 0x7F
    industry_group = (data[6] >> 4) & 0x07
    # Manufacturer code je tricky - 11 bita
    manufacturer = ((data[7] & 0xFF) << 3) | ((data[6] >> 5) & 0x07)

    func_name = {
        0x00: "Engine",
        0x03: "Transmission",
        0x05: "Shift Control / Steering",
        0x0B: "Brakes",
        0x82: "Steering Controller",   # !!! PVED uses this
        0x1D: "Off-board diagnostic",
        0x20: "Navigation",
    }.get(function, f"unknown 0x{function:02X}")

    mfg_hint = ""
    if manufacturer == 257:
        mfg_hint = " *** DANFOSS ***"
    elif manufacturer == 86:
        mfg_hint = " (John Deere)"
    elif manufacturer == 98:
        mfg_hint = " (Case New Holland)"
    elif manufacturer == 184:
        mfg_hint = " (AGCO)"
    elif manufacturer == 226:
        mfg_hint = " (Same Deutz-Fahr)"
    elif manufacturer == 11:
        mfg_hint = " (Bosch)"

    return {
        "function":     function,
        "func_name":    func_name,
        "manufacturer": manufacturer,
        "mfg_hint":     mfg_hint,
        "identity":     identity,
        "vehicle_sys":  vehicle_system,
    }


def main():
    print("=" * 64)
    print(f"  Address Claim hunter  (bitrate {BITRATE})")
    print("=" * 64)
    print()
    print("KORACI:")
    print("  1. UGASI KONTAKT traktora KOMPLETNO. Cekaj 30 sekundi.")
    print("  2. Vrati se ovdje i pritisni ENTER.")
    print("  3. Kad pise 'UPALI KONTAKT SAD', upali ga.")
    print("  4. Skripta cuva svaki Address Claim sljedecih 60s.")
    print()
    input("Kontakt OFF + 30s ceka? Pritisni ENTER za nastavak...")

    try:
        import can
    except ImportError:
        print("Fali python-can. pip install python-can")
        sys.exit(1)

    print(f"\nOtvaram CAN na {BITRATE} bps...")
    try:
        bus = can.Bus(interface="pcan", channel=PCAN_CHANNEL,
                      bitrate=BITRATE, receive_own_messages=False)
    except Exception as e:
        print(f"GRESKA: {e}")
        input("ENTER za izlaz...")
        sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path    = f"addr_claim_{ts}.log"
    report_path = f"addr_claim_{ts}.txt"
    log_fp = open(log_path, "w", encoding="utf-8")
    log_fp.write(f"# Address Claim hunter {datetime.now()}, bitrate {BITRATE}\n")

    print()
    print("*" * 64)
    print("  ZAPOCINJEM SNIFF - UPALI KONTAKT SAD")
    print("*" * 64)
    print()

    start = time.time()
    addr_claims  = []
    other_count  = 0
    sa_seen      = set()

    try:
        while time.time() - start < DURATION:
            msg = bus.recv(timeout=0.5)
            if msg is None:
                # progres
                if int(time.time() - start) % 10 == 0:
                    elapsed = int(time.time() - start)
                    if elapsed > 0 and elapsed % 10 == 0:
                        pass  # silently
                continue

            data_hex = msg.data.hex().upper()
            log_fp.write(f"({msg.timestamp:.3f}) {msg.arbitration_id:08X}#{data_hex}\n")

            if not msg.is_extended_id:
                continue

            pgn, sa = j1939_decode(msg.arbitration_id)
            sa_seen.add(sa)

            if pgn == 0xEE00:
                # Address Claim!
                info = decode_addr_claim(msg.data)
                addr_claims.append({
                    "time":    time.time() - start,
                    "sa":      sa,
                    "data":    data_hex,
                    "info":    info,
                })
                print(f"  [{time.time()-start:5.1f}s] ADDR CLAIM "
                      f"SA=0x{sa:02X}  data={data_hex}")
                if info:
                    print(f"           function={info['func_name']}  "
                          f"mfg={info['manufacturer']}{info['mfg_hint']}")
            else:
                other_count += 1

    except KeyboardInterrupt:
        pass
    finally:
        log_fp.close()
        try:
            bus.shutdown()
        except Exception:
            pass

    # SAZETAK
    print()
    print("=" * 64)
    print(f"  GOTOVO  ({DURATION}s prosao)")
    print("=" * 64)
    print(f"  Ukupno: {len(addr_claims)} Address Claim-ova")
    print(f"  Druge poruke: {other_count}")
    print(f"  Source addresses vidjene: {sorted(['0x%02X'%s for s in sa_seen])}")
    print()

    with open(report_path, "w", encoding="utf-8") as f:
        f.write(f"Address Claim report  {datetime.now()}\n")
        f.write(f"Bitrate: {BITRATE} bps\n")
        f.write(f"Trajanje: {DURATION}s\n\n")
        f.write(f"Ukupno Address Claims: {len(addr_claims)}\n")
        f.write(f"Druge poruke (ne ADR): {other_count}\n")
        f.write(f"Source addresses: {sorted(sa_seen)}\n\n")

        if addr_claims:
            f.write("=" * 60 + "\n")
            f.write("ADDRESS CLAIMS (kronoloski):\n")
            f.write("=" * 60 + "\n")
            for ac in addr_claims:
                f.write(f"\nt={ac['time']:.2f}s  SA=0x{ac['sa']:02X}  data={ac['data']}\n")
                if ac['info']:
                    info = ac['info']
                    f.write(f"  Function:     0x{info['function']:02X} ({info['func_name']})\n")
                    f.write(f"  Manufacturer: {info['manufacturer']}{info['mfg_hint']}\n")
                    f.write(f"  Identity:     {info['identity']}\n")
                    f.write(f"  Vehicle Sys:  {info['vehicle_sys']}\n")
        else:
            f.write("NEMA ADDRESS CLAIM-OVA.\n")
            f.write("Mozda:\n")
            f.write("  - Nismo na pravoj sabirnici\n")
            f.write("  - Krivi baud rate\n")
            f.write("  - Sabirnica je mrtva\n")
            f.write("  - Propustili pocetni trenutak (treba ugasiti-upaliti kontakt)\n")

    print(f"Snimljeno:")
    print(f"  Log:    {log_path}")
    print(f"  Report: {report_path}")
    print()
    input("ENTER za izlaz...")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"GRESKA: {e}")
        import traceback
        traceback.print_exc()
        input("\nENTER za izlaz...")
