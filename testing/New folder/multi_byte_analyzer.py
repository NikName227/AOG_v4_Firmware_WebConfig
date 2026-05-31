#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Multi-byte CAN analyzer s log-om
=================================
Prati ID-ove i ispisuje sve byte vrijednosti u CSV i sažetak.

Output:
   multi_byte_YYYYMMDD_HHMMSS.csv  - svaka primljena poruka, sve byte vrijednosti
   multi_byte_YYYYMMDD_HHMMSS.txt  - finalni sažetak (min/max/range po byte-u)
"""

import sys
import os
import time
from datetime import datetime

PCAN_CHANNEL = "PCAN_USBBUS1"
BITRATE      = 250_000

WATCH_IDS = [0x09F8021C, 0x0DF8051C, 0x09F8011C]


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')


def signed(b):
    return b - 256 if b > 127 else b


def main():
    print("Multi-byte analyzer s log-om")
    print(f"Pratim ID-ove: {[hex(i) for i in WATCH_IDS]}")
    print()
    print("VAZNO: motor mora biti UPALJEN da WAS daje prave vrijednosti!")
    print()
    input("ENTER za start...")

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
    csv_path = os.path.abspath(f"multi_byte_{ts}.csv")
    txt_path = os.path.abspath(f"multi_byte_{ts}.txt")

    csv_fp = open(csv_path, "w", encoding="utf-8")
    csv_fp.write("timestamp,id,b0,b1,b2,b3,b4,b5,b6,b7,b0_signed,b1_signed,b16le_signed\n")

    # State per ID
    state = {cid: {
        'last_data':   None,
        'min_byte':    [255]*8,
        'max_byte':    [0]*8,
        'count':       0,
    } for cid in WATCH_IDS}

    last_display = 0
    start_time = time.time()

    print(f"\nLogiram u: {csv_path}\n")
    time.sleep(0.5)

    try:
        while True:
            msg = bus.recv(timeout=0.1)
            now = time.time()

            if msg is not None and msg.arbitration_id in WATCH_IDS:
                s = state[msg.arbitration_id]
                s['last_data'] = bytes(msg.data)
                s['count'] += 1

                # Pad data to 8 bytes
                data = list(msg.data) + [0] * (8 - len(msg.data))
                data = data[:8]

                for i, b in enumerate(data):
                    if b < s['min_byte'][i]: s['min_byte'][i] = b
                    if b > s['max_byte'][i]: s['max_byte'][i] = b

                # CSV row
                b0_s = signed(data[0])
                b1_s = signed(data[1])
                b16le = data[0] | (data[1] << 8)
                if b16le > 32767: b16le -= 65536

                csv_fp.write(
                    f"{msg.timestamp:.3f},0x{msg.arbitration_id:08X},"
                    f"{data[0]},{data[1]},{data[2]},{data[3]},"
                    f"{data[4]},{data[5]},{data[6]},{data[7]},"
                    f"{b0_s},{b1_s},{b16le}\n"
                )
                csv_fp.flush()  # za slucaj prekidanja

            # Refresh ekran 5x u sekundi
            if now - last_display >= 0.2:
                last_display = now
                clear_screen()
                print("=" * 80)
                print("   MULTI-BYTE ANALYZER  (motor mora biti UPALJEN!)")
                print("=" * 80)
                print()
                print("  Okreci volan: LIJEVO do kraja -> CENTAR -> DESNO do kraja -> CENTAR")
                print()

                for cid in WATCH_IDS:
                    s = state[cid]
                    if s['last_data'] is None:
                        print(f"  ID 0x{cid:08X}: cekam podatke...")
                        continue

                    data = s['last_data']
                    print(f"  ID 0x{cid:08X}  ({s['count']} primljeno)  data: {data.hex().upper()}")
                    print(f"    {'B':2} {'Now':>6} {'Hex':>5} {'Signed':>8} {'Min':>5} {'Max':>5} {'Range':>6}")
                    for i in range(min(8, len(data))):
                        b = data[i]
                        bs = signed(b)
                        mn = s['min_byte'][i]
                        mx = s['max_byte'][i]
                        rng = mx - mn
                        marker = " ***" if rng > 30 else ""
                        print(f"    B{i} {b:6d} 0x{b:02X}  {bs:+5d}    {mn:5d} {mx:5d} {rng:5d}{marker}")
                    print()

                print(f"  Trajanje: {now - start_time:.0f}s   |   CSV: {os.path.basename(csv_path)}")
                print(f"  Ctrl+C za stop i sazetak")

    except KeyboardInterrupt:
        pass
    finally:
        csv_fp.close()
        try:
            bus.shutdown()
        except Exception:
            pass

    clear_screen()
    print("\n" + "=" * 70)
    print("   GOTOVO - zapisujem sazetak")
    print("=" * 70 + "\n")

    # Sazetak u TXT
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write(f"Multi-byte analyzer rezultat  {datetime.now()}\n")
        f.write(f"Bitrate: {BITRATE} bps\n")
        f.write(f"Trajanje: {time.time() - start_time:.1f}s\n")
        f.write(f"Pratio ID-ove: {[hex(i) for i in WATCH_IDS]}\n")
        f.write(f"CSV log: {os.path.basename(csv_path)}\n\n")

        for cid in WATCH_IDS:
            s = state[cid]
            f.write("=" * 70 + "\n")
            f.write(f"ID 0x{cid:08X}   ({s['count']} primljenih poruka)\n")
            f.write("=" * 70 + "\n")

            if s['count'] == 0:
                f.write("Nema primljenih poruka za ovaj ID.\n\n")
                continue

            if s['last_data']:
                f.write(f"Zadnja vrijednost: {s['last_data'].hex().upper()}\n\n")

            f.write(f"{'Byte':6} {'Min':>5} {'Max':>5} {'Range':>6}  Status\n")
            for i in range(8):
                mn = s['min_byte'][i]
                mx = s['max_byte'][i]
                rng = mx - mn
                if rng == 0:
                    status = "stabilan (konstantno)"
                elif rng <= 10:
                    status = "mali jitter (vjerojatno sum/counter)"
                elif rng <= 50:
                    status = "umjereno se mijenja"
                elif rng <= 100:
                    status = "JAKO SE MIJENJA - mozda WAS"
                else:
                    status = "*** PUNI RANGE - vrlo vjerojatno WAS ili counter ***"
                f.write(f"  B{i}   {mn:5d} {mx:5d} {rng:5d}   {status}\n")
            f.write("\n")

        f.write("\n" + "=" * 70 + "\n")
        f.write("INTERPRETACIJA\n")
        f.write("=" * 70 + "\n")
        f.write("Pravi WAS byte mora imati:\n")
        f.write("  - Range > 50 (mijenja se kroz puni okret volana)\n")
        f.write("  - Glatku promjenu (ne skokove) - vidi se u CSV\n")
        f.write("  - Vraca se na istu vrijednost kad je volan na centru\n\n")
        f.write("Counter byte ima:\n")
        f.write("  - Range puni (0..255) ali ciklicki\n")
        f.write("  - Mijenja se i kad volan stoji\n\n")
        f.write("CRC byte ima:\n")
        f.write("  - Random vrijednosti\n")
        f.write("  - Range puni (0..255)\n\n")
        f.write("Otvori CSV u Excel-u i napravi graf po byte-u za precizniju analizu.\n")

    print(f"Snimljeno:")
    print(f"  CSV (sve poruke):  {csv_path}")
    print(f"  TXT (sazetak):     {txt_path}")
    print()
    print("Posalji oba fajla Nikoli.")
    input("\nENTER za izlaz...")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"GRESKA: {e}")
        import traceback
        traceback.print_exc()
        input()