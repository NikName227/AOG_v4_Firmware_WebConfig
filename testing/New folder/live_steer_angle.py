#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Live steer angle viewer
========================
Prikazuje real-time poziciju volana iz PGN 0xF802 (SA 0x1C).

Identificirano iz prijasnjeg sniffa:
   CENTAR:  byte0 = 0x00
   LIJEVO:  byte0 = 0x38 (+56)
   DESNO:   byte0 = 0x96 (-106 ako signed)

Skripta pokazuje:
   - sirovi byte0 hex
   - byte0 kao signed integer
   - byte1 (mozda high byte 16-bit)
   - vizualna bar (lijevo/desno)
   - min/max viđen do sad
   - frekvencija update-a (Hz)

Pokretanje: python live_steer_angle.py
"""

import sys
import os
import time
from datetime import datetime

PCAN_CHANNEL = "PCAN_USBBUS1"
BITRATE      = 250_000

# Target ID koji znamo
TARGET_ID    = 0x09F8021C   # PGN 0xF802, SA 0x1C  - glavni WAS kandidat

# Sekundarni ID za dodatne info
SECONDARY_ID = 0x0DF8051C   # PGN 0xF805, SA 0x1C  - composite report

# Brzina (PGN 0xFE48 - Wheel Speed)
SPEED_ID     = 0x0CFE48F0


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')


def signed_byte(b):
    """Pretvori unsigned byte (0-255) u signed (-128 to 127)."""
    return b - 256 if b > 127 else b


def signed_word(lo, hi):
    """16-bit signed iz dva byte-a, little-endian."""
    val = lo | (hi << 8)
    return val - 65536 if val > 32767 else val


def make_bar(value, vmin=-128, vmax=127, width=50):
    """Napravi tekstualni bar: <----[+]---->."""
    # normalize value to 0..width
    range_ = vmax - vmin
    pos = int(((value - vmin) / range_) * width)
    pos = max(0, min(width, pos))
    center = width // 2

    bar = ['-'] * width
    bar[center] = '|'    # centralna oznaka
    bar[pos] = '*'       # trenutna pozicija

    left = "L"
    right = "R"
    return f"{left} [{''.join(bar)}] {right}"


def main():
    print("Live steer angle viewer")
    print(f"  Bitrate: {BITRATE} bps")
    print(f"  Target ID: 0x{TARGET_ID:08X}")
    print()
    print("Spoji PCAN na pin 1 i pin 3.")
    input("Pritisni ENTER za start...")

    try:
        import can
    except ImportError:
        print("FALI python-can. pip install python-can")
        sys.exit(1)

    try:
        bus = can.Bus(interface="pcan", channel=PCAN_CHANNEL,
                      bitrate=BITRATE, receive_own_messages=False)
    except Exception as e:
        print(f"GRESKA: {e}")
        input("ENTER za izlaz...")
        sys.exit(1)

    # State
    last_data         = None
    last_update_time  = time.time()
    last_display_time = 0
    update_count      = 0
    update_rate       = 0.0

    min_signed_b0     = 0
    max_signed_b0     = 0
    min_word          = 0
    max_word          = 0

    last_secondary    = None
    last_speed_raw    = None

    start_time        = time.time()

    print("\nPokretam sniff... (Ctrl+C za stop)\n")
    time.sleep(1)

    try:
        while True:
            msg = bus.recv(timeout=0.1)
            if msg is None:
                continue

            now = time.time()

            if msg.arbitration_id == TARGET_ID and len(msg.data) >= 2:
                last_data = msg.data
                update_count += 1
                # update rate calculation each second
                if now - last_update_time >= 1.0:
                    update_rate = update_count / (now - last_update_time)
                    update_count = 0
                    last_update_time = now

                # Track min/max
                b0_signed = signed_byte(msg.data[0])
                if b0_signed < min_signed_b0: min_signed_b0 = b0_signed
                if b0_signed > max_signed_b0: max_signed_b0 = b0_signed

                if len(msg.data) >= 2:
                    word_val = signed_word(msg.data[0], msg.data[1])
                    if word_val < min_word: min_word = word_val
                    if word_val > max_word: max_word = word_val

            elif msg.arbitration_id == SECONDARY_ID:
                last_secondary = msg.data

            elif msg.arbitration_id == SPEED_ID:
                last_speed_raw = msg.data

            # Refresh display 10 puta u sekundi
            if now - last_display_time >= 0.1 and last_data is not None:
                last_display_time = now

                clear_screen()
                print("=" * 70)
                print("   LIVE STEER ANGLE VIEWER")
                print("=" * 70)
                print()

                # Raw data
                data_hex = last_data.hex().upper()
                print(f"  ID: 0x{TARGET_ID:08X}   data: {data_hex}")
                print()

                # Byte 0 analiza
                b0 = last_data[0]
                b0_s = signed_byte(b0)
                print(f"  Byte 0:  unsigned = {b0:3d}  (0x{b0:02X})")
                print(f"           signed   = {b0_s:+4d}")
                print()

                # Byte 1
                if len(last_data) >= 2:
                    b1 = last_data[1]
                    print(f"  Byte 1:  unsigned = {b1:3d}  (0x{b1:02X})")

                    # 16-bit interpretacija
                    word_s = signed_word(b0, b1)
                    word_u = b0 | (b1 << 8)
                    print(f"  16-bit LE: unsigned = {word_u:5d}  signed = {word_s:+6d}")
                    print()

                # Vizualizacija
                print("  Pozicija (byte 0 signed):")
                print(f"  {make_bar(b0_s, -128, 127)}")
                print()

                # Min / Max
                print(f"  Min/Max byte0 signed:   {min_signed_b0:+4d}  /  {max_signed_b0:+4d}")
                print(f"  Range:                  {max_signed_b0 - min_signed_b0}")
                print()

                if last_data and len(last_data) >= 2:
                    print(f"  Min/Max 16-bit signed:  {min_word:+6d}  /  {max_word:+6d}")
                    print()

                # Sekundarni / brzina
                if last_secondary:
                    print(f"  Sec ID 0x{SECONDARY_ID:08X}: {last_secondary.hex().upper()}")
                if last_speed_raw:
                    # PGN 0xFE48 byte 0-1 je tipicno Front Axle Speed, 1/256 km/h
                    speed_raw = last_speed_raw[0] | (last_speed_raw[1] << 8)
                    speed_kmh = speed_raw / 256.0
                    print(f"  Speed PGN 0xFE48:    {last_speed_raw.hex().upper()}   ~{speed_kmh:.2f} km/h")

                print()
                print("-" * 70)
                print(f"  Update rate: {update_rate:.1f} Hz")
                print(f"  Trajanje:    {now - start_time:.0f}s")
                print()
                print("  Ctrl+C za stop")

    except KeyboardInterrupt:
        clear_screen()
        print("\nStop.")
        print()
        print("Zabiljezene granice:")
        print(f"  byte0 signed: {min_signed_b0:+4d} .. {max_signed_b0:+4d}  (range {max_signed_b0 - min_signed_b0})")
        print(f"  16-bit signed: {min_word:+6d} .. {max_word:+6d}  (range {max_word - min_word})")
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass

    input("\nENTER za izlaz...")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"GRESKA: {e}")
        import traceback
        traceback.print_exc()
        input("\nENTER za izlaz...")
