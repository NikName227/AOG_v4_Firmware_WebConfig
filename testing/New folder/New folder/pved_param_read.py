#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PVED Parameter Reader (po Tonijevom kodu)
==========================================
Salje J1939 Read Parameter request za PVED-CLS parametre i slusa odgovore.
SAMO CITA - nista ne mijenja.

Iz Tony-jevog koda (pvedReadParam funkcija):
   TX ID: 0x18EAFFFE  (PGN 0xEA00 = Request, destination 0xFF, source 0xFE)
   Data:  [param_lo, param_hi, 0xFF]  (3 bytes)

Odgovor (Danfoss proprietary, dolazi natrag na sabirnicu):
   Data pocinje s [0x1C, 0xFA, param_lo, param_hi, value_lo, value_hi, ...]

Parametri koji se citaju:
   508, 706, 707, 729, 737, 738, 747, 748, 758,
   1027, 5027, 64007*, 64022, 64023,
   65080, 65083, 65086, 65099, 65100, 65101, 65104, 65112

   * 64007 = Setpoint Controller Address (KLJUCNI!)
     - 0xFF = no controller assigned (PVED IGNORIRA sve guidance master-e)
     - 0x1E = AIO board (Tony's pločica) je assigned
     - druga vrijednost = neki specifičan controller

SIGURNOST:
   - Read-only (samo Request poruke)
   - Nista se ne mijenja u PVED EEPROM-u
   - Volan se ne mice
"""

import sys
import os
import time
import threading
from datetime import datetime

PCAN_CHANNEL = "PCAN_USBBUS1"
BITRATE      = 250_000

# === iz Tony-jevog koda ===
READ_REQUEST_ID = 0x18EAFFFE   # PGN EA00 Request, dest=FF, source=FE

# Parametri (iz Tony-jevog pvedReadAll() funkcije)
PARAMS_TO_READ = [
    508, 706, 707, 729, 737, 738, 747, 748, 758,
    1027, 5027,
    64007,      # *** KLJUCNI: Setpoint Controller Address ***
    64022, 64023,
    65080, 65083, 65086, 65099, 65100, 65101, 65104, 65112
]

# Opis parametara (poznati)
PARAM_DESCRIPTIONS = {
    64007: "*** SETPOINT CONTROLLER ADDRESS *** (0xFF=none, 0x1E=AIO)",
    65080: "Address Claim source address",
    65083: "Steering Mode",
    65086: "Vehicle Speed Source",
    65099: "Steering Wheel Sensor SA",
    65100: "High Priority Steering Device SA",
    65101: "Low Priority Steering Device SA",
    65104: "High Priority Setpoint Controller SA",
    65112: "Operator Presence Source",
}

DURATION = 30   # sekundi (dovoljno za sve odgovore + retry)
REQUEST_DELAY = 0.2   # razmak izmedju requesta (200ms)


def main():
    print("=" * 70)
    print("   PVED PARAMETER READER (Tony-style)")
    print("=" * 70)
    print()
    print("   Sta radi:")
    print(f"     1. Salje {len(PARAMS_TO_READ)} Read Parameter requesta")
    print(f"     2. Slusa odgovore (Danfoss proprietary format)")
    print(f"     3. Dekodirat ce vrijednosti i pokazati")
    print()
    print("   SAMO CITA - nista ne mijenja u PVED-u.")
    print()
    print(f"   TX ID: 0x{READ_REQUEST_ID:08X}")
    print(f"   Trajanje: {DURATION}s")
    print()
    print("   PROVJERI:")
    print("     - PCAN na pin 1/3 MCD")
    print("     - Motor radi (PVED operational)")
    print("     - 250 kbps")
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
    log_path = os.path.abspath(f"pved_read_{ts}.log")
    rep_path = os.path.abspath(f"pved_read_{ts}_report.txt")

    log_fp = open(log_path, "w", encoding="utf-8")
    log_fp.write(f"# PVED parameter read  {datetime.now()}\n")

    # State
    responses = {}      # param_id -> (value, raw_data)
    all_responses_raw = []   # za debug
    sent_count = 0
    stop_flag = threading.Event()

    # ----- TX thread: salji requeste -----
    def tx_thread():
        nonlocal sent_count
        time.sleep(1)   # cekaj sniff da se ustabili
        
        for param in PARAMS_TO_READ:
            if stop_flag.is_set():
                break
            try:
                payload = bytes([param & 0xFF, (param >> 8) & 0xFF, 0xFF])
                msg = can.Message(arbitration_id=READ_REQUEST_ID,
                                  data=payload, is_extended_id=True)
                bus.send(msg)
                sent_count += 1
                log_fp.write(f"({time.time():.6f}) TX {READ_REQUEST_ID:08X}#{payload.hex().upper()}  "
                             f"# request param {param}\n")
                print(f"   [TX] Request param {param} ({hex(param)})")
            except Exception as e:
                print(f"   TX greska za param {param}: {e}")
            time.sleep(REQUEST_DELAY)
        
        print(f"\n   Svi requesti poslani. Cekam odgovore jos {DURATION-len(PARAMS_TO_READ)*REQUEST_DELAY:.0f}s...\n")

    tx = threading.Thread(target=tx_thread, daemon=True)
    tx.start()

    # ----- Main RX loop -----
    start_time = time.time()
    
    try:
        while (time.time() - start_time) < DURATION:
            msg = bus.recv(timeout=0.1)
            if msg is None:
                continue
            
            data_hex = msg.data.hex().upper()
            log_fp.write(f"({msg.timestamp:.6f}) RX {msg.arbitration_id:08X}"
                         f"{'X' if msg.is_extended_id else ''}#{data_hex}\n")
            
            # Iz Tony-jevog koda:
            # Odgovor pocinje s [0x1C, 0xFA, param_lo, param_hi, value_lo, value_hi, ...]
            if len(msg.data) >= 6 and msg.data[0] == 0x1C and msg.data[1] == 0xFA:
                param = msg.data[2] | (msg.data[3] << 8)
                value = msg.data[4] | (msg.data[5] << 8)
                
                if param not in responses:
                    responses[param] = (value, data_hex)
                    elapsed = time.time() - start_time
                    desc = PARAM_DESCRIPTIONS.get(param, "")
                    print(f"   [RX +{elapsed:5.1f}s] *** PARAM {param} = 0x{value:04X} ({value})  {desc}")
                    log_fp.write(f"# PARAM RESPONSE: param={param} value={value} (0x{value:04X})\n")
                
                all_responses_raw.append((time.time()-start_time, param, value, data_hex))

    except KeyboardInterrupt:
        print("\n   Prekid")
    finally:
        stop_flag.set()
        time.sleep(0.2)
        log_fp.close()
        try:
            bus.shutdown()
        except Exception:
            pass

    # ===== REPORT =====
    print()
    print("=" * 70)
    print("   GOTOVO")
    print("=" * 70)
    print(f"   Poslano requesta: {sent_count}")
    print(f"   Primljeno odgovora: {len(responses)}")
    print()

    with open(rep_path, "w", encoding="utf-8") as f:
        f.write(f"PVED Parameter Read Report  {datetime.now()}\n")
        f.write(f"Bitrate: {BITRATE}\n")
        f.write(f"TX ID: 0x{READ_REQUEST_ID:08X}\n\n")
        f.write(f"Poslano requesta: {sent_count}\n")
        f.write(f"Primljeno odgovora: {len(responses)}\n\n")
        
        f.write("=" * 60 + "\n")
        f.write("REZULTATI PO PARAMETRU\n")
        f.write("=" * 60 + "\n\n")
        f.write(f"{'Param':>8} {'Hex':>8} {'Dec':>8}  Opis\n")
        f.write("-" * 60 + "\n")
        
        for param in PARAMS_TO_READ:
            desc = PARAM_DESCRIPTIONS.get(param, "")
            if param in responses:
                value, raw = responses[param]
                marker = " ***" if param == 64007 else ""
                f.write(f"{param:8d}  0x{value:04X}  {value:6d}  {desc}{marker}\n")
            else:
                f.write(f"{param:8d}    ----    ----  {desc}  (nema odgovora)\n")
        
        f.write("\n" + "=" * 60 + "\n")
        f.write("KLJUCNI PARAMETAR: 64007 - Setpoint Controller Address\n")
        f.write("=" * 60 + "\n")
        if 64007 in responses:
            val = responses[64007][0]
            f.write(f"\nTrenutna vrijednost: 0x{val:04X} ({val})\n\n")
            if val == 0xFF or val == 0xFFFF:
                f.write("** NIJEDAN SETPOINT CONTROLLER NIJE DODIJELJEN **\n")
                f.write("PVED ce IGNORIRATI sve guidance master-e dok se ovo ne promijeni.\n\n")
                f.write("Da bi prihvatio AOG/Tony's pločicu, treba zapisati 0x1E.\n")
                f.write("(Tony-jeva pločica to radi automatski preko pvedWriteParam funkcije.)\n")
            elif val == 0x1E:
                f.write("** AIO/Tony's pločica vec dodijeljena kao master! **\n")
                f.write("PVED ce prihvatiti komande s SA 0x1E.\n")
            else:
                f.write(f"Vrijednost 0x{val:04X} - neki drugi controller je dodijeljen.\n")
                f.write("Treba istraziti koji je to ECU.\n")
        else:
            f.write("\nNema odgovora za parametar 64007.\n")
            f.write("Moguce je da PVED ne razumije nas Read Request format,\n")
            f.write("ili da je ovaj Read mechanizam blokiran u tvornickoj konfiguraciji.\n")
        
        f.write("\n" + "=" * 60 + "\n")
        f.write("DRUGI VAZNI PARAMETRI\n")
        f.write("=" * 60 + "\n")
        for p in [65080, 65083, 65099, 65100, 65101, 65104]:
            if p in responses:
                val = responses[p][0]
                desc = PARAM_DESCRIPTIONS.get(p, "")
                f.write(f"  {p}: 0x{val:04X} ({val})  {desc}\n")
        
        f.write("\n" + "=" * 60 + "\n")
        f.write("RAW LOG SVIH ODGOVORA\n")
        f.write("=" * 60 + "\n")
        for t, param, value, raw in all_responses_raw:
            f.write(f"  t={t:5.2f}s  param={param:5d}  value=0x{value:04X}  raw={raw}\n")

    print(f"   Log:    {log_path}")
    print(f"   Report: {rep_path}")
    print()
    print("   Posalji oba fajla.")
    input("\n   ENTER za izlaz...")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\nGRESKA: {e}")
        import traceback
        traceback.print_exc()
        input("\nENTER za izlaz...")
