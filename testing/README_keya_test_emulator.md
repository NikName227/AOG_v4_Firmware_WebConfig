# Keya WAS test emulator

Bench-test the Keya **calibration** (wheel→bicycle angle) and the **encoder
direction** logic with no real Keya motor and no wheel IMU. One virtual steering
slider drives both sensors through the wheel geometry you enter.

```
  slider (bike angle)
        │
        ├─ ticks = k · bikeAngle ──────────► CAN  0x07000001 (ext)  ─► Teensy Keya port
        └─ wheel = bikeToWheel(bike,L,T) ──► UDP  PGN 0xD6 :8888     ─► Teensy (reference IMU)
```

The firmware should read the right-wheel IMU angle, convert it back to the bike
angle with its **own** `wheelToBike(L,T)`, fit it against the ticks and recover
`k = ticks/deg(bike)` with **RMS ≈ 0** on both sides. Wrong geometry or a sign
error shows up as mismatched L/R slopes, high RMS, or a flipped angle.

## Hardware
- **PEAK PCAN-USB** on the Teensy's **Keya CAN port** (the port set to *Keya* mode
  in WebConfig). Match the CAN bitrate to that port (default **250000**).
- Ethernet/UDP path to the Teensy (same network as the WheelCalib bridge), so the
  PGN reaches `:8888`. Set the Teensy IP in the GUI.

## Install / run
```
pip install python-can         # PCAN driver must be installed
python keya_test_emulator.py
```

## Use
1. Enter **Teensy IP**, **PCAN channel** (e.g. `PCAN_USBBUS1`), **CAN bitrate**.
2. Enter the **same L and T** as in WebConfig → Keya calibration, a **max bike
   angle**, and a **ticks/deg (bike)** ground-truth value (e.g. 24).
3. **Connect & run**. The Teensy should now show *Keya detected* and the reference
   IMU link as OK.
4. **Direction check**: move the slider RIGHT → the firmware Live angle must go
   **positive**. If it goes negative, either tick **Reverse tick direction** here
   or enable *Invert encoder* in WebConfig (this is the exact logic under test).
5. **Geometry check**: run the WebConfig calibration:
   - *Range A — IMU sweep*: Start, sweep the slider full-right then full-left a few
     times, Stop & Compute. The recovered **bike ticks/deg L and R** should both be
     ≈ your entered `k`, with **RMS ≈ 0**. The **wheel ticks/deg** L|R will differ
     from each other (Ackermann) while the **bike** values match — that proves the
     conversion works.
   - *Range B — manual*: at each lock read the **wheel angle** shown in this tool,
     type it into WebConfig, Capture. The bike ticks/deg should again ≈ `k`.

## Notes
- Ticks are proportional to the **bike** angle (the column drives the bike angle
  ~linearly); the per-wheel Ackermann nonlinearity lives only in the IMU/wheel
  angle, which is the whole point of the conversion.
- Speed bytes in the heartbeat are sent as 0 (calibration runs with the motor off).
- The tool only sends; it does not act on the enable/speed commands the Teensy
  sends back to the Keya — not needed for calibration.
