# Wheel Calib Bridge

Laptop bridge between the **ESP32 wheel-IMU** (`WheelCalib_ESP32`) and the **Teensy** AIO module,
for Keya WAS auto-calibration (dead zone + per-side ticks/deg).

```
ESP32 (TM171, soft-AP 192.168.4.1) --WiFi UDP :9000--> bridge.py --Ethernet UDP PGN 0xD6 :8888--> Teensy
```

## Hardware
- ESP32 dev board + a wheel-mounted IMU — flash `WheelCalib_ESP32`. The ESP
  **auto-detects** which sensor is connected:
  - **BNO085** on I2C (SDA=GPIO21, SCL=GPIO22) — preferred if present
  - **TM171** on UART2 (RX=GPIO16, TX=GPIO17, 115200)
- Mount the IMU **flat on the steered wheel** so roll & pitch ≈ 0; the steer angle is then the **yaw**.
- BNO085 driver (`BNO08x_AOG.h/.cpp`, the same SparkFun BNO080 fork the Teensy uses)
  is bundled in the `WheelCalib_ESP32/` folder — no Library Manager install needed.

## Network (no static IP needed)
1. Connect the **laptop WiFi** to the ESP32 access point:
   - SSID `WheelCalib`, password `calib1234` (ESP becomes 192.168.4.1).
2. Keep the laptop **Ethernet** on the AgOpenGPS / Teensy network (e.g. 192.168.5.x).
3. The app receives the IMU over WiFi and sends the reference angle to the Teensy over Ethernet.

## Run
```
python bridge.py
```
- Watch **Roll / Pitch** → adjust the sensor mount until both are near 0 (flat).
- Press **Zero yaw** with the wheels pointing straight ahead.
- Enter the **Teensy IP** (same as in AgIO / the AIO web page), tick **Forward yaw to Teensy**.
- In the AIO web GUI → Keya tab → *auto-calibration*, "Reference IMU link" should turn **OK**.
- Then run the motorized calibration from the web GUI.

Standard library only (uses `tkinter`). No pip installs.

## No IMU yet?
The ESP32 streams **simulated** slowly-varying values when no IMU is detected,
so you can verify the whole chain (ESP → bridge → Teensy → AIO web GUI) without an
IMU. The bridge shows **"no IMU detected (simulated values)"** and the AIO reference
field still moves. Connect a BNO085 or TM171 for real data — the bridge then shows
which one is in use (e.g. **"IMU: BNO085 OK"**).

## Link test
Tick **"Force simulation (clean sine — link test)"** to make the ESP send a clean,
deterministic sinusoid through the whole chain even with a real IMU connected. Watch
the Teensy web graph (signal *Keya reference IMU angle*) — a smooth sine end-to-end
means the ESP→bridge→Teensy link has no drops/jitter; gaps or steps reveal where
packets are being lost.

## Protocol
- ESP32 → app (UDP :9000, broadcast): text `roll,pitch,yaw,imuOk,sensor,rssi\n` at 50 Hz
  (`imuOk` 1 = real IMU, 0 = simulated; `sensor` 0 = none/sim, 1 = TM171, 2 = BNO085,
  3 = forced-sim; `rssi` = laptop signal in dBm, 0 = n/a). The bridge shows the WiFi
  signal plus the **link speed** as received packets/sec (≈50 = healthy, lower = drops).
- app → ESP32 (UDP :9001): `SIM1` / `SIM0` to force the test sinusoid on/off
  (resent ~1 Hz so a dropped command self-heals).
- app → Teensy (UDP :8888): PGN `0xD6` = `80 81 7F D6 03 <angleLo> <angleHi> <valid> <crc>`,
  angle = int16 `yaw*100` (deg), crc = sum of bytes [2..N-2] & 0xFF.
