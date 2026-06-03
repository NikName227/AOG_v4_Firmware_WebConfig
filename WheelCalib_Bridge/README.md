# Wheel Calib Bridge

Laptop bridge between the **ESP32 wheel-IMU** (`WheelCalib_ESP32`) and the **Teensy** AIO module,
for Keya WAS auto-calibration (dead zone + per-side ticks/deg).

```
ESP32 (TM171, soft-AP 192.168.4.1) --WiFi UDP :9000--> bridge.py --Ethernet UDP PGN 0xD6 :8888--> Teensy
```

## Hardware
- ESP32 dev board + TM171 IMU on UART2 (RX=GPIO16, TX=GPIO17, 115200) — flash `WheelCalib_ESP32`.
- Mount the TM171 **flat on the steered wheel** so roll & pitch ≈ 0; the steer angle is then the **yaw**.

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
The ESP32 streams **simulated** slowly-varying values when no TM171 is detected,
so you can verify the whole chain (ESP → bridge → Teensy → AIO web GUI) without an
IMU. The bridge shows **"no IMU detected (simulated values)"** and the AIO reference
field still moves. Connect a real TM171 (or add a BNO085 reader) for real data.

## Protocol
- ESP32 → app (UDP :9000, broadcast): text `roll,pitch,yaw,imuOk\n` at 50 Hz
  (`imuOk` 1 = real TM171, 0 = simulated / no IMU).
- app → Teensy (UDP :8888): PGN `0xD6` = `80 81 7F D6 03 <angleLo> <angleHi> <valid> <crc>`,
  angle = int16 `yaw*100` (deg), crc = sum of bytes [2..N-2] & 0xFF.
