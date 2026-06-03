# AIO v4 WebConfig — AgOpenGPS All-In-One Firmware (Teensy 4.1)

A feature-rich AgOpenGPS AIO v4 firmware for the Teensy 4.1, with a built-in
**web configuration GUI** (no recompiling to change settings), broad steering
support (Keya CAN motor, PWM, and steer-ready CAN valves of all major brands),
flexible GPS/IMU data sourcing, live data + plotting, and a motorized Keya WAS
auto-calibration rig.

Everything is configured from a browser over Ethernet — settings persist in EEPROM.

---

## Hardware / platform

- **MCU:** Teensy 4.1 (IMXRT1062, 600 MHz, clocked to 450 MHz)
- **Net:** NativeEthernet (wired) — AgIO + web GUI on the same link
- **CAN:** 3× FlexCAN_T4 ports (CAN1/CAN2/CAN3), each freely assignable
- **IMU:** BNO085 (I2C or RVC/UART), TM171 (UART)
- **WAS:** ADS1115 analog, Keya encoder, IMU-via-CAN, or CAN valve feedback
- **GPS:** single or dual (UM98x `$HPR`, or u-blox F9P `RELPOS`)

Resource use (typical build): FLASH ~6 %, RAM1 ~69 % (≈164 KB stack free),
RAM2 ~2 % — plenty of headroom.

---

## Repository layout

```
AIO_v4_WebConfig/                 ← repo root (git)
├── AIO_v4_Firmware_WebConfig/    ← the Teensy sketch (open this in Arduino IDE)
├── WheelCalib_ESP32/             ← ESP32 wheel-IMU firmware (calibration reference)
├── WheelCalib_Bridge/            ← laptop Python bridge (ESP32 → Teensy)
├── testing/                      ← Python CAN/HTTP test scripts
└── doc/                          ← protocol notes
```

### Sketch files (`AIO_v4_Firmware_WebConfig/`)

| File | Responsibility |
|---|---|
| `AIO_v4_Firmware_WebConfig.ino` | serial/port setup, globals, `setup()`, main `loop()`, CAN dispatch helpers |
| `zConfig.h` | `ModuleConfig` struct + EEPROM load/save, debug-flag bitmask, enums |
| `Autosteer.ino` | autosteer loop, WAS-source switch, engage logic, UDP/PGN receive, `SendUdp`, PGN 221 |
| `AutosteerPID.ino` | steering PID + `motorDrive()` (PWM or `SteerKeya`) |
| `zCANBUS.ino` | `CAN_Setup`, Keya drive/heartbeat/encoder, CANtest, custom engage, address-claim call |
| `zCANSteer.ino` | brand V_Bus/ISO/K_Bus steer + engage, PVED-CL tool, J1939 address claim |
| `zCAN1.ino` | IMU-as-WAS (`CAN1_Receive`, ID 0x300) |
| `zCalib.ino` | motorized Keya WAS auto-calibration state machine |
| `zKeyaCfg.ino` | Keya motor parameterization over CAN (Max Current, Speed Kp/Ki, CAN bitrate) |
| `zHandlers.ino` | GGA/VTG/HPR parsing, `BuildNmea`, IMU readers, `updateGpsMotion` |
| `zRelPos.ino` | u-blox UBX RELPOS dual-heading decode |
| `zEthernet.ino` | Ethernet/UDP bring-up |
| `zJ1939.ino` | J1939 / NMEA 2000 GPS broadcast |
| `zWebServer.ino` | HTML page (flash), all `/api/*` endpoints, web logic |
| `TM171_IMU.*`, `BNO08x_AOG.*`, `BNO_RVC.*`, `zADS1115.*`, `zNMEAParser.h` | sensor drivers / parser |

---

## Architecture

### Main loop (non-blocking)
`loop()` drains the GPS/RTK/RELPOS serials each pass (full buffer, not 1 byte),
services the IMUs, runs CAN receivers, samples the graph buffer, runs the
autosteer loop, and services the web server. Loop period is measurable live
(see Live → Graph → *Perf loop time*).

### CAN dispatch
Physical ports are decoupled from function. Each port (`can1/2/3Mode`) is mapped
to a *function mode* — `OFF, KEYA, IMU-WAS, V_Bus, K_Bus, ISO, J1939, CANtest,
Custom`. Code reads/writes by **mode** via `canRead(mode)` / `canWrite(mode)` /
`hasFuncMode(mode)`, so the same logic works regardless of which physical port a
function is wired to. `canReadPort(n)` reads a raw physical port (used by custom
engage).

### Web server model (RT-friendly)
- The full ~HTML page is served **once**; thereafter the browser polls compact
  JSON, so an open browser barely touches real-time performance.
- `/api/status` (full config) is fetched once on load; `/api/live` (small) every
  2 s; `/api/grp?g=N` (one group) every 0.5 s on the Live tab.
- Request read / header timeouts are 50 ms; API responses use immediate
  `client.stop()`. Worst-case loop block per request ≈ 100 ms (was ~1.5 s).

### Configuration / EEPROM
All settings live in one `ModuleConfig` struct, saved to EEPROM. A one-byte
`EEP_MODULE_IDENT` guards the layout — bump it whenever the struct changes and
the next boot rewrites defaults. Restart-required changes (serial/CAN/data
source) trigger a deferred reboot from the web server.

---

## Integrated features

### GPS / heading / roll — selectable data sources
- **Data Source** card lets you pick, per channel:
  - **WAS:** ADS1115 / Keya encoder / IMU-via-CAN / CAN valve
  - **Roll:** IMU (BNO085/TM171) or HPR (dual GPS)
  - **Heading:** IMU / HPR (UM980/UM982) / RELPOS (dual u-blox F9P)
  - **NMEA type to AgIO:** `$PANDA` or `$PAOGI`
- Source-based routing in `GGA_Handler`/`HPR_Handler`/`relPosDecode`, with a 3 s
  timeout fallback to IMU if a configured source stops arriving (PGN 221 warning
  to AgIO).
- `updateGpsMotion()` derives vehicle yaw rate + GPS bicycle-model wheel angle
  from VTG course in all modes (used by WAS auto-zero).

### HPR RTK quality guard (UM982 second antenna)
If the main antenna has RTK but the heading antenna loses it, heading is frozen,
AgIO gets a PGN 221 warning, and after 10 s `fixQuality` is forced to 0 to
disengage. Auto-recovers when RTK returns.

### Keya CAN motor as WAS
- Cumulative `int32` encoder (overflow-safe), per-side ticks/deg, hydraulic
  **dead-zone (backlash)** compensation, EMA filter.
- **GPS auto-zero** (Flodu-derived): window-averaged correction, optional chassis
  IMU gyro stability gate, guidance-aware beta, initial-zero boot lock (autosteer
  blocked until first straight-driving zero).
- **Motorized auto-calibration** (`zCalib.ino`): with a wheel-mounted reference
  IMU the Teensy turns the motor itself to measure dead zone + per-side ticks/deg.
- **Motor parameterization** (`zKeyaCfg.ino`): read/write Max Current, Speed
  Kp/Ki, CAN bitrate; store to the drive's EEPROM. Hold-to-move wheel test.

### Steer-ready CAN (all brands)
V_Bus curve/valve + ISO_Bus + K_Bus engage for Claas, Valtra/Massey/McCormick/MF,
CaseIH/NH, Fendt, JCB, FendtOne, Lindner, AgOpenGPS, Cat/Challenger (Late/Early).
**J1939 address claim** sent at boot for true steer-ready setups
(`WAS = CAN valve`). Includes the **PVED-CL** service tool (Claas param 64007).

### Engage options
- **Brand engage:** the tractor's native engage button (V_Bus/ISO/K_Bus) toggles
  autosteer — works with a Keya motor too (engage detection is decoupled from the
  valve; listen-only, no bus writes, no address claim).
- **Custom CAN engage:** mask+match on any CAN frame with a **Learn** workflow
  (capture idle vs pressed → XOR isolates the engage bits, so other switches like
  lights in the same frame don't break the match). Toggle or level mode.

### Other
- **J1939 / NMEA 2000 GPS broadcast** (PGN 65267/65256, 129029).
- **Disengage trigger logging** (debug flag) — logs *what* cut autosteer
  (watchdog, switch, sensors, Keya/motor speed-diff, WAS range, HPR cutoff…).
- **Speed-direction disengage** for Keya and PWM motors (hand-override detect).

---

## Web GUI tabs

| Tab | Contents |
|---|---|
| **Config** | Detected devices · Data Source · Serial port assignment · CAN port assignment · AgIO settings (read-only) · motor/IMU-WAS disengage params |
| **Live** | *Values* (7 groups: GPS / IMU / WAS / Keya / Steer / CAN Steer) + *Graph* (4 signals, dual axis, freq 1–10 Hz, pause/zoom/pan, CSV export) |
| **Keya** | *Tuning* (status, steering geometry, WAS params, auto-calibration) + *Motor Config* (CAN parameters + wheel test) |
| **CAN Steer** | Brand · Custom CAN engage · live valve status · CAN scan · CAN plot · PVED tool · J1939 broadcast |
| **Debug** | Debug flags (GPS, CAN, disengage trigger) · serial log (toggleable capture) |
| **UM98x** | GPS command console · raw GPS serial log |

### Key `/api/*` endpoints
`status` (full config), `live` (compact live), `grp?g=N` (one Live group),
`save` (write config), `graphcfg`/`graphdata` (plot), `calib` (auto-cal),
`keyacfg` (Keya motor config), `canscan`, `canraw`, `pved`, `log`, `gpsraw`,
`gpscmd`, `keyazero`, `imuwaszero`.

---

## Wheel calibration rig (optional)

For Keya-encoder-as-WAS auto-calibration:

```
ESP32 + TM171 (on the steered wheel, soft-AP 192.168.4.1)
   └─ WiFi UDP ─→ laptop  WheelCalib_Bridge/bridge.py  (shows roll/pitch/yaw)
                     └─ Ethernet UDP PGN 0xD6 ─→ Teensy (reference wheel angle)
```

Mount the IMU flat (roll/pitch ≈ 0 → steer angle = yaw), connect the bridge,
then run the motorized calibration from the web GUI. See
`WheelCalib_Bridge/README.md`.

---

## Build & flash

- **Arduino IDE** with Teensyduino. Open `AIO_v4_Firmware_WebConfig/` and select
  Teensy 4.1, USB Serial, 450 MHz.
- Libraries: NativeEthernet, FlexCAN_T4, Wire, EEPROM (Teensy core).
- After flashing, the first boot resets config to defaults (EEPROM ident) — set
  everything up in the web GUI (browse to the module IP shown in AgIO / serial).
- `WheelCalib_ESP32/` is a separate Arduino sketch for an ESP32 board.
- `WheelCalib_Bridge/bridge.py` is standard-library Python (tkinter) — no installs.

---

## Notes / safety

- Motorized calibration and the Keya wheel test physically turn the steering —
  vehicle stationary, stand clear, hand ready. The Keya's own current limit
  (configurable via *Motor Config*) is the hardware backstop.
- The Keya current limit is set **in the drive** (over CAN from this firmware or
  the standalone tool), not pushed automatically at boot.
- Keya encoder-as-WAS drifts (needs auto-zero); a physical analog WAS is absolute
  and more precise — prefer it when installed.

Credits: builds on the AgOpenGPS community AIO firmware, the Keya CAN work, the
auto-zero approach from the *AIO_Keya_WasKeyaFiltre* (Flodu) project, and the
*KeyaConfig* parameterization tool.
