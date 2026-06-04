# Live data & graph signal reference

Description of every value shown on the **Live** tab (grouped live view) and every
signal selectable in the **Graph** dropdowns. Both read the same firmware variables;
the live view shows them as text at ~0.5 s, the graph samples them at 1–10 Hz.

Signal **IDs** (graph) map 1:1 to `getSignalValue()` cases in `zWebServer.ino`. The
list is ordered Gr1 → Gr6, with the group-less **Performance** signals at the end.

---

## Live data — grouped view

The Live tab has 6 group buttons. Each polls `/api/grp?g=N` (compact JSON) only
while that group is open, so an open Live view barely costs real-time performance.

### Group 1 — GPS
Parsed from the incoming NMEA / position stream.

| Row | Meaning | Unit |
|---|---|---|
| **GGA › Msg interval** | Time between consecutive `$GGA` messages (message-rate health) | ms |
| **GGA › Fix time** | UTC time of fix from GGA | hhmmss |
| **GGA › Latitude / Longitude** | Position + hemisphere (N/S, E/W) | deg |
| **GGA › Fix quality** | 0 invalid · 1 GPS · 2 DGPS · 4 RTK fix · 5 RTK float | enum |
| **GGA › Satellites** | Satellites used in the fix | count |
| **GGA › HDOP** | Horizontal dilution of precision (lower = better) | ratio |
| **GGA › Altitude** | Antenna altitude (MSL) | m |
| **GGA › DGPS age** | Age of differential/RTK corrections | s |
| **VTG › Msg interval** | Time between `$VTG` messages | ms |
| **VTG › Heading** | Course over ground (from VTG) | ° |
| **VTG › Speed** | Ground speed | km/h |
| **HPR › Msg interval** | Time between dual-antenna heading messages (`$HPR`/RELPOS) | ms |
| **HPR › Heading** | Dual-antenna (moving-base) heading | ° |
| **HPR › Roll** | Roll derived from the dual antennas | ° |
| **HPR › RTK quality** | Heading-solution quality (second antenna) | enum |

### Group 2 — IMU
Raw IMU readings, regardless of which one is selected as a data source.

| Row | Meaning | Unit |
|---|---|---|
| **BNO085 › Heading / Roll / Pitch** | Fused orientation from the BNO085 | ° |
| **BNO085 › Yaw rate** | Angular rate about the vertical axis | °/s |
| **TM171 › Heading / Roll / Pitch** | Orientation from the TM171 | ° |
| **TM171 › Yaw rate** | Angular rate about the vertical axis | °/s |

### Group 3 — WAS (wheel angle sensor)
Shows every WAS source plus the final angle, so you can compare them.

| Row | Meaning | Unit |
|---|---|---|
| **Active source** | Which WAS source is in use: ADS1115 / Keya / IMU-via-CAN / CAN valve | enum |
| **ADS1115 › Raw counts** | Raw analog WAS reading (ADS1115) | counts |
| **IMU as WAS › Raw yaw** | Yaw used as a WAS substitute (IMU-via-CAN) | ° |
| **IMU as WAS › After scale** | Raw yaw × counts-per-degree scale | ° |
| **GPS reference › Yaw rate (GPS course)** | Vehicle yaw rate derived from VTG course change | °/s |
| **GPS reference › Wheel angle (GPS)** | Wheel angle from the GPS bicycle model (used by Keya auto-zero) | ° |
| **Final › WAS angle actual** | The wheel angle actually used by autosteer / sent to AgIO | ° |

### Group 4 — Keya (CAN motor as encoder/WAS)

| Row | Meaning | Unit |
|---|---|---|
| **Detected** | A Keya drive is responding on CAN | YES/NO |
| **Initial zero** | First straight-driving auto-zero done (autosteer stays blocked until DONE) | DONE/PENDING |
| **Encoder** | Cumulative motor encoder count (overflow-safe `int32`) | ticks |
| **Rel position** | Motor movement since you opened this view / pressed **Zero**, in wheel degrees (via ticks/deg) + raw ticks. **Zero** button re-references | ° wheel + ticks |
| **Steering wheel pos** | Same movement expressed as steering-wheel rotation (1 motor turn = 65536 ticks = 360°), assumes 1:1 motor↔column | ° |
| **Zero offset** | Encoder ticks stored as the straight-ahead zero | ticks |
| **GPS drift offset** | Slow auto-zero correction applied to compensate encoder drift | ° |
| **Actual speed** | Motor speed reported by the drive | drive units |
| **Set speed** | Speed command sent to the motor | drive units |
| **Final angle** | Resulting WAS angle when Keya is the source | ° |

### Group 5 — Steer (autosteer + switches + PCB sensors)

| Row | Meaning | Unit |
|---|---|---|
| **Steer actual** | Current wheel angle | ° |
| **Steer setpoint** | Target wheel angle from AgOpenGPS | ° |
| **Steer error** | setpoint − actual | ° |
| **PWM** | Motor drive output (sign = direction) | −255…255 |
| **Autosteer** | Engaged or not | ACTIVE/OFF |
| **Speed** | Ground speed | km/h |
| **AOG switches › Steer / Work / Remote** | State of the steer button, work switch, remote | HIGH/LOW |
| **PCB sensors › Current sensor (A17)** | Raw analog current-sense reading | ADC counts |
| **PCB sensors › Pressure sensor (A10)** | Raw analog pressure reading | ADC counts |
| **PCB sensors › Sensor reading (to AOG)** | Processed sensor value reported to AgOpenGPS | value |

### Group 6 — CAN Steer (steer-ready / brand valve)

| Row | Meaning | Unit |
|---|---|---|
| **Valve ready** | Steer-ready handshake (16 = READY) | code |
| **estCurve** | Estimated curvature from the valve (raw; offset 32128 = centre) | raw |
| **setCurve** | Commanded curvature to the valve (raw; offset 32128 = centre) | raw |
| **Rear hitch** | ISOBUS rear hitch position | % |
| **Steering intend** | Tractor signalling a steering request | STEERING/IDLE |

---

## Graph signals

Pick up to 4 signals (dual Y-axis). IDs below are the firmware signal IDs.

### Gr1 — GPS
| ID | Signal | Unit | Source |
|---|---|---|---|
| 1 | GGA fixQuality | enum | `mainFixQuality` |
| 2 | GGA numSats | count | `numSats` |
| 3 | GGA HDOP | ratio | `HDOP` |
| 4 | GGA altitude | m | `altitude` |
| 5 | VTG heading | ° | `headingVTG` |
| 6 | VTG speed | km/h | `gpsSpeed` |
| 7 | HPR heading | ° | `umHeading` |
| 8 | HPR roll | ° | `umRoll` |
| 9 | HPR quality | enum | `solQualityHPR` |
| 38 | GGA interval | ms | `ggaIntervalMs` |
| 39 | VTG interval | ms | `vtgIntervalMs` |
| 40 | HPR interval | ms | `hprIntervalMs` |

### Gr2 — IMU
| ID | Signal | Unit | Source |
|---|---|---|---|
| 10 | BNO heading | ° | `yaw/10` |
| 11 | BNO roll | ° | `roll/10` |
| 12 | BNO pitch | ° | `pitch/10` |
| 13 | BNO yawRate | °/s | `imuYawRate` |
| 14 | TM171 heading | ° | TM171 |
| 15 | TM171 roll | ° | TM171 |
| 16 | TM171 pitch | ° | TM171 |

### Gr3 — WAS
| ID | Signal | Unit | Source |
|---|---|---|---|
| 17 | WAS ADS raw | counts | `steeringPosition` |
| 18 | WAS IMU raw | ° | `imuWasRawYaw` |
| 19 | WAS IMU scaled | ° | `imuWasRawYaw × cpdScale` |
| 20 | WAS chassis yawRate | °/s | `headingRate` (GPS course) |
| 21 | WAS wheelAngleGPS | ° | GPS bicycle model |
| 22 | WAS actual | ° | `steerAngleActual` (final angle) |

### Gr4 — Keya
| ID | Signal | Unit | Source |
|---|---|---|---|
| 23 | Keya encoder | ticks | `keyaEncoderRaw` (cumulative) |
| 24 | Keya gpsOffset | ° | `keyaGpsOffset` (auto-zero drift) |
| 25 | Keya actSpeed | drive units | `keyaCurrentActualSpeed` |
| 26 | Keya setSpeed | drive units | `keyaCurrentSetSpeed` |
| 44 | Keya wheel pos | ° | rel ticks ÷ ticks/deg (zeroed at view/graph start) |
| 45 | Keya steering-wheel pos | ° | rel ticks × 360/65536 (zeroed at start) |

> Tip: plot **45 (steering wheel)** against **22 / 44 (road wheel)** to see the
> steering ratio and the dead-zone/backlash directly. Both 44 and 45 are zeroed at
> the moment you open Live Gr4 or press **Set** on the graph; **Zero** re-references.

### Gr5 — Steer
| ID | Signal | Unit | Source |
|---|---|---|---|
| 27 | Steer actual | ° | `steerAngleActual` |
| 28 | Steer setpoint | ° | `steerAngleSetPoint` |
| 29 | Steer error | ° | `steerAngleError` |
| 30 | PWM | −255…255 | `pwmDisplay` |
| 31 | speed | km/h | `gpsSpeed` |
| 41 | current sensor (A17) | ADC counts | `analogRead(CURRENT_SENSOR_PIN)` |
| 42 | pressure sensor (A10) | ADC counts | `analogRead(PRESSURE_SENSOR_PIN)` |
| 43 | sensor reading | value | `sensorReading` (to AOG) |

### Gr6 — CAN Steer
| ID | Signal | Unit | Source |
|---|---|---|---|
| 32 | valveReady | code (16=ready) | `steeringValveReady` |
| 33 | estCurve | raw (32128=centre) | `estCurve` |
| 34 | setCurve | raw (32128=centre) | `setCurve` |
| 35 | hitch | raw | `ISORearHitch` |

### Performance (no group)
| ID | Signal | Unit | Source |
|---|---|---|---|
| 36 | loop time ms | ms | `loopTimeMs` (current main-loop period) |
| 37 | loop max ms | ms | `loopTimeMax` (peak loop time over a rolling ~1 s window — reset every second) |

---

## Notes
- Graph IDs are fixed — they are the contract between the JS dropdown and
  `getSignalValue()`. When adding a signal, append a new ID; do not renumber.
- Enum values (fix quality, HPR quality) are shown decoded in the Live text view
  but plotted as their raw numeric code on the graph.
- Keya **actual/set speed** are the drive's own units (not RPM-calibrated here).
