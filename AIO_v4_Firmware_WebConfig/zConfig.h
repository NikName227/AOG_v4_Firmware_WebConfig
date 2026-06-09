#pragma once
#include <Arduino.h>
#include <EEPROM.h>

// ── Shared GPS auto-zero types (used by Keya WAS + IMU-as-WAS) ──────────────────
// Defined here (early header) so the Arduino auto-prototype for gpsDriftAutoZero()
// sees them — the generated prototype is inserted above the sketch's own code.
struct AzCfg {
    bool     enable;
    float    beta;          // gentle correction fraction (engaged)
    float    speedMin;      // km/h, below this auto-zero is off
    float    yawMax;        // deg/s, "driving straight" threshold
    float    speedSlow;     // km/h
    float    speedFast;     // km/h
    uint16_t timeSlowMs;    // straight time required at/below speedSlow
    uint16_t timeFastMs;    // straight time required at/above speedFast
};
struct AzState {
    double        diffSum  = 0;
    uint32_t      diffCnt  = 0;
    elapsedMillis window;
    elapsedMillis cooldown = elapsedMillis(5000);   // ready at boot
};

// ── EEPROM layout ──────────────────────────────────────────────────────────────
// addr  0  : EEP_Ident (uint16)   – steer settings identity (existing)
// addr 10  : steerSettings        – 11 bytes (existing)
// addr 40  : steerConfig          –  9 bytes (existing)
// addr 60  : networkAddress       –  3 bytes (existing)
// addr 80  : ModuleConfig         (NEW)

#define FW_VERSION "v0.9-proto"   // shown in web GUI; bump before tagging a release branch

#define EEP_MODULE_ADDR  80
#define EEP_MODULE_IDENT 0xCD   // change to force EEPROM reset on next boot

// Free-text setup note, stored well past ModuleConfig (~250 B, ends ~330).
// Teensy 4.1 EEPROM is 4284 B total → 1024..2025 leaves huge margin both ways.
#define EEP_NOTE_ADDR    1024
#define EEP_NOTE_MAX     1000           // characters (buffer is +1 for the null)
extern char setupNote[EEP_NOTE_MAX + 1];

// IMU type
#define IMU_AUTO    0   // auto-detect: RVC → I2C → TM171
#define IMU_BNO_RVC 1   // force Serial BNO085 RVC only
#define IMU_BNO_I2C 2   // force I2C BNO085 only
#define IMU_TM171   3   // force TM171 only
#define IMU_NONE    4   // no IMU

// CAN port function modes  (same set for CAN1, CAN2, CAN3)
#define CAN_MODE_OFF    0   // port disabled
#define CAN_MODE_KEYA   1   // Keya brushless motor drive
#define CAN_MODE_IMU    2   // wheel-mounted IMU WAS – sends yaw via CAN
#define CAN_MODE_VBUS   3   // steer-ready tractor valve – V_Bus
#define CAN_MODE_KBUS   4   // Fendt K_Bus engage signals
#define CAN_MODE_ISO    5   // ISO_Bus engage + hitch signals
#define CAN_MODE_J1939  6   // J1939/NMEA 2000 GPS broadcast
#define CAN_MODE_CANTEST 7  // CAN loopback test — echoes RX back with data+1
#define CAN_MODE_CUSTOM  8  // reserved / user-defined

// WAS sensor source
#define WAS_SOURCE_ADS1115   0   // analog sensor via ADS1115 (default)
#define WAS_SOURCE_KEYA      1   // Keya motor encoder
#define WAS_SOURCE_IMU_CAN   2   // wheel-mounted IMU via CAN_MODE_IMU port
#define WAS_SOURCE_CAN_VALVE 3   // tractor valve estCurve via CAN_MODE_VBUS port

// Roll source
#define ROLL_SRC_IMU  0   // roll from active IMU (BNO085 / TM171)
#define ROLL_SRC_HPR  1   // roll from dual GPS HPR sentence or RELPOS

// Heading source
#define HDG_SRC_IMU    0  // heading from active IMU
#define HDG_SRC_HPR    1  // heading from dual GPS HPR NMEA sentence
#define HDG_SRC_RELPOS 2  // heading from UBX RELPOS (u-blox dual GPS)

// NMEA sentence type sent to AgIO
#define NMEA_TYPE_PANDA 0
#define NMEA_TYPE_PAOGI 1

// Disengage type
#define DIS_MOTOR_SPEED  0   // speed-direction detection (PWM motor)
#define DIS_KEYA_EASY    1   // Keya easy-disengage via CAN error
#define DIS_CURRENT_ADC  2   // ADC current sensor (original)
#define DIS_TRACTOR_CAN  3   // future: disengage from tractor CAN

struct ModuleConfig {
    uint8_t ident           = EEP_MODULE_IDENT;
    uint8_t imuType         = IMU_AUTO;
    // ── CAN port modes + baud rates ─────────────────────────────────────────
    uint8_t  can1Mode       = CAN_MODE_OFF;   // CAN1 function (see CAN_MODE_*)
    uint8_t  can2Mode       = CAN_MODE_OFF;   // CAN2 function
    uint8_t  can3Mode       = CAN_MODE_OFF;   // CAN3 function
    uint32_t can1Baud       = 250000;
    uint32_t can2Baud       = 250000;
    uint32_t can3Baud       = 250000;
    uint8_t  wasSource      = WAS_SOURCE_ADS1115; // active WAS sensor (see WAS_SOURCE_*)
    uint8_t  rollSource     = ROLL_SRC_IMU;       // roll data source (see ROLL_SRC_*)
    uint8_t  headingSource  = HDG_SRC_IMU;        // heading data source (see HDG_SRC_*)
    uint8_t  nmeaType       = NMEA_TYPE_PANDA;    // NMEA sentence type to AgIO
    float    yawRateFilter  = 0.3f;               // EMA on auto-zero yaw rate (0=off, 0.3=medium)
    uint8_t  gpsSerial      = 7;                  // GPS receiver hardware serial (Serial1-8)
    uint8_t  tm171Serial    = 2;                  // TM171 IMU hardware serial (Serial1-8)
    uint32_t tm171Baud      = 115200;             // TM171 baud rate
    uint8_t steerBrand      = 1;              // steer-ready brand: 0=Claas 1=Valtra 2=CaseIH
                                              //   3=Fendt 4=JCB 5=FendtOne 6=Lindner 7=AgOpenGPS
    uint8_t disengageType   = DIS_MOTOR_SPEED;
    uint8_t debugFlags      = 0;              // bitmask, see DBG_* defines below
    // ── Keya speed-direction disengage ──────────────────────────────────────
    uint8_t keyaDisEnable   = 0;    // 0=off 1=on
    uint8_t keyaSetSpeedMin = 10;   // abs(setSpeed) threshold
    uint8_t keyaActSpeedMin = 5;    // abs(actSpeed) threshold
    // ── Motor (PWM) speed-direction disengage ───────────────────────────────
    uint8_t motorDisEnable     = 0; // 0=off 1=on
    uint8_t motorAngleErrorMin = 3; // abs(steerAngleError) threshold in degrees
    uint32_t gpsBaud           = 115200;
    uint16_t speedDiffTimeout  = 250;   // Keya + Motor speedDiff disengage timeout (ms)
    // ── Keya encoder as WAS ─────────────────────────────────────────────────
    float    keyaTicksPerDeg  = 24.0f;
    uint8_t  keyaEncInvert    = 1;      // default: typical motor mount steers correct way inverted
    int32_t  keyaZeroTicks    = 0;      // int32 — matches cumulative encoder accumulator
    uint8_t  keyaAzEnable     = 1;
    float    keyaAzBeta       = 0.05f;
    float    keyaAzSpeedMin   = 2.5f;   // Flodu default — below this auto-zero blocked
    float    keyaAzYawMax     = 0.3f;   // Flodu default — stricter straight detection
    uint8_t  keyaAzSpeedSlow  = 3;
    float    keyaAzSpeedFast  = 12.0f;  // above this → fast (timeFast) applies
    float    wheelBase        = 3.20f;  // tractor wheelbase (m) for GPS bicycle-model wheel angle
    uint16_t keyaAzTimeSlowMs = 500;
    uint16_t keyaAzTimeFastMs = 200;
    float    keyaEmaAlpha     = 0.0f;
    // ── Keya steering geometry (hydraulic backlash + asymmetry) ─────────────────
    float    keyaDeadZone     = 0.0f;   // backlash on direction reversal (deg)
    float    keyaTicksLeft    = 0.0f;   // ticks/deg when steering left  (0 = use keyaTicksPerDeg)
    float    keyaTicksRight   = 0.0f;   // ticks/deg when steering right (0 = use keyaTicksPerDeg)
    float    keyaMaxAngleLeft  = 0.0f;  // working max steer angle left  (deg, 0 = no limit)
    float    keyaMaxAngleRight = 0.0f;  // working max steer angle right (deg, 0 = no limit)
    // ── J1939 / NMEA 2000 GPS broadcast ─────────────────────────────────────
    uint8_t  j1939SrcAddr    = 0x1E;  // J1939 source address (30 = default AIO)
    uint8_t  j1939En65267    = 1;     // enable PGN 65267/65256 (position + direction)
    uint8_t  j1939En129029   = 0;     // enable PGN 129029 (NMEA 2000 fast-packet)
    uint16_t j1939Rate65267  = 200;   // send interval ms (default 5 Hz)
    uint16_t j1939Rate129029 = 1000;  // send interval ms (default 1 Hz)
    // ── IMU as WAS (wheel-mounted IMU via CAN @ ID 0x300) ───────────────────────
    uint8_t  imuWasInvert    = 0;       // flip sign of measured wheel angle
    float    imuWasCpdScale  = 1.0f;    // sensitivity scale factor
    uint8_t  imuWasAzEnable  = 1;       // auto-zero (nudges offset toward 0 when straight)
    float    imuWasAzBeta    = 0.05f;   // auto-zero step toward 0
    float    imuWasSpeedMin  = 1.0f;    // min GPS speed km/h for auto-zero
    float    imuWasYawMax    = 0.8f;    // max chassis yaw rate deg/s for straight detection
    float    imuWasAzDeltaMax = 20.0f;  // auto-zero only if |steer| below this (deg)
    uint16_t imuWasAzTimeMs   = 300;    // straight time before a correction (ms)
    // Per-side drift compensation (deg of integrator error per 360 deg of rotation)
    float    imuWasChDriftL  = -0.1f;   // chassis IMU, turning left
    float    imuWasChDriftR  = -0.1f;   // chassis IMU, turning right
    float    imuWasKnDriftL  =  0.1f;   // knuckle IMU, turning left
    float    imuWasKnDriftR  =  0.1f;   // knuckle IMU, turning right
    // ── ADS1115 analog WAS auto-zero (slow nudge toward 0, persisted in EEPROM) ──
    uint8_t  adsAzEnable    = 0;        // 0=off 1=on
    float    adsAzBeta      = 0.02f;    // very slow correction toward 0
    float    adsAzSpeedMin  = 3.0f;     // min GPS speed km/h (higher = stricter, like Keya)
    float    adsAzYawMax    = 0.5f;     // max yaw rate deg/s for "straight" (stricter)
    float    adsAzDeltaMax  = 10.0f;    // only correct if |angle| below this (deg)
    uint16_t adsAzTimeMs    = 1000;     // straight time required before a correction (ms)
    float    adsAutoOffset  = 0.0f;     // persisted auto-zero offset (deg)
    // ── PVED tool ────────────────────────────────────────────────────────────
    uint16_t pvedParam64007Factory = 0xFFFF;  // original tractor value (0xFFFF = never read)
    // ── Custom CAN engage (mask+match on a user-defined frame) ──────────────────
    uint8_t  customEngageEnable  = 0;          // 0=off 1=on
    uint8_t  customEngageCanPort = 1;          // physical CAN 1/2/3 to listen on
    uint8_t  customEngageExt     = 1;          // 1=extended 29-bit, 0=standard 11-bit
    uint8_t  customEngageMode    = 0;          // 0=toggle (momentary button), 1=level (latched switch)
    uint32_t customEngageId      = 0;          // CAN ID to match
    uint8_t  customEngageMatch[8] = {0,0,0,0,0,0,0,0};  // expected byte values
    uint8_t  customEngageMask[8]  = {0,0,0,0,0,0,0,0};   // per-byte mask (0=ignore byte)
};
extern ModuleConfig moduleConfig;

// Debug flag bitmask
#define DBG_GPS        (moduleConfig.debugFlags & 0x01)
#define DBG_IMU        (moduleConfig.debugFlags & 0x02)
#define DBG_WAS        (moduleConfig.debugFlags & 0x04)
#define DBG_STEER      (moduleConfig.debugFlags & 0x08)
#define DBG_CAN        (moduleConfig.debugFlags & 0x10)
#define DBG_KEYA_DIFF  (moduleConfig.debugFlags & 0x20)
#define DBG_MOTOR_DIFF (moduleConfig.debugFlags & 0x40)
#define DBG_DISENGAGE  (moduleConfig.debugFlags & 0x80)   // log who triggered autosteer disengage

inline void moduleConfigLoad()
{
    uint8_t ident;
    EEPROM.get(EEP_MODULE_ADDR, ident);
    if (ident == EEP_MODULE_IDENT) {
        EEPROM.get(EEP_MODULE_ADDR, moduleConfig);
        EEPROM.get(EEP_NOTE_ADDR, setupNote);
        setupNote[EEP_NOTE_MAX] = 0;        // guarantee null-terminated
        Serial.println("ModuleConfig: loaded from EEPROM");
    } else {
        EEPROM.put(EEP_MODULE_ADDR, moduleConfig);
        setupNote[0] = 0;                   // first boot → empty note
        EEPROM.put(EEP_NOTE_ADDR, setupNote);
        Serial.println("ModuleConfig: first boot, defaults written");
    }
    Serial.print("  imuType=");   Serial.print(moduleConfig.imuType);
    Serial.printf("  CAN1=%u@%luk", moduleConfig.can1Mode, moduleConfig.can1Baud/1000);
    Serial.printf("  CAN2=%u@%luk", moduleConfig.can2Mode, moduleConfig.can2Baud/1000);
    Serial.printf("  CAN3=%u@%luk", moduleConfig.can3Mode, moduleConfig.can3Baud/1000);
    Serial.print("  wasSrc=");    Serial.print(moduleConfig.wasSource);
    Serial.print("  brand=");     Serial.print(moduleConfig.steerBrand);
    Serial.print("  dis=");       Serial.println(moduleConfig.disengageType);
}

inline void moduleConfigSave()
{
    EEPROM.put(EEP_MODULE_ADDR, moduleConfig);
}

inline void setupNoteSave()
{
    setupNote[EEP_NOTE_MAX] = 0;
    EEPROM.put(EEP_NOTE_ADDR, setupNote);
}

// ── Web log helpers (defined in zWebServer.ino) ────────────────────────────────
void webLog(const char* msg);
void webLogf(const char* fmt, ...);
void gpsRawByte(uint8_t c);
void disengageLog(const char* reason);   // log autosteer disengage source (DBG_DISENGAGE)

// SLOG – print to USB Serial AND buffer for web display
#define SLOG(msg)  do { Serial.println(msg); webLog(msg); } while(0)
