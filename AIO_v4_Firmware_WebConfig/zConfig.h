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

// Per-side least-squares accumulator for the Keya sweep calibration. Declared here
// (early header) so the auto-prototypes for calFit*() see the type.
// Accumulates the wheel column plus BOTH bike conversions (inner & outer); the
// inner/outer choice per side is decided at the end from the max wheel angle
// (larger = inner = right turn), so calibration is robust to encoder polarity.
struct CalFit {
    uint32_t n;
    double Sx, Sxx;          // Σtick, Σtick²
    double Sw, Swx, Sww;     // wheel deg
    double Si, Six, Sii;     // bike-if-inner
    double So, Sox, Soo;     // bike-if-outer
    double maxW;             // max |wheel| seen (inner detection)
};

// ── Variable Steering: per-side accumulator for the analog-WAS sweep fit ────────
// Fits ADS counts (x) → bike angle (y, signed) so the noisy OEM WAS can be used as
// a slow absolute reference for the Keya encoder. Filled during the same sweep that
// calibrates ticks/deg, so it costs no extra time on the tractor. Both bike columns
// are accumulated; the inner/outer choice is resolved in calStopSweep().
struct WasFit {
    uint32_t n;
    double Sx, Sxx;          // Σcounts, Σcounts²
    double Si, Six, Sii;     // bike-if-inner (magnitude)
    double So, Sox, Soo;     // bike-if-outer (magnitude)
};

// ── Variable Steering config (custom: Deutz-Fahr twin orbital + noisy OEM WAS) ──
// Kept as its own struct so it can be migrated independently: growing ModuleConfig
// would otherwise load whatever was in the untouched EEPROM tail (0xFF) into these
// fields — enabling every feature with a NaN calibration. VS_MAGIC guards that.
// Bumping the module ident instead would work, but at the cost of wiping every
// existing setting on the tractor.
//
// Encoder stays primary and carries all the dynamics; the analog WAS is only a slow
// absolute anchor. Everything that changes behaviour defaults OFF → stock firmware.
// The magic doubles as a version: it is the FIRST byte of the struct, so it reads
// back correctly no matter how the tail has grown. New fields are only ever
// APPENDED, which keeps every older image readable up to its own last field — the
// migration below then fills in just the new tail. Never insert into the middle.
#define VS_MAGIC_V1 0x5C                // v1: through disengageOnBad
#define VS_MAGIC    0x5D                // v2: + WAS zero button / slow intercept trim
struct VsConfig {
    uint8_t  magic          = VS_MAGIC; // must stay first — migration marker
    // Kill switch for everything WAS-derived: fusion, WAS zero, ratio detection.
    // OFF = Keya encoder + GPS only, i.e. the proven pre-VS behaviour. Kept so the
    // whole thing can be abandoned in the field without a reflash.
    // NOTE: the orbital ratio is deliberately NOT under this switch — a manually
    // chosen ratio is just picking which calibrated scale to use, as deterministic
    // as keyaTicksPerDeg itself, and switching it off mid-orchard would leave the
    // angle 2x wrong in 250 ccm. For truly stock behaviour set 125 ccm as well.
    // Monitoring/display stays live either way — it reads the ADS the stock
    // firmware already read, and is outside the control path.
    uint8_t  masterEnable   = 0;        // 0=off (Keya + GPS only) 1=on
    // Separate switch for a physically dead/faulty WAS. With this off nothing reads
    // the sensor — importantly including ratio detection, which on a faulty WAS
    // could otherwise flip the gain 2x or spuriously disengage autosteer.
    uint8_t  wasPresent     = 1;        // 0 = WAS sensor unavailable, ignore it entirely
    uint8_t  medianEnable   = 1;        // median-5 on raw ADS counts (VS reference only)
    float    wasDegPerCount = 0.0f;     // WAS calibration: angle = a*counts + b (0 = uncalibrated)
    float    wasIntercept   = 0.0f;     // b
    // WAS as a slow absolute anchor for the encoder (master switch, default OFF)
    uint8_t  fuseEnable     = 0;        // 0=off 1=on
    float    fuseBeta       = 0.01f;    // correction fraction per accepted sample @20 Hz (~5 s TC)
    float    gateDeg        = 4.0f;     // reject a WAS sample this far from the encoder prediction
    float    rateMaxDegS    = 0.5f;     // hard cap on how fast the WAS offset may move (deg/s)
    float    offsetMaxDeg   = 15.0f;    // clamp on the accumulated WAS offset (deg)
    // Initial zero taken from the WAS (unlocks autosteer without waiting for GPS)
    uint8_t  zeroEnable     = 0;        // 0=off (stock GPS initial zero) 1=on
    uint16_t zeroStillMs    = 2000;     // encoder must be still this long
    float    zeroStillTicks = 3.0f;     // |tick movement| below this counts as still
    float    zeroMaxDeg     = 4.0f;     // only zero while |WAS| under this (keeps it ratio-neutral)
    float    zeroSpreadDeg  = 1.0f;     // max spread of accepted samples in the window
    // Twin orbital: 125 ccm is the calibrated base, 250 ccm halves ticks/deg exactly
    uint8_t  orbitalEnable  = 0;        // 0=off (single ratio) 1=twin orbital handling
    uint8_t  orbitalMode    = 0;        // active ratio: 0 = 125 ccm (base), 1 = 250 ccm
    uint8_t  orbitalAuto    = 0;        // 0=manual only, 1=auto-switch from the estimate
    float    orbitalRatio   = 2.0f;     // displacement ratio 250/125 (exact from ccm)
    float    detectMinDeg   = 8.0f;     // WAS travel needed before an estimate counts
    uint8_t  detectConfirm  = 3;        // consecutive agreeing windows before switching
    uint8_t  disengageOnBad = 1;        // wrong ratio while engaged → drop autosteer
    // ── v2: the WAS zero itself ──────────────────────────────────────────────
    // The sweep fits angle = a*counts + b and writes both at once, so whatever
    // centring error the wheels had at "Start sweep" lands in b and stays there.
    // These move b alone; the slope a from the sweep is never touched.
    //
    // wasInterceptBase is b as last DEFINED (by a sweep or by the manual zero) and
    // is the reference the clamp is measured against — so a slow trim can never
    // walk away one small step at a time, and the drift is visible as a number.
    float    wasInterceptBase = 0.0f;
    // Slow auto-trim: a second, INDEPENDENT loop. It never looks at the encoder;
    // its only input is "I am driving straight, therefore the angle is 0", which is
    // why it cannot circle with the fusion. Runs at 1 Hz, hours-long time constant.
    uint8_t  wasTrimEnable    = 0;      // 0=off 1=on
    float    wasTrimBeta      = 0.002f; // fraction removed per accepted second (~8 min TC)
    float    wasTrimSpeedMin  = 3.0f;   // km/h
    float    wasTrimYawMax    = 0.5f;   // deg/s — stricter than the GPS auto-zero
    float    wasTrimAngleMax  = 2.0f;   // applies to |WAS| AND |steer actual|: two witnesses
    uint16_t wasTrimStraightMs= 2000;   // conditions must hold this long before trimming
    float    wasTrimMaxDeg    = 5.0f;   // hard clamp on the drift from wasInterceptBase
};

// ── EEPROM layout ──────────────────────────────────────────────────────────────
// addr  0  : EEP_Ident (uint16)   – steer settings identity (existing)
// addr 10  : steerSettings        – 11 bytes (existing)
// addr 40  : steerConfig          –  9 bytes (existing)
// addr 60  : networkAddress       –  3 bytes (existing)
// addr 80  : ModuleConfig         (NEW)

// Shown in the web GUI header — it is how you tell, standing at the tractor, which
// build is actually flashed. The LAST number is bumped on every single commit, so it
// is a commit counter, not a semantic version.
//   standard branch : v1.0.N
//   custom solution : v1.0.1-CS_1.N   base frozen (where it branched from),
//                                     CS_1 = which custom solution, N = commit
// This branch is CS_1 (variable steering) and never merges to master.
#define FW_VERSION "v1.0.1-CS_1.1"

#define EEP_MODULE_ADDR  80
#define EEP_MODULE_IDENT 0xD1   // change to force EEPROM reset on next boot

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
    float    yawRateFilter  = 0.2f;               // EMA on auto-zero yaw rate (0=off, lower=smoother)
    float    adsEmaAlpha    = 0.0f;               // EMA on ADS1115 WAS raw counts (0=off, lower=smoother)
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
    uint8_t  keyaEncInvert    = 0;      // default OFF: base logic already gives right-turn = positive
    int32_t  keyaZeroTicks    = 0;      // int32 — matches cumulative encoder accumulator
    uint8_t  keyaAzEnable     = 1;
    float    keyaAzBeta       = 0.2f;    // fraction/cycle when NOT engaged; beta/5 when engaged
    float    keyaAzSpeedMin   = 2.5f;   // Flodu default — below this auto-zero blocked
    float    keyaAzYawMax     = 0.6f;   // 0.3 too strict (hard to reach even on bench)
    float    keyaAzYawMaxInit = 5.0f;  // initial zero only — bicycle model corrects for curvature so strict limit not needed
    uint8_t  keyaAzSpeedSlow  = 5;
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
    // Calibration geometry: wheel-angle -> bicycle (virtual centre) angle conversion.
    // L = the shared `wheelBase` above; T is keya-specific:
    float    keyaTrackT        = 1.5f;  // T: track at kingpin-axis ground intersection (m)
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
    float    imuWasAzBeta    = 0.2f;    // fraction/cycle when NOT engaged; beta/5 when engaged
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
    VsConfig vs;                        // Variable Steering (custom) — see VsConfig
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
        // ── Variable Steering block migration ────────────────────────────────
        // Two different situations, and they must not be confused:
        //
        //  v1 image  — the block is real and calibrated, only the newer tail is
        //              untouched EEPROM. Wiping it would cost the WAS sweep, i.e.
        //              another session on the tractor with the reference IMU on
        //              the wheel. Fill in the new fields only.
        //  no block  — bytes are untouched flash (0xFF), which would read back as
        //              "every feature enabled" with a NaN calibration. Reset it.
        //
        // Either way the rest of ModuleConfig survives.
        if (moduleConfig.vs.magic == VS_MAGIC_V1) {
            ModuleConfig fresh;
            moduleConfig.vs.wasTrimEnable     = fresh.vs.wasTrimEnable;
            moduleConfig.vs.wasTrimBeta       = fresh.vs.wasTrimBeta;
            moduleConfig.vs.wasTrimSpeedMin   = fresh.vs.wasTrimSpeedMin;
            moduleConfig.vs.wasTrimYawMax     = fresh.vs.wasTrimYawMax;
            moduleConfig.vs.wasTrimAngleMax   = fresh.vs.wasTrimAngleMax;
            moduleConfig.vs.wasTrimStraightMs = fresh.vs.wasTrimStraightMs;
            moduleConfig.vs.wasTrimMaxDeg     = fresh.vs.wasTrimMaxDeg;
            // Nothing has ever moved the intercept in v1, so the value now stored
            // IS the sweep value — which makes it exactly the right clamp origin.
            moduleConfig.vs.wasInterceptBase  = moduleConfig.vs.wasIntercept;
            moduleConfig.vs.magic             = VS_MAGIC;
            EEPROM.put(EEP_MODULE_ADDR, moduleConfig);
            Serial.println("ModuleConfig: Variable Steering migrated v1 -> v2 (calibration kept)");
        } else if (moduleConfig.vs.magic != VS_MAGIC) {
            ModuleConfig fresh;
            moduleConfig.vs = fresh.vs;
            EEPROM.put(EEP_MODULE_ADDR, moduleConfig);
            Serial.println("ModuleConfig: Variable Steering block initialised to defaults");
        }
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

// ── Variable Steering (defined in zVarSteer.ino) ───────────────────────────────
// Declared in this early header so they are visible to Autosteer.ino, which the
// tab-concatenation puts BEFORE zVarSteer.ino — a forward decl inside zVarSteer
// would come too late. Only primitive types here, so no extra dependencies.
void  vsApplyZero(int32_t encRaw, float angleDeg);
void  vsSetOrbitalMode(uint8_t mode);
float vsTicksPerDeg();
float vsRatioDiv();
void  vsTrimUpdate();                          // slow auto-trim of the WAS intercept
bool  vsWasZeroNow(char* out, uint16_t n);     // manual "set WAS zero now" button
// Rolling "encoder still" window, published in COUNTS so it is usable before the
// WAS has any calibration at all. zCalib.ino reads these (it is concatenated
// before zVarSteer.ino, so a plain definition there would come too late).
extern float vsStillMeanCounts;                // mean ADS counts over the still window
extern float vsStillSpreadCounts;              // min→max spread over that window
extern elapsedMillis vsStillTimer;             // how long the encoder has been still

// SLOG – print to USB Serial AND buffer for web display
#define SLOG(msg)  do { Serial.println(msg); webLog(msg); } while(0)
