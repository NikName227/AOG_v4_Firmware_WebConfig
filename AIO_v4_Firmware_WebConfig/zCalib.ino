// ─────────────────────────────────────────────────────────────────────────────
// Keya WAS motorized auto-calibration
//
// Uses a wheel-mounted reference IMU (yaw via the bridge → PGN 0xD6 → refWheelAngle).
// Two phases:
//   1. DEAD ZONE — automatic: the Teensy slowly turns the Keya motor and reverses
//      it a few times to measure the hydraulic backlash on direction change.
//   2. RANGE — manual: the motor is OFF and the operator turns the wheel by hand to
//      centre / full-left / full-right, capturing each (calCapCentre/Left/Right).
//      The operator defines absolute centre, which is more precise than relying on
//      the motor's stall detection. calComputeRange() then derives ticks/deg/side.
//
// Safety: only runs stationary, autosteer off, reference link fresh. Aborts on
// steer switch, vehicle motion, or lost reference. Keya current limit (≈5 A)
// lets the operator grab the wheel to override.
// ─────────────────────────────────────────────────────────────────────────────

// Tunables
#define CAL_REACT_THRESH   2.5f     // ref must move this many deg to confirm reaction
#define CAL_REACT_TIMEOUT  9000     // ms per reaction sweep
#define CAL_DZ_CYCLES      3        // dead-zone reversal samples
#define CAL_DZ_AMPL        5.0f     // ref amplitude per cycle (deg)
#define CAL_DZ_MOVE        0.5f     // ref delta that counts as "wheel started moving"
#define CAL_STOP_SPEED     5        // |keyaActualSpeed| below this = stalled
#define CAL_STOP_HOLD      350      // ms stalled = mechanical stop reached
#define CAL_RANGE_TIMEOUT  25000    // ms per range sweep
#define CAL_MOTOR_MS       50       // motor command rate limit

static elapsedMillis calStateTimer;
static elapsedMillis calMotorTimer;
static elapsedMillis calStallTimer;
static int    calPhase;
static int32_t calEncCenter;
static float  calRefCenter;
// dead-zone accumulation
static float  calDzSum;
static int    calDzDone;
static int32_t calRevEnc;
static float  calCycleStartRef;
static int8_t calDir;
// range
static int32_t calEncRight, calEncLeft;
static float  calRefRight, calRefLeft;

static void calSet(uint8_t s, const char* m) {
    calState = s;
    strncpy(calMsg, m, sizeof(calMsg) - 1);
    calMsg[sizeof(calMsg) - 1] = 0;
    calStateTimer = 0;
    calPhase = 0;
}

static void calMotor(int spd) {
    if (calMotorTimer < CAL_MOTOR_MS) return;
    calMotorTimer = 0;
    SteerKeya(spd, spd != 0);
}

static void calStop() { SteerKeya(0, false); }

// Public: start / abort / apply (called from web handler)
void calStart() {
    if (!keyaDetected)                 { calSet(CAL_FAIL, "Keya not detected"); return; }
    if (!(refAngleTime < 1000 && refAngleValid)) { calSet(CAL_FAIL, "no reference IMU link"); return; }
    if (gpsSpeed > 0.5f)               { calSet(CAL_FAIL, "vehicle moving"); return; }
    calEncCenter = keyaEncoderRaw;
    calRefCenter = refWheelAngle;
    calDzSum = 0; calDzDone = 0;
    calResDz = calResTL = calResTR = calResTpd = 0;
    calManCap = 0;
    calDir = +1;
    calSet(CAL_REACT, "reaction check: sweeping +");
}

void calAbort() { calStop(); calSet(CAL_IDLE, "aborted"); }

void calApply() {
    if (calState != CAL_DONE) return;
    if (calResTpd > 1.0f)  moduleConfig.keyaTicksPerDeg = calResTpd;
    if (calResTR  > 1.0f)  moduleConfig.keyaTicksRight   = calResTR;
    if (calResTL  > 1.0f)  moduleConfig.keyaTicksLeft    = calResTL;
    moduleConfig.keyaDeadZone = calResDz;
    moduleConfigSave();
    calSet(CAL_IDLE, "applied & saved");
}

// Common abort guards
static bool calGuard() {
    if (refAngleTime > 1500 || !refAngleValid) { calStop(); calSet(CAL_FAIL, "reference lost"); return false; }
    if (gpsSpeed > 0.5f)                        { calStop(); calSet(CAL_FAIL, "vehicle moving"); return false; }
    if (digitalRead(STEERSW_PIN) == LOW)        { calStop(); calSet(CAL_FAIL, "steer switch pressed"); return false; }
    return true;
}

void calibrationLoop()
{
    if (calState == CAL_IDLE || calState == CAL_DONE || calState == CAL_FAIL) { calStop(); return; }
    if (!calGuard()) return;

    float refDelta = refWheelAngle - calRefCenter;
    float tpd = (moduleConfig.keyaTicksPerDeg > 1.0f) ? moduleConfig.keyaTicksPerDeg : 24.0f;

    switch (calState) {

    // ── Reaction: sweep +, confirm ref moves; then sweep -, confirm ───────────
    case CAL_REACT: {
        if (calPhase == 0) {                       // sweep +
            calMotor(+calSpeed);
            if (refDelta >= CAL_REACT_THRESH) { calStop(); calPhase = 1; calStateTimer = 0; strncpy(calMsg,"reaction check: sweeping -",sizeof(calMsg)); }
            else if (calStateTimer > CAL_REACT_TIMEOUT) { calStop(); calSet(CAL_FAIL, "no reaction (+) - check IMU mount"); }
        } else {                                   // sweep - back toward/below center
            calMotor(-calSpeed);
            if (refDelta <= 0.0f) {                // returned through center
                calStop();
                calRefCenter = refWheelAngle; calEncCenter = keyaEncoderRaw;  // re-zero
                calDir = +1; calRevEnc = keyaEncoderRaw; calCycleStartRef = refWheelAngle;
                calSet(CAL_DEADZONE, "dead zone: cycle 1");
            } else if (calStateTimer > CAL_REACT_TIMEOUT) { calStop(); calSet(CAL_FAIL, "no reaction (-)"); }
        }
        break;
    }

    // ── Dead zone: drive to +AMPL, reverse, measure backlash; repeat ──────────
    case CAL_DEADZONE: {
        calMotor(calDir * calSpeed);
        float refFromCycle = (refWheelAngle - calCycleStartRef) * calDir;   // progress this leg
        if (calPhase == 0) {
            // driving until ref advanced ~AMPL, then reverse and start measuring
            if (refFromCycle >= CAL_DZ_AMPL) {
                calDir = -calDir;
                calRevEnc = keyaEncoderRaw;            // encoder at reversal
                calCycleStartRef = refWheelAngle;
                calPhase = 1;
            } else if (calStateTimer > CAL_RANGE_TIMEOUT) { calStop(); calSet(CAL_FAIL, "dead zone timeout"); }
        } else {
            // after reversal: ref flat during backlash; when ref moves, encoder travel = backlash
            float refMove = (refWheelAngle - calCycleStartRef) * calDir;    // positive once moving new dir
            if (refMove >= CAL_DZ_MOVE) {
                float backlashDeg = fabs((float)(keyaEncoderRaw - calRevEnc)) / tpd;
                calDzSum += backlashDeg;
                calDzDone++;
                if (calDzDone >= CAL_DZ_CYCLES) {
                    calStop();
                    calResDz = calDzSum / calDzDone;
                    // dead zone done → hand over to MANUAL range capture (operator
                    // turns the wheel by hand and clicks Capture centre/left/right).
                    calManCap = 0;
                    calSet(CAL_MANUAL_RANGE, "dead zone done. Turn to CENTRE, click Capture centre");
                } else {
                    calCycleStartRef = refWheelAngle;
                    calPhase = 0;       // next cycle drive in current (new) dir to +AMPL
                    char m[48]; snprintf(m, sizeof(m), "dead zone: cycle %d", calDzDone + 1);
                    strncpy(calMsg, m, sizeof(calMsg));
                }
            } else if (calStateTimer > CAL_RANGE_TIMEOUT) { calStop(); calSet(CAL_FAIL, "dead zone reversal timeout"); }
        }
        break;
    }

    // ── Manual range: motor OFF, operator turns by hand and clicks captures ────
    // Captures happen in calCapCentre/Left/Right (web handler). Here we just keep
    // the motor stopped and let the safety guards (ref link) keep running.
    case CAL_MANUAL_RANGE: {
        calStop();
        break;
    }
    }
}

// ── Manual range capture (called from the web handler) ───────────────────────
// The operator turns the steered wheel by hand to centre / full-left / full-right
// and clicks the matching button. Captures the encoder + reference IMU angle at
// each point; absolute centre is defined by the operator (more precise than the
// motor's stall detection).
void calCapCentre() {
    if (calState != CAL_MANUAL_RANGE) return;
    calEncCenter = keyaEncoderRaw; calRefCenter = refWheelAngle;
    calManCap |= 0x01;
    strncpy(calMsg, "centre captured. Turn full LEFT, click Capture left", sizeof(calMsg) - 1);
}

void calCapLeft() {
    if (calState != CAL_MANUAL_RANGE) return;
    calEncLeft = keyaEncoderRaw; calRefLeft = refWheelAngle;
    calManCap |= 0x02;
    strncpy(calMsg, "left captured. Back to centre, then full RIGHT + Capture right", sizeof(calMsg) - 1);
}

void calCapRight() {
    if (calState != CAL_MANUAL_RANGE) return;
    calEncRight = keyaEncoderRaw; calRefRight = refWheelAngle;
    calManCap |= 0x04;
    strncpy(calMsg, "right captured. Click Compute when all three are done", sizeof(calMsg) - 1);
}

void calComputeRange() {
    if (calState != CAL_MANUAL_RANGE) return;
    if ((calManCap & 0x07) != 0x07) {
        strncpy(calMsg, "capture centre + left + right first", sizeof(calMsg) - 1);
        return;
    }
    float degR  = fabs(calRefRight - calRefCenter);
    float degL  = fabs(calRefLeft  - calRefCenter);
    float tickR = fabs((float)(calEncRight - calEncCenter));
    float tickL = fabs((float)(calEncLeft  - calEncCenter));
    calResTR = (degR > 1.0f) ? tickR / degR : 0;
    calResTL = (degL > 1.0f) ? tickL / degL : 0;
    calResTpd = ((calResTR > 0) + (calResTL > 0)) ?
                ((calResTR + calResTL) / ((calResTR > 0) + (calResTL > 0))) : 0;
    char m[48]; snprintf(m, sizeof(m), "done: L %.1f R %.1f t/deg", calResTL, calResTR);
    calSet(CAL_DONE, m);
}
