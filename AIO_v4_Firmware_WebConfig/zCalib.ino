// ─────────────────────────────────────────────────────────────────────────────
// Keya WAS motorized auto-calibration
//
// Calibrates Keya counts → bicycle (virtual-centre) angle. Wheel angle (from the
// reference IMU or a protractor) is converted to the bike angle via L,T (wheelToBike).
//   1. DEAD ZONE  (calStartDeadzone) — automatic: motor turns + reverses to measure
//      backlash on direction change.
//   2. RANGE A    (calStartSweep) — operator turns lock-to-lock by hand (motor OFF),
//      ~10 Hz samples feed a per-side least-squares fit → ticks/deg (bike) + RMS.
//   3. RANGE B    (calStartManual) — no IMU: per lock the operator enters the measured
//      inner-wheel angle (protractor); ticks/deg = tickΔ / angle (+ bike conversion).
// Apply writes the BIKE ticks/deg into keyaTicksLeft/Right (+ base).
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
int32_t calEncCenter;                 // non-static: web status shows tick delta from centre
static float  calRefCenter;
// dead-zone accumulation
static float  calDzSum;
static int    calDzDone;
static int32_t calRevEnc;
static float  calCycleStartRef;
static int8_t calDir;

// ── Wheel-angle → bicycle (virtual centre) angle, MAGNITUDE in degrees ─────────
// L,T from config. inner = the measured wheel is the inner wheel for this turn.
//   tan(δ_bike) = L/R ; inner: R = L/tan(δ_w) + T/2 ; outer: R = L/tan(δ_w) − T/2
static float wheelToBike(float dwMagDeg, bool inner) {
    float L = moduleConfig.wheelBase;     // shared tractor wheelbase
    float T = moduleConfig.keyaTrackT;
    if (L < 0.1f) return dwMagDeg;                       // no geometry → wheel == bike
    float t = tanf(dwMagDeg * (float)DEG_TO_RAD);
    float denom = L + (inner ? +1.0f : -1.0f) * (T * 0.5f) * t;
    if (denom < 0.01f) return dwMagDeg;
    return atanf(L * t / denom) * (float)RAD_TO_DEG;
}

// CalFit struct lives in zConfig.h (early header) so the auto-prototypes see it.
// Fits angle(y) vs tick(x): slope = deg/tick → ticks/deg = 1/slope ; RMS in degrees.
static CalFit calFitL, calFitR;

static void calFitReset(CalFit &f) { memset(&f, 0, sizeof(f)); }
// Accumulate one (tick, wheel-angle) sample; both bike conversions are pre-computed
// so the inner/outer choice can be deferred to the end (robust to encoder polarity).
static void calFitAdd(CalFit &f, double tick, double wheelDeg) {
    double bi = wheelToBike((float)wheelDeg, true);     // bike if this side is inner
    double bo = wheelToBike((float)wheelDeg, false);    // bike if this side is outer
    f.n++; f.Sx += tick; f.Sxx += tick*tick;
    f.Sw += wheelDeg; f.Swx += tick*wheelDeg; f.Sww += wheelDeg*wheelDeg;
    f.Si += bi; f.Six += tick*bi; f.Sii += bi*bi;
    f.So += bo; f.Sox += tick*bo; f.Soo += bo*bo;
    if (wheelDeg > f.maxW) f.maxW = wheelDeg;
}
// Generic least-squares: y(angle) vs x(tick) → ticks/deg = 1/slope, RMS in degrees.
// col 0 = wheel, 1 = bike-inner, 2 = bike-outer.
static float calFitSlope(const CalFit &f, uint8_t col, float &rmsOut) {
    rmsOut = 0;
    if (f.n < 5) return 0;
    double n = f.n, Sx = f.Sx, Sxx = f.Sxx;
    double Sy, Sxy, Syy;
    if      (col == 0) { Sy = f.Sw; Sxy = f.Swx; Syy = f.Sww; }
    else if (col == 1) { Sy = f.Si; Sxy = f.Six; Syy = f.Sii; }
    else               { Sy = f.So; Sxy = f.Sox; Syy = f.Soo; }
    double denom = n*Sxx - Sx*Sx;
    if (fabs(denom) < 1e-6) return 0;
    double slope = (n*Sxy - Sx*Sy) / denom;             // deg per tick
    if (fabs(slope) < 1e-9) return 0;
    double Sxy_c = Sxy - Sx*Sy/n;
    double Syy_c = Syy - Sy*Sy/n;
    double ssres = Syy_c - slope*Sxy_c;
    if (ssres < 0) ssres = 0;
    rmsOut = (float)sqrt(ssres / n);
    return (float)(1.0 / fabs(slope));                  // ticks per degree
}

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

static bool calCommonGuards() {
    if (!keyaDetected)                            { calSet(CAL_FAIL, "Keya not detected"); return false; }
    if (!(refAngleTime < 1000 && refAngleValid))  { calSet(CAL_FAIL, "no reference IMU link"); return false; }
    if (gpsSpeed > 0.5f)                          { calSet(CAL_FAIL, "vehicle moving"); return false; }
    return true;
}

// Public: start dead-zone (auto, motor turns) — stops at DONE, range untouched
void calStartDeadzone() {
    if (!calCommonGuards()) return;
    calResDz = 0; calHaveDz = false;               // this one is being measured
    calEncCenter = keyaEncoderRaw;
    calRefCenter = refWheelAngle;
    calDzSum = 0; calDzDone = 0;
    calDir = +1;
    calSet(CAL_REACT, "reaction check: sweeping +");
}

static void calResetRange() {
    calResTL = calResTR = calResTpd = 0;
    calResWheelL = calResWheelR = 0;
    calRmsL = calRmsR = 0;
    calResMaxL = calResMaxR = 0;
    calHaveRange = false;
    calManCap = 0;
}

// Public: Tool A — IMU sweep (operator turns lock-to-lock, motor OFF). Needs the
// reference IMU. Accumulates a per-side least-squares fit → ticks/deg (bike) + RMS.
void calStartSweep() {
    if (!calCommonGuards()) return;                // needs Keya + reference IMU + stationary
    calResetRange();
    calFitReset(calFitL); calFitReset(calFitR);
    calStop();
    calEncCenter = keyaEncoderRaw;
    calRefCenter = refWheelAngle;
    calSet(CAL_SWEEP, "sweep: turn slowly full RIGHT then full LEFT (a few times), then Stop");
}

// Public: Tool B — manual protractor (no IMU). Set centre, then per lock the
// operator enters the measured INNER-wheel angle and captures the tick delta.
void calStartManual() {
    if (!keyaDetected)   { calSet(CAL_FAIL, "Keya not detected"); return; }
    if (gpsSpeed > 0.5f) { calSet(CAL_FAIL, "vehicle moving");    return; }
    calResetRange();
    calStop();
    calEncCenter = keyaEncoderRaw;
    calSet(CAL_MANUAL_RANGE, "manual: wheels straight = Set Centre, then turn to a lock + enter angle");
}

void calAbort() { calStop(); calSet(CAL_IDLE, "aborted"); }

// Apply only what was actually measured this session — the other value is left
// untouched in EEPROM (so running one calibration never zeroes the other).
void calApply() {
    if (calState != CAL_DONE && calState != CAL_MANUAL_RANGE) return;
    if (calHaveDz)    moduleConfig.keyaDeadZone = calResDz;
    if (calHaveRange) {
        if (calResTpd > 1.0f) moduleConfig.keyaTicksPerDeg = calResTpd;
        if (calResTR  > 1.0f) moduleConfig.keyaTicksRight   = calResTR;
        if (calResTL  > 1.0f) moduleConfig.keyaTicksLeft    = calResTL;
    }
    moduleConfigSave();
    calSet(CAL_IDLE, "applied & saved");
}

// Common abort guards. Manual (Tool B) is protractor-based — no reference IMU needed.
static bool calGuard() {
    bool needRef = (calState != CAL_MANUAL_RANGE);
    if (needRef && (refAngleTime > 1500 || !refAngleValid)) { calStop(); calSet(CAL_FAIL, "reference lost"); return false; }
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
                    calHaveDz = true;
                    char m[48]; snprintf(m, sizeof(m), "dead zone = %.2f deg (done)", calResDz);
                    calSet(CAL_DONE, m);
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

    // ── Manual (Tool B): motor OFF, operator captures locks via the web handler ─
    // NB: do NOT call calStop() every iteration — during calibration autosteerLoop
    // returns early and runs thousands of Hz; SteerKeya writes 2 CAN frames each
    // call and would flood the bus, blocking the Keya heartbeat (encoder freezes).
    case CAL_MANUAL_RANGE: {
        static elapsedMillis manStop = 0;
        if (manStop > 500) { manStop = 0; calStop(); }   // keep motor off, rate-limited
        break;
    }

    // ── Sweep (Tool A): motor OFF, operator turns lock-to-lock by hand. Sample at
    // ~10 Hz: sort by steer-sign side, convert wheel→bike, feed the per-side fit. ─
    case CAL_SWEEP: {
        static elapsedMillis swStop = 0;
        if (swStop > 500) { swStop = 0; calStop(); }     // keep motor off, rate-limited
        static elapsedMillis sweepSamp = 0;
        if (sweepSamp < 100) break;                     // ~10 Hz
        sweepSamp = 0;
        int32_t dtick   = keyaEncoderRaw - calEncCenter;
        int32_t sgnTick = moduleConfig.keyaEncInvert ? dtick : -dtick;  // steer-sign (matches deltaSigned)
        float   dwMag   = fabs(refWheelAngle - calRefCenter);
        float   tickMag = fabs((float)dtick);
        if (tickMag < 20.0f || dwMag < 0.5f) break;     // skip the dead band around centre
        // Just sort by encoder sign into two buckets; which bucket is inner/outer
        // (i.e. right/left turn) is decided in calStopSweep from the max wheel angle.
        if (sgnTick > 0) calFitAdd(calFitR, tickMag, dwMag);
        else             calFitAdd(calFitL, tickMag, dwMag);
        break;
    }
    }
}

// ── Helper: recompute base ticks/deg (avg of valid bike slopes) + ready flag ──
static void calUpdateBase() {
    int nv = (calResTR > 1.0f) + (calResTL > 1.0f);
    calResTpd = nv ? ((calResTR * (calResTR > 1.0f) + calResTL * (calResTL > 1.0f)) / nv) : 0;
    calHaveRange = (nv > 0);
}

// ── Tool A: finish the sweep — compute per-side least-squares slopes ──────────
void calStopSweep() {
    if (calState != CAL_SWEEP) return;
    calStop();
    // Inner side detection (robust to encoder polarity AND sweep extent): the inner
    // wheel turns MORE per tick → SMALLER wheel ticks/deg. Compare the two buckets'
    // wheel slopes; smaller = inner = right turn. Fall back to max angle if a slope
    // is unavailable (one side not swept enough).
    float dwR, dwL;
    float wR = calFitSlope(calFitR, 0, dwR);
    float wL = calFitSlope(calFitL, 0, dwL);
    bool rBucketInner;
    if (wR > 0.1f && wL > 0.1f) rBucketInner = (wR <= wL);          // smaller wheel t/d = inner
    else                        rBucketInner = (calFitR.maxW >= calFitL.maxW);
    CalFit &inner = rBucketInner ? calFitR : calFitL;
    CalFit &outer = rBucketInner ? calFitL : calFitR;
    float dummy;
    calResTR     = calFitSlope(inner, 1, calRmsR);   // bike, inner column → right
    calResTL     = calFitSlope(outer, 2, calRmsL);   // bike, outer column → left
    calResWheelR = calFitSlope(inner, 0, dummy);     // raw wheel t/d (compare)
    calResWheelL = calFitSlope(outer, 0, dummy);
    calResMaxR   = (float)inner.maxW;
    calResMaxL   = (float)outer.maxW;
    calUpdateBase();
    char m[80];
    if (!calHaveRange)
        snprintf(m, sizeof(m), "not enough sweep data - turn fuller/slower, both sides");
    else
        snprintf(m, sizeof(m), "sweep done: bike t/d L %.1f R %.1f (RMS L %.2f R %.2f)",
                 calResTL, calResTR, calRmsL, calRmsR);
    calSet(CAL_DONE, m);
}

// ── Tool B: manual capture (called from the web handler) ─────────────────────
void calManSetCentre() {
    if (calState != CAL_MANUAL_RANGE) return;
    calEncCenter = keyaEncoderRaw;
    calManCap |= 0x01;
    strncpy(calMsg, "centre set. Turn full RIGHT, measure inner-wheel angle, Capture right", sizeof(calMsg) - 1);
}

// side: +1 = right lock, -1 = left lock. angleDeg = measured INNER wheel angle (protractor).
void calManCapLock(int8_t side, float angleDeg) {
    if (calState != CAL_MANUAL_RANGE) return;
    if (angleDeg < 1.0f) { strncpy(calMsg, "enter the measured wheel angle first", sizeof(calMsg) - 1); return; }
    float tickMag = fabs((float)(keyaEncoderRaw - calEncCenter));
    float bike    = wheelToBike(angleDeg, true);   // inner wheel measured at each lock
    float tdWheel = (angleDeg > 0.5f) ? tickMag / angleDeg : 0;
    float tdBike  = (bike     > 0.5f) ? tickMag / bike      : 0;
    if (side > 0) { calResWheelR = tdWheel; calResTR = tdBike; calResMaxR = angleDeg; calManCap |= 0x04; }
    else          { calResWheelL = tdWheel; calResTL = tdBike; calResMaxL = angleDeg; calManCap |= 0x02; }
    calUpdateBase();
    char m[72];
    snprintf(m, sizeof(m), "%s lock: wheel %.1f t/d, bike %.1f t/d", side > 0 ? "right" : "left", tdWheel, tdBike);
    strncpy(calMsg, m, sizeof(calMsg) - 1);
}
