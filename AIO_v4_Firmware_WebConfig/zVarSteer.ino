// ─────────────────────────────────────────────────────────────────────────────
// Variable Steering (custom — Deutz-Fahr orchard tractor)
//
// Two tractor-specific problems the stock firmware does not cover:
//
//   1. The OEM analog WAS is unusable for control. With the engine running it
//      shows impulsive spikes (baseline steady, then a jump of up to ~10 deg and
//      straight back). Filtering it hard enough to control on adds far too much
//      lag. But the BASELINE is stable — so it is perfectly good as a slow
//      ABSOLUTE reference, which is exactly what the Keya encoder lacks.
//
//   2. A twin orbital (125/250 ccm) gives two steering-wheel:wheel-angle ratios,
//      exactly 2:1 from the displacements. The encoder scale therefore is not a
//      constant. There is no electrical signal available for which one is
//      active, so it has to be measured.
//
// Design: the encoder stays PRIMARY and carries 100% of the dynamics. The WAS
// never enters the fast path — it only moves a slow offset (time constant of
// seconds), well below the control bandwidth, so the PID cannot see it. Spikes
// are removed by a median filter plus an innovation gate against the encoder
// prediction (the encoder is short-term trustworthy, which makes it an excellent
// outlier detector).
//
// The orbital ratio is measured, never remembered: the tractor may be powered up
// in either mode, and the Teensy may be powered independently of it, so a stored
// mode would be confidently wrong. Instead the zero is taken at a small angle
// (where the ratio barely matters) and the first real turn resolves the mode.
// Because the ratio is exactly 2:1 and the zero inputs are kept, correcting it is
// an exact recompute — not a re-acquisition.
//
// Hard safety rule: ticks/deg is NEVER changed while autosteer is engaged.
// Changing the gain under active control would step the feedback signal and jerk
// the wheel.
// ─────────────────────────────────────────────────────────────────────────────

// Forward declarations for these live in zConfig.h (included first, so Autosteer.ino
// sees them too) rather than relying on the Arduino auto-prototype generator — its
// insertion point has already caused one build break in this sketch.

// ── Median-5 on the raw ADS counts ───────────────────────────────────────────
// Impulsive noise is the wrong problem for an EMA: a single 10 deg spike through
// an EMA leaves a tail that decays over seconds. A median rejects an isolated
// spike completely (up to 2 of 5 samples) without shifting the baseline and with
// essentially no lag on the mean. readAdsRaw() runs at 20 Hz when Keya is the
// active source, so 5 samples = 250 ms.
#define VS_MED_N 5
static int16_t vsMedBuf[VS_MED_N];
static uint8_t vsMedIdx = 0;
static uint8_t vsMedCnt = 0;

void vsMedianPush(int16_t raw)
{
    // Disabled → pass through, and reset the window so re-enabling starts clean
    // (otherwise the warm-up would read stale slots).
    if (!moduleConfig.vs.medianEnable) { adsMedCounts = raw; vsMedCnt = 0; vsMedIdx = 0; return; }

    vsMedBuf[vsMedIdx] = raw;
    vsMedIdx = (uint8_t)((vsMedIdx + 1) % VS_MED_N);
    if (vsMedCnt < VS_MED_N) vsMedCnt++;

    // Insertion sort of a copy — N=5, cheaper than anything clever.
    int16_t s[VS_MED_N];
    for (uint8_t i = 0; i < vsMedCnt; i++) s[i] = vsMedBuf[i];
    for (uint8_t i = 1; i < vsMedCnt; i++) {
        int16_t v = s[i];
        int8_t  j = (int8_t)i - 1;
        while (j >= 0 && s[j] > v) { s[j + 1] = s[j]; j--; }
        s[j + 1] = v;
    }
    adsMedCounts = s[vsMedCnt / 2];
}

// ── WAS counts → bike (virtual-centre) angle ─────────────────────────────────
// Calibrated against the same reference IMU sweep that produces ticks/deg, so it
// lands in the SAME angle space as steerAngleActual (bike angle) and the two are
// directly comparable. Returns false when the WAS has never been calibrated.
bool vsWasAngle(float &angleOut)
{
    if (!adcConnected) return false;
    if (fabs(moduleConfig.vs.wasDegPerCount) < 1e-9f) return false;   // uncalibrated
    angleOut = moduleConfig.vs.wasDegPerCount * (float)adsMedCounts
             + moduleConfig.vs.wasIntercept;
    return true;
}

// ── Runtime state for the v2 zero work ───────────────────────────────────────
// Deliberately declared here rather than with the other VS globals in the main
// sketch tab: everything that touches them lives in this file and in zWebServer,
// which the tab concatenation puts after it.
char  vsTrimMsg[56]   = "off";   // auto-trim state, shown live in the GUI
elapsedMillis vsStillTimer = 0;  // how long the encoder has been still
float vsStillSpread   = 999.0f;  // WAS spread over that window (deg, needs a slope)
// Counts-domain versions of the same window. These stay valid with NO calibration
// at all, which is what lets the manual calibration (Tool B) use them to produce
// the very slope the degree version depends on.
float vsStillMeanCounts   = 0.0f;
float vsStillSpreadCounts = 0.0f;

// ── WAS angle monitor ────────────────────────────────────────────────────────
// Runs every cycle from autosteerLoop, independent of the WAS source, of the
// initial zero and of the fusion switch. The Keya branch breaks out early until
// the initial zero is done, so keeping this here is what makes the tractor WAS
// visible from power-up — which is exactly when you want to compare it against
// the encoder before enabling anything.
void vsWasMonitor()
{
    float a;
    vsWasUsable = vsWasAngle(a) && fabs(a) < 90.0f;
    if (vsWasUsable) vsLastWasAngle = a;

    // ── Rolling "encoder still" window ───────────────────────────────────────
    // The manual zero button has to answer instantly — a web handler cannot sit
    // and watch the wheel for two seconds. So the window is kept running here and
    // the button just reads the verdict. Same thresholds as the automatic zero.
    // That one keeps its own window on purpose: it is a one-shot with different
    // arming, and re-using this one would change behaviour already validated.
    //
    // Deliberately gated on adcConnected, NOT on vsWasUsable: the manual
    // calibration reads the mean from here while the sensor is still completely
    // uncalibrated, and vsWasUsable is false exactly then. Everything the window
    // publishes in counts is therefore always meaningful; only the degree figure
    // needs a slope, and callers that use it check for one first.
    static int32_t refTicks = 0;
    static int16_t cMin = 0, cMax = 0;
    static double  cSum = 0;
    static uint32_t cN = 0;
    static bool    armed = false;

    if (!adcConnected) {
        armed = false; vsStillTimer = 0;
        vsStillSpread = 999.0f; vsStillSpreadCounts = 0.0f; vsStillMeanCounts = 0.0f;
    } else if (!armed || labs(keyaEncoderRaw - refTicks) > (int32_t)moduleConfig.vs.zeroStillTicks) {
        armed    = true;
        refTicks = keyaEncoderRaw;
        vsStillTimer = 0;
        cMin = cMax = adsMedCounts;
        cSum = (double)adsMedCounts;
        cN   = 1;
        vsStillMeanCounts   = (float)adsMedCounts;
        vsStillSpreadCounts = 0.0f;
        vsStillSpread       = 0.0f;
    } else {
        if (adsMedCounts < cMin) cMin = adsMedCounts;
        if (adsMedCounts > cMax) cMax = adsMedCounts;
        cSum += (double)adsMedCounts;
        cN++;
        vsStillMeanCounts   = (float)(cSum / (double)cN);
        vsStillSpreadCounts = (float)(cMax - cMin);
        vsStillSpread       = vsStillSpreadCounts * fabs(moduleConfig.vs.wasDegPerCount);
    }

    // The slow intercept trim rides along here rather than adding a second call
    // into autosteerLoop — it needs exactly the same "every cycle" cadence, and
    // it self-throttles to 1 Hz internally.
    vsTrimUpdate();
}

// ── Manual "set WAS zero now" ────────────────────────────────────────────────
// The sweep is the only thing that has ever written the intercept, so a centring
// error at "Start sweep" is baked in until the next sweep — which costs another
// session with the reference IMU on the wheel. This is the one-click way out:
// straighten the wheels by whatever you trust, press, done. Only b moves; the
// slope a from the sweep is untouched, so the scale stays calibrated.
//
// Every refusal says WHICH condition failed. On a tractor "failed" is useless.
bool vsWasZeroNow(char* out, uint16_t n)
{
    // Changing the zero steps the feedback signal — never under active control.
    if (watchdogTimer < WATCHDOG_THRESHOLD) {
        snprintf(out, n, "refused: autosteer engaged - disengage first"); return false;
    }
    if (!adcConnected) {
        snprintf(out, n, "refused: no ADS1115 - nothing to read"); return false;
    }
    if (fabs(moduleConfig.vs.wasDegPerCount) < 1e-9f) {
        snprintf(out, n, "refused: WAS slope not calibrated - run the sweep first"); return false;
    }
    if (!vsWasUsable) {
        snprintf(out, n, "refused: no usable WAS reading"); return false;
    }
    if (vsStillTimer < moduleConfig.vs.zeroStillMs) {
        snprintf(out, n, "refused: encoder moving - hold the wheel still"); return false;
    }
    // With the encoder still the true angle is constant, so a wide spread is noise
    // that got through the median — zeroing on it would just bake the noise in.
    if (vsStillSpread > moduleConfig.vs.zeroSpreadDeg) {
        snprintf(out, n, "refused: sample spread %.2f deg over limit - WAS too noisy",
                 vsStillSpread);
        return false;
    }

    float old = moduleConfig.vs.wasIntercept;
    moduleConfig.vs.wasIntercept = -moduleConfig.vs.wasDegPerCount * (float)adsMedCounts;
    // This IS the new definition of zero, so the trim clamp has to measure from
    // here. Leaving the old sweep value as the origin could park a fresh manual
    // zero right on the clamp, with the trim then refusing to move at all.
    moduleConfig.vs.wasInterceptBase = moduleConfig.vs.wasIntercept;
    // The fusion offset was partly compensating for exactly the error just removed;
    // keeping it would double-count. Safe to snap — autosteer is off, checked above.
    vsWasOffset = 0.0f;
    moduleConfigSave();

    snprintf(out, n, "zeroed: intercept %+.3f -> %+.3f deg (shift %+.3f)",
             old, moduleConfig.vs.wasIntercept, moduleConfig.vs.wasIntercept - old);
    webLogf("VS: WAS zero set manually (%+.3f -> %+.3f deg)", old, moduleConfig.vs.wasIntercept);
    return true;
}

// ── Slow auto-trim of the WAS intercept ──────────────────────────────────────
// A second, INDEPENDENT loop. It never looks at the encoder: its only input is
// "I am driving straight, therefore the true angle is 0". That is what stops it
// circling with the fusion above — the fusion pulls the encoder towards the WAS,
// this pulls the WAS towards the road, and the two never feed each other.
//
// Deliberately glacial. The fusion corrects a drifting offset in seconds; this
// corrects a calibration constant, and a wrong constant is not an emergency. At
// the default beta a 2° error takes something like an hour to walk out.
//
// Two witnesses for "straight" — the WAS angle AND the steer actual, each below
// the same limit. One alone would let a long gentle curve poison the calibration,
// which is precisely the failure that would be impossible to spot afterwards.
#define VS_TRIM_STEP_MS   1000
#define VS_TRIM_SAVE_MS   300000UL    // 5 min between EEPROM writes
#define VS_TRIM_SAVE_MIN  0.02f       // ...and only if it actually moved this far

void vsTrimUpdate()
{
    static elapsedMillis step      = 0;
    static elapsedMillis straight  = 0;
    static elapsedMillis sinceSave = 0;
    static float lastSaved = 0.0f;
    static bool  haveSaved = false;

    if (!haveSaved) { lastSaved = moduleConfig.vs.wasIntercept; haveSaved = true; }

    if (!moduleConfig.vs.masterEnable || !moduleConfig.vs.wasPresent
        || !moduleConfig.vs.wasTrimEnable) {
        strncpy(vsTrimMsg, "off", sizeof(vsTrimMsg) - 1);
        straight = 0;
        return;
    }

    // Checked every cycle, not once a second: a yaw spike between two ticks still
    // has to break the window, otherwise "straight for 2 s" means very little.
    const char* why = NULL;
    if      (!vsWasUsable)                                          why = "no calibrated WAS";
    else if (moduleConfig.wasSource == WAS_SOURCE_KEYA
             && !keyaInitialZeroDone)                               why = "waiting: initial zero";
    else if (gpsSpeed < moduleConfig.vs.wasTrimSpeedMin)            why = "waiting: too slow";
    else if (fabs(headingRate) > moduleConfig.vs.wasTrimYawMax)     why = "waiting: turning";
    else if (fabs(vsLastWasAngle) > moduleConfig.vs.wasTrimAngleMax) why = "waiting: WAS angle too large";
    else if (fabs(steerAngleActual) > moduleConfig.vs.wasTrimAngleMax) why = "waiting: steer angle too large";

    if (why) { straight = 0; strncpy(vsTrimMsg, why, sizeof(vsTrimMsg) - 1); }

    if (step < VS_TRIM_STEP_MS) return;
    step = 0;
    if (why) return;
    if (straight < moduleConfig.vs.wasTrimStraightMs) {
        strncpy(vsTrimMsg, "settling...", sizeof(vsTrimMsg) - 1);
        return;
    }

    // We know the wheels are straight, so whatever the WAS reads is its own error.
    moduleConfig.vs.wasIntercept -= vsLastWasAngle * moduleConfig.vs.wasTrimBeta;

    // Clamp against where the zero was last DEFINED, never against the previous
    // correction — otherwise the trim could walk anywhere one small step at a time.
    float base = moduleConfig.vs.wasInterceptBase;
    float lim  = moduleConfig.vs.wasTrimMaxDeg;
    bool  clamped = false;
    if (moduleConfig.vs.wasIntercept > base + lim) { moduleConfig.vs.wasIntercept = base + lim; clamped = true; }
    if (moduleConfig.vs.wasIntercept < base - lim) { moduleConfig.vs.wasIntercept = base - lim; clamped = true; }

    float drift = moduleConfig.vs.wasIntercept - base;
    // Hitting the clamp is not a working trim: the sweep is wrong by more than a
    // zero error and needs redoing. Say so instead of sitting there looking settled.
    if (clamped) snprintf(vsTrimMsg, sizeof(vsTrimMsg), "at clamp %+.2f deg - recalibrate", drift);
    else         snprintf(vsTrimMsg, sizeof(vsTrimMsg), "trimming (drift %+.3f deg)", drift);

    // EEPROM cells are finite and this value moves in thousandths — writing every
    // second would burn the cell to record noise. Time AND movement must both pass.
    if (sinceSave > VS_TRIM_SAVE_MS) {
        sinceSave = 0;
        if (fabs(moduleConfig.vs.wasIntercept - lastSaved) > VS_TRIM_SAVE_MIN) {
            lastSaved = moduleConfig.vs.wasIntercept;
            moduleConfigSave();
            webLogf("VS: WAS intercept auto-trim saved %+.3f deg (drift %+.3f)", lastSaved, drift);
        }
    }
}

// ── WAS as a slow absolute anchor ────────────────────────────────────────────
// Runs at a fixed 20 Hz (one step per new ADS sample) so the tuning is not tied
// to the loop rate. 'predBase' is the encoder angle plus the GPS offset, i.e.
// everything except this correction.
//
// Two independent defences against the impulsive noise:
//   1. The median already removed isolated spikes.
//   2. The innovation gate: the encoder is short-term trustworthy, so a WAS
//      sample far from its prediction is physically impossible and is dropped.
// What survives moves the offset by a small fraction, additionally rate-limited,
// so even a sustained bad reading can only creep.
//
// Requires the initial zero to be done — before that the encoder has no absolute
// meaning, so there is nothing to innovate against (that is phase 3's job).
#define VS_FUSE_STEP_MS 50

void vsFuseUpdate(float predBase)
{
    static elapsedMillis vsStep = 0;
    static elapsedMillis vsRejWindow = 0;
    static uint16_t      vsRejCount = 0;

    // vsWasMonitor() already refreshed these this cycle.
    float wasAngle = vsLastWasAngle;

    if (vsStep < VS_FUSE_STEP_MS) return;
    vsStep = 0;

    // Switched off (or no usable WAS) → clear the offset. The rate limit only exists
    // so the feedback signal does not step under active control; with autosteer off
    // nobody is reading it in a loop, so it can simply snap to zero. That is also the
    // realistic case — you disengage before flipping the switch.
    if (!moduleConfig.vs.masterEnable || !moduleConfig.vs.wasPresent
        || !moduleConfig.vs.fuseEnable || !vsWasUsable || !keyaInitialZeroDone) {
        vsGateBlocked = 0;
        if (watchdogTimer >= WATCHDOG_THRESHOLD) {
            vsWasOffset = 0.0f;                        // not engaged → immediate
        } else {
            float maxStep = moduleConfig.vs.rateMaxDegS * (VS_FUSE_STEP_MS / 1000.0f);
            if      (vsWasOffset >  maxStep) vsWasOffset -= maxStep;
            else if (vsWasOffset < -maxStep) vsWasOffset += maxStep;
            else                             vsWasOffset  = 0.0f;
        }
        return;
    }

    vsLastInnov = wasAngle - (predBase + vsWasOffset);

    // Rejection rate — how hard the noise is hitting right now (diagnostic).
    if (vsRejWindow > 1000) { vsRejectPerSec = vsRejCount; vsRejCount = 0; vsRejWindow = 0; }

    if (fabs(vsLastInnov) > moduleConfig.vs.gateDeg) {
        vsRejCount++;
        return;                       // impossible jump — drop it, keep vsGateBlocked running
    }
    vsGateBlocked = 0;                // a sample got through: the anchor is alive

    // Gentler while engaged, exactly like the GPS auto-zero, so it cannot fight the PID.
    float k = (watchdogTimer < WATCHDOG_THRESHOLD) ? (moduleConfig.vs.fuseBeta / 5.0f)
                                                   : moduleConfig.vs.fuseBeta;
    float step = vsLastInnov * k;

    float maxStep = moduleConfig.vs.rateMaxDegS * (VS_FUSE_STEP_MS / 1000.0f);
    if (step >  maxStep) step =  maxStep;
    if (step < -maxStep) step = -maxStep;

    vsWasOffset += step;
    if (vsWasOffset >  moduleConfig.vs.offsetMaxDeg) vsWasOffset =  moduleConfig.vs.offsetMaxDeg;
    if (vsWasOffset < -moduleConfig.vs.offsetMaxDeg) vsWasOffset = -moduleConfig.vs.offsetMaxDeg;
}

// ── Orbital ratio detection ──────────────────────────────────────────────────
// The WAS sits AFTER the orbital: it measures the real wheel angle and is therefore
// independent of the ratio. So over a window with enough travel,
//
//     ratio = d(WAS) / d(encoder angle at base scale)
//
// is ~1 in 125 ccm and ~2 in 250 ccm. The separation is a full factor of two, so
// the threshold sits at the geometric mean (sqrt(2)) with plenty of margin — the
// impulsive WAS noise is irrelevant here because the measurement is a DIFFERENCE
// over a large movement, where signal utterly dominates noise.
//
// Requires a decent amount of travel per window (vsDetectMinDeg) and several
// agreeing windows before acting, so a single bad reading cannot flip the gain.
void vsOrbitalDetect(float encAngleBase)
{
    // A faulty WAS must never reach this: a garbage estimate could flip the gain 2x
    // or spuriously disengage. With the sensor declared absent the ratio is manual only.
    if (!moduleConfig.vs.wasPresent)    { strncpy(vsOrbitalMsg, "WAS disabled - manual only", sizeof(vsOrbitalMsg) - 1); return; }
    if (!moduleConfig.vs.masterEnable)  { strncpy(vsOrbitalMsg, "master off - manual only", sizeof(vsOrbitalMsg) - 1); return; }
    if (!moduleConfig.vs.orbitalEnable) { strncpy(vsOrbitalMsg, "off", sizeof(vsOrbitalMsg) - 1); return; }

    static elapsedMillis step = 0;
    static bool  have = false;
    static float wasStart = 0, encStart = 0;

    if (!vsWasUsable) {
        have = false; strncpy(vsOrbitalMsg, "no WAS", sizeof(vsOrbitalMsg) - 1); return;
    }
    float wasAngle = vsLastWasAngle;
    if (step < 50) return;
    step = 0;

    if (!have) { have = true; wasStart = wasAngle; encStart = encAngleBase; return; }

    float dWas = wasAngle - wasStart;
    float dEnc = encAngleBase - encStart;
    if (fabs(dWas) < moduleConfig.vs.detectMinDeg) {
        // Not enough travel yet. If the encoder moved a lot while the WAS did not,
        // the window is stale (reversal) — restart it.
        if (fabs(dEnc) > moduleConfig.vs.detectMinDeg * 2.0f) { wasStart = wasAngle; encStart = encAngleBase; }
        strncpy(vsOrbitalMsg, "turn more to measure", sizeof(vsOrbitalMsg) - 1);
        return;
    }
    if (fabs(dEnc) < 1.0f) { have = false; return; }        // encoder barely moved → meaningless
    if ((dWas > 0) != (dEnc > 0)) { have = false; return; } // opposite directions → not a clean sweep

    vsRatioEst = fabs(dWas) / fabs(dEnc);
    have = false;                                            // start a fresh window

    // sqrt(ratio) is the geometric mean between 1x and 2x — the natural split.
    float thresh = sqrtf((moduleConfig.vs.orbitalRatio > 0.1f) ? moduleConfig.vs.orbitalRatio : 2.0f);
    uint8_t seen = (vsRatioEst > thresh) ? 1 : 0;

    if (seen == vsDetectMode) { if (vsDetectCnt < 250) vsDetectCnt++; }
    else                      { vsDetectMode = seen; vsDetectCnt = 1; }

    if (vsDetectCnt < moduleConfig.vs.detectConfirm) {
        snprintf(vsOrbitalMsg, sizeof(vsOrbitalMsg), "est %.2f (%u/%u)",
                 vsRatioEst, vsDetectCnt, moduleConfig.vs.detectConfirm);
        return;
    }
    if (vsDetectMode == moduleConfig.vs.orbitalMode) {
        snprintf(vsOrbitalMsg, sizeof(vsOrbitalMsg), "confirmed %s ccm (est %.2f)",
                 vsDetectMode ? "250" : "125", vsRatioEst);
        return;
    }

    // ── Mismatch confirmed ───────────────────────────────────────────────────
    bool engaged = (watchdogTimer < WATCHDOG_THRESHOLD);
    if (engaged) {
        // HARD RULE: the gain is never changed under active control.
        snprintf(vsOrbitalMsg, sizeof(vsOrbitalMsg), "WRONG RATIO (%s ccm) - engaged, not changing",
                 vsDetectMode ? "250" : "125");
        webLogf("VS: wrong orbital ratio detected while engaged (est %.2f)", vsRatioEst);
        // The dangerous direction is running 125 when it is really 250: the angle is
        // UNDER-reported 2x, so AOG keeps steering into the turn and overshoots. Hand
        // control back rather than fight it. The benign direction (over-reported →
        // sluggish) only warns. Disengaging is not a gain change, so the rule holds.
        if (moduleConfig.vs.disengageOnBad && vsDetectMode == 1 && moduleConfig.vs.orbitalMode == 0) {
            if (steerSwitch == 0) disengageLog("VS: orbital ratio mismatch");
            steerSwitch = 1;
            currentState = 1;
            previous = 0;
        }
        return;
    }

    if (!moduleConfig.vs.orbitalAuto) {
        snprintf(vsOrbitalMsg, sizeof(vsOrbitalMsg), "suggests %s ccm (auto off)",
                 vsDetectMode ? "250" : "125");
        return;
    }
    vsSetOrbitalMode(vsDetectMode);
    snprintf(vsOrbitalMsg, sizeof(vsOrbitalMsg), "auto-switched to %s ccm (est %.2f)",
             vsDetectMode ? "250" : "125", vsRatioEst);
}

// ── Initial zero from the WAS ────────────────────────────────────────────────
// Unlocks autosteer without waiting for the GPS conditions (speed + straight),
// which is the difference between working immediately and idling at the headland.
//
// The gate is "encoder still", NOT "vehicle stopped". If the encoder is not
// moving, the true angle is constant BY DEFINITION, so every variation on the WAS
// is noise and can be measured and rejected. That also makes this work while
// driving straight — powering up mid-drive is fine.
//
// |WAS| must be small. keyaZeroTicks depends on ticks/deg whenever the zero angle
// is non-zero, and at power-up the orbital mode is unknown (it is measured, never
// remembered — the tractor may be started in either mode, and the Teensy may be
// powered independently). At a small angle a 2x ratio error contributes only 1-2
// deg, which the GPS auto-zero then trims away.
//
// This is the one place where a wrong value gives a wrong angle from the very
// first second, before anything else can react — hence the quality checks and the
// default-off switch.
void vsZeroFromWasUpdate()
{
    if (!moduleConfig.vs.masterEnable || !moduleConfig.vs.wasPresent
        || !moduleConfig.vs.zeroEnable || keyaInitialZeroDone) return;
    if (moduleConfig.wasSource != WAS_SOURCE_KEYA) return;
    if (!keyaDetected || !keyaEncInitDone) return;

    static elapsedMillis stillTimer = 0;
    static int32_t  refTicks = 0;
    static float    wMin = 0, wMax = 0;
    static bool     armed = false;

    if (!vsWasUsable) {
        armed = false; strncpy(vsZeroMsg, "no calibrated WAS", sizeof(vsZeroMsg) - 1);
        return;
    }
    float wasAngle = vsLastWasAngle;

    // Encoder still? Any movement beyond the tolerance restarts the window.
    if (!armed || labs(keyaEncoderRaw - refTicks) > (int32_t)moduleConfig.vs.zeroStillTicks) {
        armed = true;
        refTicks = keyaEncoderRaw;
        stillTimer = 0;
        wMin = wMax = wasAngle;
        strncpy(vsZeroMsg, "encoder moving", sizeof(vsZeroMsg) - 1);
        return;
    }

    if (wasAngle < wMin) wMin = wasAngle;
    if (wasAngle > wMax) wMax = wasAngle;

    if (fabs(wasAngle) > moduleConfig.vs.zeroMaxDeg) {
        strncpy(vsZeroMsg, "angle too large - straighten up", sizeof(vsZeroMsg) - 1);
        return;
    }
    if (stillTimer < moduleConfig.vs.zeroStillMs) {
        strncpy(vsZeroMsg, "settling...", sizeof(vsZeroMsg) - 1);
        return;
    }
    // Quality: with the encoder still the truth is constant, so a wide spread means
    // the noise got through the median. Don't zero on that — fall back to GPS.
    if ((wMax - wMin) > moduleConfig.vs.zeroSpreadDeg) {
        armed = false;
        strncpy(vsZeroMsg, "WAS too noisy - not zeroing", sizeof(vsZeroMsg) - 1);
        return;
    }

    vsApplyZero(keyaEncoderRaw, wasAngle);
    keyaInitialZeroDone = true;
    vsZeroFromWas = true;
    vsWasOffset = 0.0f;
    armed = false;
    strncpy(vsZeroMsg, "zeroed from WAS", sizeof(vsZeroMsg) - 1);
    webLogf("VS: initial zero from WAS at %.2f deg - autosteer unlocked", wasAngle);
}

// ── Twin orbital: the active ticks/deg ───────────────────────────────────────
// Calibration is done in 125 ccm, so that is the base. 250 ccm passes twice the
// oil per steering-wheel turn → twice the wheel angle per turn → half the ticks
// per degree. The factor is geometrically exact from the displacements, not an
// empirical number, which is why correcting a wrong guess is exact too.
// Deliberately NOT gated on masterEnable: a manually chosen ratio is a calibration
// choice, not a WAS-derived guess. Killing it with the master switch would leave the
// angle 2x wrong whenever the tractor is actually in 250 ccm.
float vsRatioDiv()
{
    if (!moduleConfig.vs.orbitalEnable || moduleConfig.vs.orbitalMode == 0) return 1.0f;
    float r = moduleConfig.vs.orbitalRatio;
    return (r > 0.1f) ? r : 1.0f;
}

// ── Boot rule for the orbital ratio ──────────────────────────────────────────
// Manual selection is REMEMBERED: set 250 ccm once for the orchard and it survives
// power cycles. But when auto-detection is actually going to run, start from 125
// (the base) every time — a remembered mode would be confidently wrong whenever the
// tractor is started in the other one, and detection resolves it on the first turn
// anyway. Called from setup() after the config load.
void vsBootInit()
{
    bool autoWillRun = moduleConfig.vs.masterEnable
                    && moduleConfig.vs.wasPresent
                    && moduleConfig.vs.orbitalEnable
                    && moduleConfig.vs.orbitalAuto;
    if (autoWillRun && moduleConfig.vs.orbitalMode != 0) {
        moduleConfig.vs.orbitalMode = 0;                 // RAM only — no EEPROM wear
        Serial.println("VS: auto-detect active, orbital ratio starts at 125 ccm");
    }
}

// Base ticks/deg for zeroing, scaled by the active orbital ratio.
float vsTicksPerDeg()
{
    return moduleConfig.keyaTicksPerDeg / vsRatioDiv();
}

// The backlash dead zone is applied in OUTPUT degrees, but the free play is
// physically in the column/orbital — the same number of TICKS. Half the ticks/deg
// therefore means twice as many degrees.
float vsDeadZone()
{
    return moduleConfig.keyaDeadZone * vsRatioDiv();
}

// ── Switch the active ratio and repair the zero exactly ──────────────────────
// The zero inputs were kept, so this is arithmetic, not a new acquisition: no
// waiting for a still window, no gap in the angle. Because the ratio is exactly
// 2:1, the error is cancelled outright.
//
// Hard rule: NEVER call this while autosteer is engaged. Changing the gain under
// active control steps the feedback signal and jerks the wheel.
void vsSetOrbitalMode(uint8_t mode)
{
    if (mode == moduleConfig.vs.orbitalMode) return;
    moduleConfig.vs.orbitalMode = mode;
    // Recompute the zero for the new scale from the stored inputs.
    moduleConfig.keyaZeroTicks = vsZeroEncRaw - (int32_t)(vsZeroWasAngle * vsTicksPerDeg());
    vsWasOffset = 0.0f;             // the old trim belonged to the old scale
    vsOrbitalSwitches++;
    webLogf("VS: orbital ratio -> %s ccm (zero recomputed)", mode ? "250" : "125");
}

// Set keyaZeroTicks from a (ticks, angle) pair and REMEMBER the inputs, so the
// same zero can be recomputed exactly for a different ticks/deg later.
void vsApplyZero(int32_t encRaw, float angleDeg)
{
    vsZeroEncRaw   = encRaw;
    vsZeroWasAngle = angleDeg;
    moduleConfig.keyaZeroTicks = encRaw - (int32_t)(angleDeg * vsTicksPerDeg());
}
