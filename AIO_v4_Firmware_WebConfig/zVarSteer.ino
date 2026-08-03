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

// Explicit forward declarations rather than relying on the Arduino auto-prototype
// generator — its insertion point has already caused one build break in this sketch.
void  vsApplyZero(int32_t encRaw, float angleDeg);
void  vsSetOrbitalMode(uint8_t mode);
float vsTicksPerDeg();
float vsRatioDiv();

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

    // Master switch off (or no usable WAS) → ramp the offset back out at the same
    // rate limit, so flipping the switch returns to stock behaviour without a step
    // in the feedback signal.
    if (!moduleConfig.vs.fuseEnable || !vsWasUsable || !keyaInitialZeroDone) {
        vsGateBlocked = 0;
        float maxStep = moduleConfig.vs.rateMaxDegS * (VS_FUSE_STEP_MS / 1000.0f);
        if      (vsWasOffset >  maxStep) vsWasOffset -= maxStep;
        else if (vsWasOffset < -maxStep) vsWasOffset += maxStep;
        else                             vsWasOffset  = 0.0f;
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
    if (!moduleConfig.vs.zeroEnable || keyaInitialZeroDone) return;
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
float vsRatioDiv()
{
    if (!moduleConfig.vs.orbitalEnable || moduleConfig.vs.orbitalMode == 0) return 1.0f;
    float r = moduleConfig.vs.orbitalRatio;
    return (r > 0.1f) ? r : 1.0f;
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
