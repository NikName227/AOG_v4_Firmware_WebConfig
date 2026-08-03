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
    if (!moduleConfig.vsMedianEnable) { adsMedCounts = raw; vsMedCnt = 0; vsMedIdx = 0; return; }

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
    if (fabs(moduleConfig.vsWasDegPerCount) < 1e-9f) return false;   // uncalibrated
    angleOut = moduleConfig.vsWasDegPerCount * (float)adsMedCounts
             + moduleConfig.vsWasIntercept;
    return true;
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

    float wasAngle;
    vsWasUsable = vsWasAngle(wasAngle) && fabs(wasAngle) < 90.0f;
    if (vsWasUsable) vsLastWasAngle = wasAngle;

    if (vsStep < VS_FUSE_STEP_MS) return;
    vsStep = 0;

    // Master switch off (or no usable WAS) → ramp the offset back out at the same
    // rate limit, so flipping the switch returns to stock behaviour without a step
    // in the feedback signal.
    if (!moduleConfig.vsFuseEnable || !vsWasUsable || !keyaInitialZeroDone) {
        vsGateBlocked = 0;
        float maxStep = moduleConfig.vsRateMaxDegS * (VS_FUSE_STEP_MS / 1000.0f);
        if      (vsWasOffset >  maxStep) vsWasOffset -= maxStep;
        else if (vsWasOffset < -maxStep) vsWasOffset += maxStep;
        else                             vsWasOffset  = 0.0f;
        return;
    }

    vsLastInnov = wasAngle - (predBase + vsWasOffset);

    // Rejection rate — how hard the noise is hitting right now (diagnostic).
    if (vsRejWindow > 1000) { vsRejectPerSec = vsRejCount; vsRejCount = 0; vsRejWindow = 0; }

    if (fabs(vsLastInnov) > moduleConfig.vsGateDeg) {
        vsRejCount++;
        return;                       // impossible jump — drop it, keep vsGateBlocked running
    }
    vsGateBlocked = 0;                // a sample got through: the anchor is alive

    // Gentler while engaged, exactly like the GPS auto-zero, so it cannot fight the PID.
    float k = (watchdogTimer < WATCHDOG_THRESHOLD) ? (moduleConfig.vsFuseBeta / 5.0f)
                                                   : moduleConfig.vsFuseBeta;
    float step = vsLastInnov * k;

    float maxStep = moduleConfig.vsRateMaxDegS * (VS_FUSE_STEP_MS / 1000.0f);
    if (step >  maxStep) step =  maxStep;
    if (step < -maxStep) step = -maxStep;

    vsWasOffset += step;
    if (vsWasOffset >  moduleConfig.vsOffsetMaxDeg) vsWasOffset =  moduleConfig.vsOffsetMaxDeg;
    if (vsWasOffset < -moduleConfig.vsOffsetMaxDeg) vsWasOffset = -moduleConfig.vsOffsetMaxDeg;
}
