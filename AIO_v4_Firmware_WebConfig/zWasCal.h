// ─────────────────────────────────────────────────────────────────────────────
// WAS calibration core (F11) — analog WAS (ADS1115) → bicycle angle table.
//
// Pure logic, no Arduino dependency: the same file is compiled on a PC by the
// test harness (testing/wascal/), the firmware glue lives in zWasCal.ino.
//
// Calibration: stationary, motor OFF, the operator turns the steering wheel
// lock-to-lock by hand. The ESP32 reference IMU (PGN 0xD6, refWheelAngle) sits
// on one front wheel: measurement 1 on the RIGHT wheel, measurement 2 on the
// LEFT wheel. Samples are binned by ADS raw (128 counts), per rising / falling raw,
// so the two directions can be averaged (cancels linkage play and link latency).
// With both wheels the bicycle angle needs no track width:
//     tan(bike) = 2·tanR·tanL / (tanR + tanL)        (cot bike = mean of cots)
// With one wheel only, wheelToBike(L, T) is used instead.
// Result: up to 23 points (end, 21 × 5° grid, end) raw ↔ bike angle, stored in
// its own EEPROM block. Outside the measured range the angle is clamped.
// The right / left wheel angles at the same points are stored too (display only:
// the page shows the calibrated wheels; the bicycle angle is what goes to AOG).
// If the Keya encoder is present, the same turning also gives ticks per bike
// degree per side and the reachable maximum, used to extend the range past the
// end of the analog sensor (blend near the ends, or from a set handover angle,
// clamp at the measured lock).
// ─────────────────────────────────────────────────────────────────────────────
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define WC_BIN_SHIFT    7          // 128 raw counts per bin (~1° on a typical sensor)
#define WC_NBINS        128        // raw 0..16383 (ADS >> 1)
#define WC_NGRID        21         // −50..+50° in 5° steps
#define WC_NPTS         (WC_NGRID + 2)
#define WC_GRID_STEP    5.0f
#define WC_MIN_BIN_N    2          // samples per bin and direction
#define WC_MAX_BIN_STD  2.0f       // deg — hard limit on the wheel spread inside one bin
#define WC_MAX_RATE     20.0f      // deg/s wheel rate — faster samples are skipped
#define WC_DIR_HYST     16         // raw counts before the turning direction flips
#define WC_SIGN_DEG     5.0f       // first turn RIGHT past this many degrees sets the signs
#define WC_STORE_MAGIC  0xC511

// ── Persisted block (own EEPROM address, own magic) ──────────────────────────
struct WasCalStore {
    uint16_t magic;
    uint8_t  useTable;          // 1 = ADS angle from the table, 0 = AOG counts/Ackermann/offset
    uint8_t  keyaExtend;        // 1 = Keya encoder takes over past the table ends
    uint8_t  nPts;              // valid points in ang/raw (0 = no calibration)
    uint8_t  twoWheel;          // calibration used both wheels
    uint8_t  wheelMask;         // wR/wL measured: bit0 right, bit1 left (0 = none, older block)
    uint8_t  pad;
    float    ang[WC_NPTS];      // bike angle, ascending (right = +)
    float    raw[WC_NPTS];      // ADS raw at that angle (monotonic)
    float    zeroShift;         // raw counts, from the straight-driving zero
    float    azShift;           // raw counts, slow auto-zero (table mode)
    float    keyaTpdL, keyaTpdR;   // Keya ticks per bike degree (signed), 0 = unknown
    float    keyaMaxL, keyaMaxR;   // reachable bike angle (L negative), 0 = unknown
    float    blendDeg;          // ADS → Keya blend width before a table end
    float    zYawMax;           // straight zero: mean heading change limit (deg/s)
    float    zSpeedMin;         // km/h
    uint16_t zTimeMs;           // window length
    uint16_t handX10;           // v1.0.12: Keya handover angle ×10 (0 = at the table ends; was pad, 0 in older blocks)
    float    rms, hyst;         // quality of the applied calibration (deg)
    // v1.0.10 — appended so an older block still loads (wheelMask was pad = 0)
    float    wR[WC_NPTS], wL[WC_NPTS];   // right / left wheel angle at raw[i] (display only)
};

inline void wcStoreDefaults(WasCalStore &s) {
    memset(&s, 0, sizeof(s));
    s.magic     = WC_STORE_MAGIC;
    s.blendDeg  = 5.0f;
    s.zYawMax   = 0.1f;
    s.zSpeedMin = 3.0f;
    s.zTimeMs   = 8000;
}

// ── Table lookup ─────────────────────────────────────────────────────────────
// raw → angle with linear interpolation; outside the table the end angle is
// returned and region says which end (−1 left/ang[0], +1 right/ang[n-1]).
inline float wcLookup(const float *ang, const float *raw, uint8_t n, float x, int8_t *region) {
    if (region) *region = 0;
    if (n < 2) return 0.0f;
    bool inc = raw[n - 1] > raw[0];
    float lo = inc ? raw[0] : raw[n - 1];
    float hi = inc ? raw[n - 1] : raw[0];
    if (x <= lo) { if (region) *region = inc ? -1 : +1; return inc ? ang[0] : ang[n - 1]; }
    if (x >= hi) { if (region) *region = inc ? +1 : -1; return inc ? ang[n - 1] : ang[0]; }
    for (uint8_t k = 0; k + 1 < n; k++) {
        float a = raw[k], b = raw[k + 1];
        if ((x >= a && x <= b) || (x <= a && x >= b)) {
            float d = b - a;
            if (fabsf(d) < 1e-6f) return ang[k];
            return ang[k] + (x - a) * (ang[k + 1] - ang[k]) / d;
        }
    }
    return 0.0f;
}

// angle → raw (inverse), clamped to the table.
inline float wcAngleToRaw(const float *ang, const float *raw, uint8_t n, float a) {
    if (n < 2) return 0.0f;
    if (a <= ang[0])     return raw[0];
    if (a >= ang[n - 1]) return raw[n - 1];
    for (uint8_t k = 0; k + 1 < n; k++)
        if (a >= ang[k] && a <= ang[k + 1]) {
            float d = ang[k + 1] - ang[k];
            if (d < 1e-6f) return raw[k];
            return raw[k] + (a - ang[k]) * (raw[k + 1] - raw[k]) / d;
        }
    return raw[n - 1];
}

// Raw counts per degree around 0° (signed), for the raw-domain auto-zero.
inline float wcCountsPerDegAtZero(const WasCalStore &s) {
    if (s.nPts < 2) return 0.0f;
    return (wcAngleToRaw(s.ang, s.raw, s.nPts, 1.0f) - wcAngleToRaw(s.ang, s.raw, s.nPts, -1.0f)) * 0.5f;
}

// ADS angle from the table, with both zero shifts.
inline float wcAdsAngle(const WasCalStore &s, float rawCounts, int8_t *region) {
    return wcLookup(s.ang, s.raw, s.nPts, rawCounts - s.zeroShift - s.azShift, region);
}

// ── Wheel → bicycle angle, one wheel only (magnitude in, magnitude out) ──────
//   tan(bike) = L / R ; inner: R = L/tan(w) + T/2 ; outer: R = L/tan(w) − T/2
inline float wcWheelToBike(float wMagDeg, bool inner, float L, float T) {
    if (L < 0.1f) return wMagDeg;
    float t = tanf(wMagDeg * 0.01745329252f);
    float denom = L + (inner ? +1.0f : -1.0f) * (T * 0.5f) * t;
    if (denom < 0.01f) return wMagDeg;
    return atanf(L * t / denom) * 57.2957795f;
}

// Bicycle angle (signed) → right and left wheel (ideal Ackermann, L, T).
inline void wcBikeToWheels(float bDeg, float L, float T, float &wr, float &wl) {
    if (fabsf(bDeg) < 0.01f || L < 0.1f) { wr = wl = bDeg; return; }
    float R = L / tanf(fabsf(bDeg) * 0.01745329252f);
    float in = atanf(L / (R - T * 0.5f)) * 57.2957795f, out = atanf(L / (R + T * 0.5f)) * 57.2957795f;
    if (R - T * 0.5f <= 0.01f) in = 89.0f;
    if (bDeg > 0) { wr = in; wl = out; } else { wr = -out; wl = -in; }
}

// Both wheels (signed, same steering state) → bicycle angle, no track needed.
inline float wcCombineWheels(float aDeg, float bDeg) {
    if (aDeg * bDeg <= 0.0f || (fabsf(aDeg) < 0.5f && fabsf(bDeg) < 0.5f))
        return 0.5f * (aDeg + bDeg);                 // around centre: plain mean
    float ta = tanf(aDeg * 0.01745329252f), tb = tanf(bDeg * 0.01745329252f);
    return atanf(2.0f * ta * tb / (ta + tb)) * 57.2957795f;
}

// ── Recording ────────────────────────────────────────────────────────────────
struct WcBin {
    uint16_t n, nt;             // samples, samples with Keya ticks
    float    sw, sww;           // wheel angle sum / sum of squares (deg, as measured)
    float    sr;                // raw sum, relative to the bin start (keeps float exact)
    double   srr;               // raw sum of squares, relative to the bin start
    double   st;                // Keya ticks sum, relative to the session centre
};

struct WcSession {
    uint8_t  state;             // 0 empty, 1 recording, 2 done
    int8_t   wSign, rSign;      // set by the first clear RIGHT turn
    int8_t   dir;               // +1 raw rising, −1 falling, 0 not moved yet
    int16_t  ext;               // raw extreme since the last direction change
    int16_t  rawStart;
    float    refCenter;         // reference angle at start (wheels straight)
    int32_t  tickCenter, tickLo, tickHi;
    int16_t  rawLo, rawHi;      // raw range seen
    float    wLo, wHi;          // wheel range seen (as measured)
    bool     ticks;             // Keya ticks recorded in this session
    bool     haveLast;
    float    lastW, rate;       // wheel rate (deg/s, smoothed)
    uint32_t nOk, nFast, nStale;
    uint8_t  cov[WC_NGRID];     // per 5° grid point: bit0 rising, bit1 falling
};

struct WcResult {
    bool     ok;
    char     msg[96];
    uint8_t  nPts, twoWheel, wheelMask;
    float    ang[WC_NPTS], raw[WC_NPTS];
    float    wR[WC_NPTS], wL[WC_NPTS];   // wheel angles at the table points
    float    rms, hystMean, hystMax, relOffset;
    int      inversions, nBins, nSat;
    bool     keyaOk;
    float    keyaTpdL, keyaTpdR, keyaMaxL, keyaMaxR;
    uint16_t nCurve;            // measured points (raw, bike) for the plot
    float    curveRaw[WC_NBINS], curveBike[WC_NBINS];
};

struct WcCal {
    WcBin     bins[2][2][WC_NBINS];   // [session: 0 right wheel, 1 left wheel][dir][bin]
    WcSession ses[2];
    int8_t    active;                  // session being recorded, −1 none
    // scratch for compute()
    float     w[2][WC_NBINS], r[2][WC_NBINS], t[2][WC_NBINS], h[2][WC_NBINS];
    float     bike[WC_NBINS], braw[WC_NBINS], bwr[WC_NBINS], bwl[WC_NBINS];

    void reset() {
        memset(bins, 0, sizeof(bins));
        memset(ses, 0, sizeof(ses));
        active = -1;
    }

    void start(uint8_t wheel, float ref, int16_t raw, bool hasTicks, int32_t ticks) {
        if (wheel > 1) return;
        memset(bins[wheel], 0, sizeof(bins[wheel]));
        WcSession &s = ses[wheel];
        memset(&s, 0, sizeof(s));
        s.state = 1;
        s.refCenter = ref;
        s.rawStart = raw;
        s.ext = raw;
        s.rawLo = s.rawHi = raw;
        s.ticks = hasTicks;
        s.tickCenter = ticks;
        active = (int8_t)wheel;
    }

    void stop() {
        if (active >= 0) ses[active].state = 2;
        active = -1;
    }

    // One sample (every autosteer loop, ~40 Hz). ref = reference wheel angle (deg), raw = ADS raw.
    void sample(float ref, int16_t raw, bool tickOk, int32_t ticks, float dt) {
        if (active < 0) return;
        WcSession &s = ses[active];
        float w = ref - s.refCenter;
        if (s.haveLast && dt > 0.001f) {
            float rt = fabsf(w - s.lastW) / dt;
            s.rate = s.rate * 0.7f + rt * 0.3f;
        }
        s.lastW = w; s.haveLast = true;

        if (s.wSign == 0 && fabsf(w) >= WC_SIGN_DEG) {          // first turn is RIGHT
            s.wSign = (w > 0) ? 1 : -1;
            int d = raw - s.rawStart;
            s.rSign = (d > 0) ? 1 : (d < 0 ? -1 : 0);
        }

        if (s.dir == 0) {
            if      (raw - s.rawStart > WC_DIR_HYST) { s.dir = +1; s.ext = raw; }
            else if (s.rawStart - raw > WC_DIR_HYST) { s.dir = -1; s.ext = raw; }
        } else if (s.dir > 0) {
            if (raw > s.ext) s.ext = raw;
            else if (s.ext - raw > WC_DIR_HYST) { s.dir = -1; s.ext = raw; }
        } else {
            if (raw < s.ext) s.ext = raw;
            else if (raw - s.ext > WC_DIR_HYST) { s.dir = +1; s.ext = raw; }
        }

        if (s.rate > WC_MAX_RATE) { s.nFast++; return; }
        if (s.dir == 0) return;

        int b = (raw < 0) ? 0 : (raw >> WC_BIN_SHIFT);
        if (b >= WC_NBINS) b = WC_NBINS - 1;
        WcBin &c = bins[active][s.dir > 0 ? 0 : 1][b];
        if (c.n >= 65000) return;
        c.n++;
        c.sw += w; c.sww += w * w;
        float rr = (float)(raw - (b << WC_BIN_SHIFT));
        c.sr += rr; c.srr += (double)rr * rr;
        if (raw < s.rawLo) s.rawLo = raw;
        if (raw > s.rawHi) s.rawHi = raw;
        if (w < s.wLo) s.wLo = w;
        if (w > s.wHi) s.wHi = w;
        if (s.ticks && tickOk) {
            int32_t rel = ticks - s.tickCenter;
            c.nt++; c.st += rel;
            if (rel < s.tickLo) s.tickLo = rel;
            if (rel > s.tickHi) s.tickHi = rel;
        }
        s.nOk++;

        if (s.wSign != 0) {
            float ws = w * s.wSign;
            int g = (int)lroundf(ws / WC_GRID_STEP) + WC_NGRID / 2;
            if (g >= 0 && g < WC_NGRID && fabsf(ws - (g - WC_NGRID / 2) * WC_GRID_STEP) <= 1.5f)
                s.cov[g] |= (s.dir > 0) ? 1 : 2;
        }
    }

    // Per-session bin values: wheel angle (sign-corrected, mean of both directions),
    // mean raw, mean ticks, hysteresis. NaN = no valid value.
    // A bin is rejected when the wheel moved much more than its raw spread can
    // explain (raw stuck at the sensor end = saturation), using the session's mean
    // counts per degree.
    void binValues(uint8_t si, int &nSat) {
        const WcSession &s = ses[si];
        float wRange = s.wHi - s.wLo;
        float cpd = (wRange > 1.0f) ? (float)(s.rawHi - s.rawLo) / wRange : 100.0f;
        if (cpd < 1.0f) cpd = 1.0f;
        for (int b = 0; b < WC_NBINS; b++) {
            w[si][b] = r[si][b] = t[si][b] = h[si][b] = NAN;
            float m[2], rr[2]; bool ok[2]; double ts = 0; uint32_t tn = 0;
            for (int d = 0; d < 2; d++) {
                const WcBin &c = bins[si][d][b];
                ok[d] = false;
                if (c.n < WC_MIN_BIN_N) continue;
                float mean = c.sw / c.n;
                float var = c.sww / c.n - mean * mean;
                float sd = var > 0 ? sqrtf(var) : 0;
                float rm = c.sr / c.n;
                float rvar = (float)(c.srr / c.n) - rm * rm;
                float rsd = rvar > 0 ? sqrtf(rvar) : 0;
                if (sd > WC_MAX_BIN_STD || sd > 0.3f + 3.0f * rsd / cpd) { nSat++; continue; }
                m[d] = mean; rr[d] = c.sr / c.n + (float)(b << WC_BIN_SHIFT); ok[d] = true;
                ts += c.st; tn += c.nt;
            }
            if (!ok[0] && !ok[1]) continue;
            if (ok[0] && ok[1]) {
                w[si][b] = 0.5f * (m[0] + m[1]) * s.wSign;
                r[si][b] = 0.5f * (rr[0] + rr[1]);
                h[si][b] = fabsf(m[0] - m[1]);
            } else {
                int d = ok[0] ? 0 : 1;
                w[si][b] = m[d] * s.wSign;
                r[si][b] = rr[d];
            }
            if (tn > 0) t[si][b] = (float)(ts / tn);
        }
    }

    // Build the table. L, T only used with a single wheel.
    bool compute(WcResult &R, float L, float T) {
        memset(&R, 0, sizeof(R));
        bool use[2];
        for (int i = 0; i < 2; i++) use[i] = (ses[i].state == 2 && ses[i].wSign != 0 && ses[i].nOk > 20);
        if (!use[0] && !use[1]) { strcpy(R.msg, "no measurement - turn RIGHT first, then lock to lock"); return false; }
        if (use[0] && use[1] && ses[0].rSign != 0 && ses[1].rSign != 0 && ses[0].rSign != ses[1].rSign) {
            strcpy(R.msg, "measurements disagree on direction - start each by turning RIGHT");
            return false;
        }
        int nSat = 0;
        for (int i = 0; i < 2; i++) if (use[i]) binValues(i, nSat);
        R.nSat = nSat;
        R.twoWheel = (use[0] && use[1]) ? 1 : 0;

        // Relative offset between the two measurements (each started "straight")
        float d = 0;
        if (R.twoWheel) {
            double sd = 0; int nd = 0;
            for (int b = 0; b < WC_NBINS; b++)
                if (!isnan(w[0][b]) && !isnan(w[1][b]) && fabsf(0.5f * (w[0][b] + w[1][b])) < 4.0f) {
                    sd += w[0][b] - w[1][b]; nd++;
                }
            if (nd >= 3) d = (float)(sd / nd);
            R.relOffset = d;
        }

        // Bicycle angle per bin
        // Wheel angles too (display): both measured with two wheels, otherwise the
        // measured one plus the other from Ackermann (L, T).
        R.wheelMask = R.twoWheel ? 3 : (use[0] ? 1 : 2);
        int nb = 0; double hs = 0; int hn = 0;
        for (int b = 0; b < WC_NBINS; b++) {
            float bk = NAN, rw = NAN, wrv = NAN, wlv = NAN;
            if (R.twoWheel) {
                if (isnan(w[0][b]) || isnan(w[1][b])) continue;
                wrv = w[0][b] - 0.5f * d; wlv = w[1][b] + 0.5f * d;
                bk = wcCombineWheels(wrv, wlv);
                rw = 0.5f * (r[0][b] + r[1][b]);
            } else {
                int si = use[0] ? 0 : 1;
                if (isnan(w[si][b])) continue;
                float wv = w[si][b];
                bool inner = (si == 0) ? (wv > 0) : (wv < 0);   // right wheel is inner in a right turn
                float bm = wcWheelToBike(fabsf(wv), inner, L, T);
                bk = (wv < 0) ? -bm : bm;
                rw = r[si][b];
                wcBikeToWheels(bk, L, T, wrv, wlv);
                if (si == 0) wrv = wv; else wlv = wv;
            }
            bwr[nb] = wrv; bwl[nb] = wlv;
            for (int i = 0; i < 2; i++) if (use[i] && !isnan(h[i][b])) {
                hs += h[i][b]; hn++;
                if (h[i][b] > R.hystMax) R.hystMax = h[i][b];
            }
            bike[nb] = bk; braw[nb] = rw; nb++;
        }
        R.hystMean = hn ? (float)(hs / hn) : 0;
        R.nBins = nb;
        if (nb < 5) { strcpy(R.msg, "too few points - turn slower, lock to lock, both directions"); return false; }

        // Monotonic bike(raw): keep points that keep moving the same way
        float sgn = (bike[nb - 1] > bike[0]) ? 1.0f : -1.0f;
        int m = 0;
        for (int i = 0; i < nb; i++) {
            if (m == 0 || (bike[i] - bike[m - 1]) * sgn > 0.05f) { bike[m] = bike[i]; braw[m] = braw[i]; bwr[m] = bwr[i]; bwl[m] = bwl[i]; m++; }
            else R.inversions++;
        }
        if (m < 5 || R.inversions > nb / 5) { strcpy(R.msg, "angle not monotonic in raw - check sensor / reference mount"); return false; }
        R.nCurve = (uint16_t)m;
        for (int i = 0; i < m; i++) { R.curveRaw[i] = braw[i]; R.curveBike[i] = bike[i]; }

        // Ascending in angle
        float A[WC_NBINS], Rw[WC_NBINS], WR[WC_NBINS], WL[WC_NBINS];
        for (int i = 0; i < m; i++) {
            int j = (sgn > 0) ? i : (m - 1 - i);
            A[i] = bike[j]; Rw[i] = braw[j]; WR[i] = bwr[j]; WL[i] = bwl[j];
        }
        if (A[m - 1] - A[0] < 10.0f) { strcpy(R.msg, "range too small - turn further to both locks"); return false; }

        uint8_t n = 0;
        R.ang[n] = A[0]; R.raw[n] = Rw[0]; R.wR[n] = WR[0]; R.wL[n] = WL[0]; n++;
        int k = 0;
        for (int g = 0; g < WC_NGRID; g++) {
            float a = (g - WC_NGRID / 2) * WC_GRID_STEP;
            if (a <= A[0] + 0.5f || a >= A[m - 1] - 0.5f) continue;
            while (k + 1 < m && A[k + 1] < a) k++;
            float da = A[k + 1] - A[k];
            float f = (da > 1e-6f) ? (a - A[k]) / da : 0.0f;
            R.ang[n] = a;
            R.raw[n] = Rw[k] + f * (Rw[k + 1] - Rw[k]);
            R.wR[n]  = WR[k] + f * (WR[k + 1] - WR[k]);
            R.wL[n]  = WL[k] + f * (WL[k + 1] - WL[k]);
            n++;
        }
        R.ang[n] = A[m - 1]; R.raw[n] = Rw[m - 1]; R.wR[n] = WR[m - 1]; R.wL[n] = WL[m - 1]; n++;
        R.nPts = n;

        double se = 0;
        for (int i = 0; i < m; i++) {
            float e = wcLookup(R.ang, R.raw, n, Rw[i], 0) - A[i];
            se += e * e;
        }
        R.rms = (float)sqrt(se / m);

        keyaFit(R, use);

        snprintf(R.msg, sizeof(R.msg), "OK %s: %.1f..%.1f deg, %u pts, RMS %.2f, hyst %.2f",
                 R.twoWheel ? "2 wheels" : "1 wheel", R.ang[0], R.ang[n - 1], n, R.rms, R.hystMean);
        R.ok = true;
        return true;
    }

    // Keya ticks per bike degree per side, and the reachable bike angle at each lock.
    void keyaFit(WcResult &R, const bool *use) {
        float tpd[2][2] = {{0, 0}, {0, 0}};      // [session][side 0 left, 1 right]
        float mx[2][2]  = {{0, 0}, {0, 0}};
        int   nOk[2] = {0, 0};
        float d = R.relOffset;
        for (int si = 0; si < 2; si++) {
            if (!use[si] || !ses[si].ticks) continue;
            for (int side = 0; side < 2; side++) {
                double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
                for (int b = 0; b < WC_NBINS; b++) {
                    if (isnan(t[si][b]) || isnan(w[si][b])) continue;
                    float bk;
                    if (R.twoWheel) {
                        if (isnan(w[1 - si][b])) continue;
                        bk = wcCombineWheels(w[0][b] - 0.5f * d, w[1][b] + 0.5f * d);
                    } else {
                        bk = NAN;
                        for (int i = 0; i < R.nCurve; i++)
                            if (fabsf(R.curveRaw[i] - r[si][b]) < 1.0f) { bk = R.curveBike[i]; break; }
                        if (isnan(bk)) continue;
                    }
                    if (side == 1 ? (bk < 3.0f) : (bk > -3.0f)) continue;
                    n++; sx += bk; sy += t[si][b]; sxx += bk * bk; sxy += bk * t[si][b];
                }
                double den = n * sxx - sx * sx;
                if (n < 5 || fabs(den) < 1e-9) continue;
                float slope = (float)((n * sxy - sx * sy) / den);   // ticks per bike degree
                if (fabsf(slope) < 0.1f) continue;
                tpd[si][side] = slope;
                // tick at the table end on this side → extrapolate to the session's tick extreme
                float endA = side ? R.ang[R.nPts - 1] : R.ang[0];
                float endRaw = side ? R.raw[R.nPts - 1] : R.raw[0];
                int eb = (int)endRaw >> WC_BIN_SHIFT;
                float tEnd = NAN;
                for (int db = 0; db <= 2 && isnan(tEnd); db++) {
                    int b1 = eb - db, b2 = eb + db;
                    if (b1 >= 0 && !isnan(t[si][b1])) tEnd = t[si][b1];
                    else if (b2 < WC_NBINS && !isnan(t[si][b2])) tEnd = t[si][b2];
                }
                if (isnan(tEnd)) continue;
                // the lock on this side is where ticks went furthest in this side's direction
                bool towardsHi = (side == 1) ? (slope > 0) : (slope < 0);
                float tExt = (float)(towardsHi ? ses[si].tickHi : ses[si].tickLo);
                float a = endA + (tExt - tEnd) / slope;
                if (side == 1 ? (a < endA) : (a > endA)) a = endA;
                mx[si][side] = a;
                nOk[si]++;
            }
        }
        for (int side = 0; side < 2; side++) {
            float st = 0, sm = 0; int c = 0;
            for (int si = 0; si < 2; si++) if (tpd[si][side] != 0 && mx[si][side] != 0) { st += tpd[si][side]; sm += mx[si][side]; c++; }
            if (!c) continue;
            if (side) { R.keyaTpdR = st / c; R.keyaMaxR = sm / c; }
            else      { R.keyaTpdL = st / c; R.keyaMaxL = sm / c; }
        }
        R.keyaOk = (R.keyaTpdL != 0 && R.keyaTpdR != 0);
    }
};

// ── Straight-driving zero ────────────────────────────────────────────────────
// Window counts while the speed is up, the wheel is still and the heading has
// not moved more than zYawMax·window since the window started (mean yaw rate).
// One finished window = one pass; drive the line both ways and average.
struct WcZero {
    uint8_t  running;
    uint8_t  nPass;
    uint8_t  tableMode;         // mode the passes were taken in
    uint32_t tStart;
    float    hdgStart;
    double   sRaw, sAng, sAng2;
    uint32_t n;
    float    passRaw[4], passAng[4], passHdg[4], passStd[4];
    float    progress;          // 0..1 of the current window
    char     msg[80];
};

inline float wcWrap180(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }

inline void wcZeroStart(WcZero &z, bool tableMode) {
    if (z.tableMode != (tableMode ? 1 : 0)) z.nPass = 0;   // passes from the other mode are void
    z.tableMode = tableMode ? 1 : 0;
    z.running = 1; z.n = 0; z.tStart = 0; z.progress = 0;
    strcpy(z.msg, "waiting for straight driving");
}

inline void wcZeroClear(WcZero &z) { memset(&z, 0, sizeof(z)); strcpy(z.msg, "idle"); }

// yawMaxEff already includes any relaxing for a VTG heading.
inline void wcZeroStep(WcZero &z, uint32_t nowMs, float raw, float ang, float hdg,
                       float speed, float steerRate, float yawMaxEff, float speedMin, uint16_t winMs) {
    if (!z.running) return;
    bool ok = (speed >= speedMin) && (fabsf(steerRate) < 5.0f);
    if (ok && z.n > 0) {
        float dh = fabsf(wcWrap180(hdg - z.hdgStart));
        if (dh > yawMaxEff * winMs / 1000.0f) ok = false;      // heading moved → not straight
    }
    if (!ok) {
        if (z.n > 0) strcpy(z.msg, speed < speedMin ? "too slow - window restarted" : "not straight - window restarted");
        z.n = 0; z.progress = 0;
        if (!(speed >= speedMin && fabsf(steerRate) < 5.0f)) return;
    }
    if (z.n == 0) { z.tStart = nowMs; z.hdgStart = hdg; z.sRaw = z.sAng = z.sAng2 = 0; strcpy(z.msg, "measuring"); }
    z.sRaw += raw; z.sAng += ang; z.sAng2 += (double)ang * ang; z.n++;
    uint32_t el = nowMs - z.tStart;
    z.progress = (float)el / winMs; if (z.progress > 1) z.progress = 1;
    if (el >= winMs && z.n >= 10) {
        float mr = (float)(z.sRaw / z.n), ma = (float)(z.sAng / z.n);
        float var = (float)(z.sAng2 / z.n) - ma * ma;
        uint8_t i = z.nPass < 4 ? z.nPass : 3;
        z.passRaw[i] = mr; z.passAng[i] = ma; z.passHdg[i] = z.hdgStart; z.passStd[i] = var > 0 ? sqrtf(var) : 0;
        if (z.nPass < 4) z.nPass++;
        z.running = 0; z.progress = 1;
        snprintf(z.msg, sizeof(z.msg), "pass %u done: angle %.2f deg (sd %.2f)", z.nPass, ma, z.passStd[i]);
    }
}

inline float wcZeroMeanRaw(const WcZero &z) { float s = 0; for (int i = 0; i < z.nPass; i++) s += z.passRaw[i]; return z.nPass ? s / z.nPass : 0; }
inline float wcZeroMeanAng(const WcZero &z) { float s = 0; for (int i = 0; i < z.nPass; i++) s += z.passAng[i]; return z.nPass ? s / z.nPass : 0; }

// Table mode: shift so that the mean straight raw reads 0°. Clears the auto-zero.
inline bool wcZeroApplyTable(WasCalStore &s, const WcZero &z) {
    if (!z.nPass || s.nPts < 2) return false;
    float raw0 = wcAngleToRaw(s.ang, s.raw, s.nPts, 0.0f);
    s.zeroShift = wcZeroMeanRaw(z) - raw0;
    s.azShift = 0;
    return true;
}

// ── Keya extends the range past the table ends ───────────────────────────────
struct WcKeyaExt {
    bool    init;
    float   keyaAng;            // Keya-integrated bike angle, anchored to the ADS in the middle
    int32_t prevTicks;
    float   wAds;               // last ADS weight (1 = ADS only), for display
};

// adsAng must come from wcAdsAngle (clamped at the ends). Returns the angle to use.
inline float wcKeyaBlend(const WasCalStore &s, WcKeyaExt &e, float adsAng, bool keyaOk, int32_t ticks) {
    e.wAds = 1.0f;
    if (!s.keyaExtend || s.nPts < 2 || s.keyaTpdL == 0 || s.keyaTpdR == 0 || !keyaOk) { e.init = false; return adsAng; }
    if (!e.init) { e.keyaAng = adsAng; e.prevTicks = ticks; e.init = true; }
    int32_t dT = ticks - e.prevTicks;
    e.prevTicks = ticks;
    float tpd = (e.keyaAng >= 0) ? s.keyaTpdR : s.keyaTpdL;
    e.keyaAng += (float)dT / tpd;

    float endL = s.ang[0], endR = s.ang[s.nPts - 1], b = s.blendDeg > 0.5f ? s.blendDeg : 0.5f;
    float lo = s.keyaMaxL < endL ? s.keyaMaxL : endL;
    float hi = s.keyaMaxR > endR ? s.keyaMaxR : endR;
    // ramp start: blend before the table end, or the handover angle when it is set
    // (never later than that, the ADS is clamped at the end)
    float stR = endR - b, stL = endL + b, h = s.handX10 / 10.0f;
    if (h > 0) { if (h < stR) stR = h; if (-h > stL) stL = -h; }
    if (stR < 0) stR = 0;
    if (stL > 0) stL = 0;
    if (adsAng > stL && adsAng < stR) { e.keyaAng = adsAng; return adsAng; }   // middle: anchor

    float w = (adsAng >= 0) ? (stR + b - adsAng) / b : (adsAng - (stL - b)) / b;
    if (w < 0) w = 0;
    if (w > 1) w = 1;
    // Keya cannot be on the other side of the ADS's own zone (encoder ran away) → fall back
    if (adsAng >= 0 && e.keyaAng < stR - 2.0f) e.keyaAng = adsAng;
    if (adsAng <  0 && e.keyaAng > stL + 2.0f) e.keyaAng = adsAng;
    if (e.keyaAng > hi) e.keyaAng = hi;
    if (e.keyaAng < lo) e.keyaAng = lo;
    e.wAds = w;
    return w * adsAng + (1.0f - w) * e.keyaAng;
}
