// zWasFuse.h - WAS source "Keya + ADS": the Keya encoder gives the angle, the ADS1115 WAS pulls its offset slowly.
// Pure logic, no Arduino dependency, so testing/wasfuse can run it on a PC.
//
//   out = keyaAngle + offset         offset follows (adsAngle - keyaAngle) with time constant tau, at most maxRate deg/s
//
// An ADS sample is a disturbance when it differs from the last accepted one (moved on by the Keya change since then)
// by more than gate degrees: it is dropped and the offset stays. A disturbance that lasts longer than failMs is no
// disturbance but a fault: the caller stops the steering ("ADS unstable"). Nothing falls back to the other sensor.
#ifndef Z_WASFUSE_H
#define Z_WASFUSE_H

#include <stdint.h>
#include <math.h>

#define WF_STORE_MAGIC 0xF5E1

struct WasFuseCfg {                 // stored at the end of WasCalStore
    uint16_t magic;
    uint16_t failMs;                // continuous dropping until "ADS unstable"
    float    gate;                  // deg per step
    float    tau;                   // s
    float    maxRate;               // deg/s
};
inline void wfDefaults(WasFuseCfg &c) { c.magic = WF_STORE_MAGIC; c.failMs = 1000; c.gate = 1.5f; c.tau = 3.0f; c.maxRate = 0.5f; }
inline bool wfCfgOk(const WasFuseCfg &c)
{
    return c.magic == WF_STORE_MAGIC && c.failMs >= 100 && c.failMs <= 10000 && c.gate >= 0.1f && c.gate <= 20.0f
        && c.tau >= 0.2f && c.tau <= 120.0f && c.maxRate >= 0.01f && c.maxRate <= 10.0f;
}

struct WasFuse {
    bool     init = false;
    float    offset = 0, accAds = 0, accKeya = 0;   // last accepted ADS sample and the Keya angle at that time
    float    out = 0;
    uint32_t dropped = 0;                           // samples dropped as disturbance (since reset)
    uint32_t dropSince = 0;                         // ms when the current run of drops began (0 = none)
    bool     fault = false;                         // true while the drops last longer than failMs
    float    lastInnov = 0;

    void reset() { *this = WasFuse(); }

    // dt in s, now in ms. Returns the fused angle.
    float step(const WasFuseCfg &c, uint32_t now, float dt, float adsAng, float keyaAng)
    {
        if (!init) { init = true; offset = adsAng - keyaAng; accAds = adsAng; accKeya = keyaAng; out = adsAng; return out; }
        float expected = accAds + (keyaAng - accKeya);
        lastInnov = adsAng - expected;
        if (fabsf(lastInnov) > c.gate) {
            dropped++;
            if (!dropSince) dropSince = now | 1;
            fault = (int32_t)(now - dropSince) > (int32_t)c.failMs;
        } else {
            dropSince = 0; fault = false;
            accAds = adsAng; accKeya = keyaAng;
            float a = dt / c.tau; if (a > 1.0f) a = 1.0f;
            float d = a * ((adsAng - keyaAng) - offset);
            float lim = c.maxRate * dt;
            if (d > lim) d = lim; else if (d < -lim) d = -lim;
            offset += d;
        }
        out = keyaAng + offset;
        return out;
    }
};

#endif
