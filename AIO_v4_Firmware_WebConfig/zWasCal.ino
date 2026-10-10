// ─────────────────────────────────────────────────────────────────────────────
// WAS calibration — firmware glue for zWasCal.h (F11)
//
//   • table + settings in their own EEPROM block (EEP_WASCAL_ADDR, own magic), so
//     ModuleConfig and every existing setting stay untouched
//   • recording: called every autosteer loop while a measurement runs (stationary,
//     autosteer off, reference IMU fresh)
//   • ADS angle: table mode (raw → bike angle, AOG counts / Ackermann / offset /
//     invert not used) or the old AOG formula; each mode keeps its own zero
//   • straight-driving zero (heading from the selected heading source)
//   • Keya extends the range past the table ends (optional)
//   • /api/wascal (commands) and /api/wcstat (JSON status)
// ─────────────────────────────────────────────────────────────────────────────

WasCalStore wasCal;
DMAMEM WcCal    wcCal;           // ~22 KB recording bins — RAM2, RAM1 is tight
DMAMEM WcResult wcRes;           // last computed (not yet applied) calibration
bool      wcResValid  = false;
WcZero    wcZero;
WcKeyaExt wcKeyaExt;
char      wcMsg[96]   = "idle";

// Live values for the page (refreshed every loop while an ADS is present)
float  wcAngTable = 0, wcAngAog = 0, wcAngOut = 0;
int8_t wcRegion   = 0;
float  wcRawAvg   = 0;           // ADS raw smoothed (~0.25 s) for Set centre / manual captures
bool   wcRawInit  = false;

// Manual calibration captures (RAM until Build): 0 left lock, 1 centre, 2 right lock
struct WcManCap {
    uint8_t have;                // bit per capture
    uint8_t tkOk;                // bit per capture: Keya ticks valid
    float   raw[3], ang[3];      // ang: wheel angle typed at the lock (magnitude)
    int32_t tk[3];
} wcMan;

static void wcSetMsg(const char *m) { strncpy(wcMsg, m, sizeof(wcMsg) - 1); wcMsg[sizeof(wcMsg) - 1] = 0; webLog(m); }

void wasCalLoad()
{
    EEPROM.get(EEP_WASCAL_ADDR, wasCal);
    bool bad = (wasCal.magic != WC_STORE_MAGIC) || wasCal.nPts > WC_NPTS
            || isnan(wasCal.zeroShift) || isnan(wasCal.azShift) || isnan(wasCal.blendDeg);
    for (uint8_t i = 0; !bad && i < wasCal.nPts; i++)
        if (isnan(wasCal.ang[i]) || isnan(wasCal.raw[i])) bad = true;
    // wheel tables were appended in v1.0.10 — an older block has wheelMask 0
    if (!bad && wasCal.wheelMask) {
        bool wbad = wasCal.wheelMask > 3;
        for (uint8_t i = 0; !wbad && i < wasCal.nPts; i++)
            if (isnan(wasCal.wR[i]) || isnan(wasCal.wL[i])) wbad = true;
        if (wbad) wasCal.wheelMask = 0;
    }
    if (!bad && wasCal.handX10 > 600) wasCal.handX10 = 0;     // v1.0.12 field, was pad
    if (bad) {
        wcStoreDefaults(wasCal);
        EEPROM.put(EEP_WASCAL_ADDR, wasCal);
        Serial.println("WAS cal: no table in EEPROM, defaults written");
    } else {
        Serial.printf("WAS cal: %u pts, table %s, Keya ext %s\n", wasCal.nPts,
                      wasCal.useTable ? "ON" : "off", wasCal.keyaExtend ? "ON" : "off");
    }
    wcCal.reset();                 // DMAMEM is not zeroed at boot
    memset(&wcRes, 0, sizeof(wcRes));
    wcZeroClear(wcZero);
    memset(&wcKeyaExt, 0, sizeof(wcKeyaExt));
    memset(&wcMan, 0, sizeof(wcMan));
}

void wasCalSave() { EEPROM.put(EEP_WASCAL_ADDR, wasCal); }

bool wasCalTableActive() { return wasCal.useTable && wasCal.nPts >= 2; }

static bool wcAutosteerOn() { return watchdogTimer < WATCHDOG_THRESHOLD; }

// The old ADS formula (AOG counts / offset / invert), auto-zero offset included.
// Ackermann is applied later for every source, as before.
float adsAogAngle(int16_t raw)
{
    float a;
    if (steerConfig.InvertWAS) a = (float)(raw - 6805 - steerSettings.wasOffset) / -steerSettings.steerSensorCounts;
    else                       a = (float)(raw - 6805 + steerSettings.wasOffset) /  steerSettings.steerSensorCounts;
    return a - moduleConfig.adsAutoOffset;
}

// ADS angle in table mode, incl. the raw-domain auto-zero and the Keya extension.
float adsTableAngle(int16_t raw)
{
    int8_t reg;
    float a = wcAdsAngle(wasCal, (float)raw, &reg);
    wcRegion = reg;

    // Slow auto-zero (same gate as the AOG mode), but it moves the raw zero
    if (moduleConfig.adsAzEnable) {
        static uint32_t stable = 0;
        if (gpsSpeed > moduleConfig.adsAzSpeedMin && fabs(headingRate) < moduleConfig.adsAzYawMax
            && fabs(a) < moduleConfig.adsAzDeltaMax && reg == 0) {
            uint32_t n = millis();
            if (stable == 0) stable = n;
            if (n - stable > moduleConfig.adsAzTimeMs)
                wasCal.azShift += moduleConfig.adsAzBeta * a * wcCountsPerDegAtZero(wasCal);
        } else stable = 0;
        static uint32_t saveT = 0; static float savedShift = 0;
        if (millis() - saveT > 300000UL) {
            saveT = millis();
            if (fabsf(wasCal.azShift - savedShift) > 2.0f) {
                wasCalSave(); savedShift = wasCal.azShift;
                webLog("WAS table auto-zero saved to EEPROM");
            }
        }
    }
    a = wcKeyaBlend(wasCal, wcKeyaExt, a, keyaDetected, keyaEncoderRaw);
    return a;
}

// Every autosteer loop (~40 Hz) when an ADS is present: recording + straight zero.
// angPreAck = the ADS angle before the Ackermann stage (what the zero must null).
void wasCalLoop(float angPreAck)
{
    static elapsedMillis dtT;
    float dt = dtT / 1000.0f;
    dtT = 0;

    if (!wcRawInit) { wcRawAvg = adsRawCounts; wcRawInit = true; }
    wcRawAvg += 0.15f * ((float)adsRawCounts - wcRawAvg);       // ~0.25 s at 40 Hz

    wcAngAog   = adsAogAngle(adsRawCounts);
    wcAngTable = (wasCal.nPts >= 2) ? wcAdsAngle(wasCal, (float)adsRawCounts, 0) : 0;

    if (wcCal.active >= 0) {
        if (gpsSpeed > 0.5f)   { wcCal.stop(); wcSetMsg("WAS cal: vehicle moving - measurement stopped"); }
        else if (wcAutosteerOn()) { wcCal.stop(); wcSetMsg("WAS cal: autosteer engaged - measurement stopped"); }
        else if (refAngleTime < 300 && refAngleValid)
            wcCal.sample(refWheelAngle, adsRawCounts, keyaDetected, keyaEncoderRaw, dt);
        else wcCal.ses[wcCal.active].nStale++;
    }

    if (wcZero.running) {
        bool tbl = wasCalTableActive();
        float yawMax = wasCal.zYawMax * (gpsMotionVtg ? 3.0f : 1.0f);
        wcZeroStep(wcZero, millis(), (float)adsRawCounts, angPreAck, gpsMotionHdg,
                   gpsSpeed, steerAngleSpeedActual, yawMax, wasCal.zSpeedMin, wasCal.zTimeMs);
        if (tbl != (wcZero.tableMode != 0)) { wcZero.running = 0; strcpy(wcZero.msg, "mode changed - start again"); }
    }
}

// ── Commands: /api/wascal?... ────────────────────────────────────────────────
// Query parameter value: the key must follow '?' or '&' exactly ("stop=" must not
// match "zstop=", "apply=" not "zapply=").
static const char *wcArg(const char *req, const char *key)
{
    size_t n = strlen(key);
    for (const char *p = strstr(req, key); p; p = strstr(p + 1, key))
        if (p > req && (p[-1] == '?' || p[-1] == '&')) return p + n;
    return NULL;
}
static bool wcHas(const char *req, const char *kv) { const char *eq = strchr(kv, '='); char key[16];
    size_t n = eq ? (size_t)(eq - kv + 1) : strlen(kv); if (n >= sizeof key) return false;
    memcpy(key, kv, n); key[n] = 0; const char *v = wcArg(req, key);
    return v && strncmp(v, kv + n, strlen(kv + n)) == 0; }

// Store a built table (IMU, manual or counts) and start its zero again.
// keepKeya: counts table — the Keya ticks per bike degree stay valid (physical).
static void wcApplyResult(const WcResult &r, uint8_t kind, bool keepKeya)
{
    wasCal.nPts = r.nPts; wasCal.twoWheel = r.twoWheel;
    memcpy(wasCal.ang, r.ang, sizeof(wasCal.ang));
    memcpy(wasCal.raw, r.raw, sizeof(wasCal.raw));
    memcpy(wasCal.wR, r.wR, sizeof(wasCal.wR));
    memcpy(wasCal.wL, r.wL, sizeof(wasCal.wL));
    wasCal.wheelMask = r.wheelMask;
    if (!keepKeya) {
        wasCal.keyaTpdL = r.keyaTpdL; wasCal.keyaTpdR = r.keyaTpdR;
        wasCal.keyaMaxL = r.keyaMaxL; wasCal.keyaMaxR = r.keyaMaxR;
        if (!r.keyaOk) wasCal.keyaExtend = 0;
    }
    wasCal.rms = r.rms; wasCal.hyst = r.hystMean;
    wasCal.flags = (wasCal.flags & ~WC_F_KIND) | (kind & WC_F_KIND);
    wasCal.zeroShift = 0; wasCal.azShift = 0;
    wcZeroClear(wcZero);
    memset(&wcKeyaExt, 0, sizeof(wcKeyaExt));
    wasCalSave();
}

// Raw that reads 0° now (table + both zero shifts)
static float wcRawZeroNow() { return wcAngleToRaw(wasCal.ang, wasCal.raw, wasCal.nPts, 0.0f) + wasCal.zeroShift + wasCal.azShift; }

void handleApiWasCal(EthernetClient& client, const char* req)
{
    const char *p;
    const char *err = NULL;
    bool stationary = gpsSpeed <= 0.5f;

    if ((p = wcArg(req, "start=")) != NULL) {
        uint8_t wheel = (*p == 'L') ? 1 : 0;
        if (!adcConnected)                              err = "no ADS1115";
        else if (!(refAngleTime < 300 && refAngleValid)) err = "no reference IMU";
        else if (!stationary)                           err = "vehicle moving";
        else if (wcAutosteerOn())                       err = "autosteer engaged";
        else {
            wcCal.start(wheel, refWheelAngle, adsRawCounts, keyaDetected, keyaEncoderRaw);
            wcResValid = false;
            wcSetMsg(wheel ? "WAS cal: LEFT wheel - turn RIGHT first, then lock to lock" : "WAS cal: RIGHT wheel - turn RIGHT first, then lock to lock");
        }
    }
    else if (wcHas(req, "stop=1"))  { wcCal.stop(); wcSetMsg("WAS cal: measurement stopped"); }
    else if (wcHas(req, "discard=1")) { wcCal.reset(); wcResValid = false; wcSetMsg("WAS cal: measurements discarded"); }
    else if (wcHas(req, "compute=1")) {
        wcCal.stop();
        wcResValid = wcCal.compute(wcRes, moduleConfig.wheelBase, moduleConfig.keyaTrackT);
        wcSetMsg(wcRes.msg);
    }
    else if (wcHas(req, "apply=1")) {
        if (!wcResValid)          err = "nothing to apply";
        else if (wcAutosteerOn()) err = "autosteer engaged";
        else {
            wcApplyResult(wcRes, WC_KIND_IMU, false);      // new table → zero again
            wcResValid = false;
            wcSetMsg("WAS cal: table applied & saved - now set the zero");
        }
    }
    else if (wcHas(req, "resettable=1")) {
        if (wcAutosteerOn()) err = "autosteer engaged";
        else {
            WasCalStore keep = wasCal;
            wcStoreDefaults(wasCal);
            wasCal.blendDeg = keep.blendDeg; wasCal.zYawMax = keep.zYawMax;
            wasCal.zSpeedMin = keep.zSpeedMin; wasCal.zTimeMs = keep.zTimeMs;
            wasCal.handX10 = keep.handX10;
            wasCal.flags = keep.flags & (WC_F_REAR | WC_F_SENS_LEFT);   // display / manual side stay
            wasCalSave();
            wcSetMsg("WAS cal: table cleared - AOG settings in use");
        }
    }
    else if ((p = wcArg(req, "use=")) != NULL) {
        if (wcAutosteerOn())                 err = "autosteer engaged";
        else if (*p == '1' && wasCal.nPts < 2) err = "no table yet";
        else { wasCal.useTable = (*p == '1'); memset(&wcKeyaExt, 0, sizeof(wcKeyaExt)); wasCalSave();
               wcSetMsg(wasCal.useTable ? "WAS: table in use" : "WAS: AOG settings in use"); }
    }
    else if ((p = wcArg(req, "kx=")) != NULL) {
        if (wcAutosteerOn())                                         err = "autosteer engaged";
        else if (*p == '1' && (wasCal.keyaTpdL == 0 || wasCal.keyaTpdR == 0)) err = "no Keya data in the table";
        else { wasCal.keyaExtend = (*p == '1'); memset(&wcKeyaExt, 0, sizeof(wcKeyaExt)); wasCalSave(); }
    }
    else if (wcHas(req, "params=1")) {
        if ((p = wcArg(req, "blend=")) != NULL) { float v = atof(p); if (v >= 1 && v <= 15) wasCal.blendDeg = v; }
        if ((p = wcArg(req, "hand="))  != NULL) { float v = atof(p); if (v >= 0 && v <= 60) wasCal.handX10 = (uint16_t)(v * 10 + 0.5f);
                                                  memset(&wcKeyaExt, 0, sizeof(wcKeyaExt)); }
        // steering limits live in ModuleConfig (same fields as before, now for every WAS source)
        bool lim = false;
        if ((p = wcArg(req, "maxl="))  != NULL) { float v = atof(p); if (v >= 0 && v <= 80) { moduleConfig.keyaMaxAngleLeft  = v; lim = true; } }
        if ((p = wcArg(req, "maxr="))  != NULL) { float v = atof(p); if (v >= 0 && v <= 80) { moduleConfig.keyaMaxAngleRight = v; lim = true; } }
        if (lim) moduleConfigSave();
        if ((p = wcArg(req, "zyaw="))  != NULL) { float v = atof(p); if (v >= 0.02f && v <= 2) wasCal.zYawMax = v; }
        if ((p = wcArg(req, "zspd="))  != NULL) { float v = atof(p); if (v >= 1 && v <= 25) wasCal.zSpeedMin = v; }
        if ((p = wcArg(req, "ztime=")) != NULL) { float v = atof(p); if (v >= 2 && v <= 60) wasCal.zTimeMs = (uint16_t)(v * 1000); }
        wasCalSave();
    }
    else if (wcHas(req, "flip=1")) {
        if (wcAutosteerOn())                                         err = "autosteer engaged";
        else if (!wcFlipStore(wasCal, moduleConfig.wheelBase, moduleConfig.keyaTrackT)) err = "no table yet";
        else { memset(&wcKeyaExt, 0, sizeof(wcKeyaExt)); wcZeroClear(wcZero); wasCalSave();
               wcSetMsg("WAS cal: table flipped (left <-> right) & saved"); }
    }
    else if (wcHas(req, "centre=1")) {
        // Set centre now: the wheels are straight, this reads 0° from now on
        if (wcAutosteerOn()) err = "autosteer engaged";
        else if (moduleConfig.wasSource == WAS_SOURCE_ADS1115) {
            if (!adcConnected || !wcRawInit) err = "no ADS1115";
            else if (wasCalTableActive()) {
                wasCal.zeroShift = wcRawAvg - wcAngleToRaw(wasCal.ang, wasCal.raw, wasCal.nPts, 0.0f);
                wasCal.azShift = 0;
                wasCalSave();
                wcSetMsg("WAS cal: centre set (table zero)");
            } else {
                moduleConfig.adsAutoOffset += adsAogAngle((int16_t)lroundf(wcRawAvg));
                moduleConfigSave();
                wcSetMsg("WAS cal: centre set (AOG mode offset)");
            }
            if (!err) { wcZeroClear(wcZero); memset(&wcKeyaExt, 0, sizeof(wcKeyaExt)); }
        }
        else if (moduleConfig.wasSource == WAS_SOURCE_KEYA) {
            if (!keyaDetected) err = "Keya not detected";
            else {
                moduleConfig.keyaZeroTicks = keyaEncoderRaw;
                keyaGpsOffset = 0;
                keyaInitialZeroDone = true;                  // a set centre unlocks autosteer like the initial zero
                moduleConfigSave();
                wcSetMsg("WAS cal: Keya centre set");
            }
        }
        else err = "no centre for this WAS source";
    }
    else if ((p = wcArg(req, "axle=")) != NULL) {
        if (*p == 'R') wasCal.flags |= WC_F_REAR; else wasCal.flags &= ~WC_F_REAR;
        wasCalSave();
    }
    else if ((p = wcArg(req, "mside=")) != NULL) {
        if (*p == 'L') wasCal.flags |= WC_F_SENS_LEFT; else wasCal.flags &= ~WC_F_SENS_LEFT;
        wasCalSave();
    }
    else if ((p = wcArg(req, "mcap=")) != NULL) {
        int i = (*p == 'L') ? 0 : (*p == 'C') ? 1 : (*p == 'R') ? 2 : -1;
        const char *a = wcArg(req, "ang=");
        float v = a ? atof(a) : 0;
        if (i < 0)                             err = "bad capture";
        else if (!adcConnected || !wcRawInit)  err = "no ADS1115";
        else if (wcAutosteerOn())              err = "autosteer engaged";
        else if (i != 1 && !(v >= 3 && v <= 70)) err = "type the wheel angle first (3..70 deg)";
        else {
            wcMan.raw[i] = wcRawAvg; wcMan.ang[i] = (i == 1) ? 0 : v;
            wcMan.tk[i] = keyaEncoderRaw;
            wcMan.have |= (1 << i);
            if (keyaDetected) wcMan.tkOk |= (1 << i); else wcMan.tkOk &= ~(1 << i);
            char m[64];
            snprintf(m, sizeof m, "WAS cal: manual %s captured, raw %.0f", i == 0 ? "left lock" : i == 1 ? "centre" : "right lock", wcRawAvg);
            wcSetMsg(m);
        }
    }
    else if (wcHas(req, "mclear=1")) { memset(&wcMan, 0, sizeof(wcMan)); wcSetMsg("WAS cal: manual captures cleared"); }
    else if (wcHas(req, "mbuild=1")) {
        if (wcAutosteerOn())          err = "autosteer engaged";
        else if (wcMan.have != 7)     err = "capture left lock, centre and right lock first";
        else {
            WcResult *r = &wcRes;                         // scratch (a pending IMU result is dropped)
            bool tk = wcMan.tkOk == 7;
            if (!wcBuildManual(*r, (wasCal.flags & WC_F_SENS_LEFT) != 0, wcMan.ang[0], wcMan.raw[0], wcMan.raw[1],
                               wcMan.ang[2], wcMan.raw[2], moduleConfig.wheelBase, moduleConfig.keyaTrackT,
                               tk, wcMan.tk[0], wcMan.tk[1], wcMan.tk[2])) { wcResValid = false; wcSetMsg(r->msg); err = r->msg; }
            else {
                wcApplyResult(*r, WC_KIND_MANUAL, false);
                wasCal.useTable = 1; wasCalSave();
                wcResValid = false;
                wcSetMsg(r->msg);
            }
        }
    }
    else if (wcHas(req, "slope=1")) {
        const char *a = wcArg(req, "cl="), *b = wcArg(req, "cr="), *c = wcArg(req, "c=");
        if (wcAutosteerOn())      err = "autosteer engaged";
        else if (!a || !b || !c)  err = "cl, cr and c needed";
        else {
            // raw direction: as the table in use, else the AOG Invert WAS
            int8_t dir = (wasCal.nPts >= 2) ? ((wasCal.raw[wasCal.nPts - 1] > wasCal.raw[0]) ? 1 : -1)
                                            : (steerConfig.InvertWAS ? -1 : 1);
            WcResult *r = &wcRes;
            if (!wcBuildSlope(*r, atof(c), atof(a), atof(b), dir, moduleConfig.wheelBase, moduleConfig.keyaTrackT)) {
                wcResValid = false; wcSetMsg(r->msg); err = r->msg;
            } else {
                wcApplyResult(*r, WC_KIND_SLOPE, true);
                wasCal.useTable = 1; wasCalSave();
                wcResValid = false;
                wcSetMsg(r->msg);
            }
        }
    }
    else if ((p = wcArg(req, "zraw=")) != NULL) {
        float v = atof(p);
        if (wcAutosteerOn())              err = "autosteer engaged";
        else if (wasCal.nPts < 2)         err = "no table yet";
        else if (!(v >= 100 && v <= 16300)) err = "centre raw must be 100..16300";
        else {
            wasCal.zeroShift = v - wcAngleToRaw(wasCal.ang, wasCal.raw, wasCal.nPts, 0.0f);
            wasCal.azShift = 0;
            wcZeroClear(wcZero); memset(&wcKeyaExt, 0, sizeof(wcKeyaExt));
            wasCalSave();
            wcSetMsg("WAS cal: centre raw set");
        }
    }
    else if (wcHas(req, "zstart=1")) {
        if (!adcConnected || moduleConfig.wasSource != WAS_SOURCE_ADS1115) err = "WAS source is not ADS1115";
        else wcZeroStart(wcZero, wasCalTableActive());
    }
    else if (wcHas(req, "zstop=1"))  { wcZero.running = 0; strcpy(wcZero.msg, "stopped"); }
    else if (wcHas(req, "zclear=1")) { wcZeroClear(wcZero); }
    else if (wcHas(req, "zapply=1")) {
        if (!wcZero.nPass)        err = "no straight pass yet";
        else if (wcAutosteerOn()) err = "autosteer engaged";
        else if (wcZero.tableMode) {
            if (!wasCalTableActive() || !wcZeroApplyTable(wasCal, wcZero)) err = "table not active";
            else { wasCalSave(); wcSetMsg("WAS cal: straight zero applied (table)"); wcZeroClear(wcZero); }
        } else {
            if (wasCalTableActive()) err = "mode changed - measure again";
            else {
                moduleConfig.adsAutoOffset += wcZeroMeanAng(wcZero);
                moduleConfigSave();
                wcSetMsg("WAS cal: straight zero applied (AOG mode offset)");
                wcZeroClear(wcZero);
            }
        }
    }

    sendHeaders(client, "text/plain");
    if (err) { client.print(F("ERR ")); client.print(err); }
    else client.print(F("OK"));
}

// Right / left wheel angle for the page. From the calibrated wheel tables while the
// table drives the angle inside its range (no Keya blend); otherwise from the
// angle sent to AOG with ideal Ackermann (L, T). Returns the calibrated-wheel mask.
static uint8_t wcWheelsNow(float &wr, float &wl)
{
    if (moduleConfig.wasSource == WAS_SOURCE_ADS1115 && wasCalTableActive() && wasCal.wheelMask
        && wcRegion == 0 && wcKeyaExt.wAds >= 0.999f) {
        float x = (float)adsRawCounts - wasCal.zeroShift - wasCal.azShift;
        wr = wcLookup(wasCal.wR, wasCal.raw, wasCal.nPts, x, 0);
        wl = wcLookup(wasCal.wL, wasCal.raw, wasCal.nPts, x, 0);
        return wasCal.wheelMask;
    }
    wcBikeToWheels(wcAngOut, moduleConfig.wheelBase, moduleConfig.keyaTrackT, wr, wl);
    return 0;
}

// ── Status: /api/wcstat[?curve=1] ────────────────────────────────────────────
static void wcPrintArr(EthernetClient& c, const float *v, int n, int dec)
{
    c.print('[');
    for (int i = 0; i < n; i++) { if (i) c.print(','); c.print(v[i], dec); }
    c.print(']');
}

void handleApiWasCalStatus(EthernetClient& client, const char* req)
{
    sendHeaders(client, "application/json");
    client.print(F("{\"src\":")); client.print(moduleConfig.wasSource);
    client.print(F(",\"ads\":")); client.print(adcConnected ? 1 : 0);
    client.print(F(",\"keya\":")); client.print(keyaDetected ? 1 : 0);
    client.print(F(",\"refOk\":")); client.print((refAngleTime < 300 && refAngleValid) ? 1 : 0);
    client.print(F(",\"ref\":")); client.print(refWheelAngle, 2);
    client.print(F(",\"raw\":")); client.print(adsRawCounts);
    client.print(F(",\"aTbl\":")); client.print(wcAngTable, 2);
    client.print(F(",\"aAog\":")); client.print(wcAngAog, 2);
    client.print(F(",\"aOut\":")); client.print(wcAngOut, 2);
    client.print(F(",\"aKeya\":")); client.print(wcKeyaExt.init ? wcKeyaExt.keyaAng : 0.0f, 2);
    client.print(F(",\"kInit\":")); client.print(wcKeyaExt.init ? 1 : 0);
    client.print(F(",\"wAds\":")); client.print(wcKeyaExt.wAds, 2);
    { float wr, wl; uint8_t wm = wcWheelsNow(wr, wl);
      client.print(F(",\"wR\":")); client.print(wr, 2);
      client.print(F(",\"wL\":")); client.print(wl, 2);
      client.print(F(",\"wCal\":")); client.print(wm); }
    client.print(F(",\"region\":")); client.print(wcRegion);
    client.print(F(",\"steerOn\":")); client.print(wcAutosteerOn() ? 1 : 0);
    client.print(F(",\"speed\":")); client.print(gpsSpeed, 1);
    client.print(F(",\"hdgRate\":")); client.print(headingRate, 2);
    client.print(F(",\"vtg\":")); client.print(gpsMotionVtg ? 1 : 0);
    client.print(F(",\"L\":")); client.print(moduleConfig.wheelBase, 2);
    client.print(F(",\"T\":")); client.print(moduleConfig.keyaTrackT, 2);
    client.print(F(",\"msg\":\"")); client.print(wcMsg); client.print('"');
    client.print(F(",\"rawAvg\":")); client.print(wcRawAvg, 1);
    client.print(F(",\"sp\":")); client.print(steerAngleSetPoint, 2);
    client.print(F(",\"aogInv\":")); client.print(steerConfig.InvertWAS ? 1 : 0);
    // Keya encoder as WAS (live graph): ticks, zero, ticks/deg base | left | right, invert, drift offset
    client.print(F(",\"kEnc\":")); client.print((long)keyaEncoderRaw);
    client.print(F(",\"kZero\":")); client.print((long)moduleConfig.keyaZeroTicks);
    client.print(F(",\"kTB\":")); client.print(moduleConfig.keyaTicksPerDeg, 2);
    client.print(F(",\"kTL\":")); client.print(moduleConfig.keyaTicksLeft, 2);
    client.print(F(",\"kTR\":")); client.print(moduleConfig.keyaTicksRight, 2);
    client.print(F(",\"kInv\":")); client.print(moduleConfig.keyaEncInvert ? 1 : 0);
    client.print(F(",\"kOff\":")); client.print(keyaGpsOffset, 2);
    client.print(F(",\"kZd\":")); client.print(keyaInitialZeroDone ? 1 : 0);

    // stored calibration
    client.print(F(",\"use\":")); client.print(wasCal.useTable);
    client.print(F(",\"kx\":")); client.print(wasCal.keyaExtend);
    client.print(F(",\"nPts\":")); client.print(wasCal.nPts);
    client.print(F(",\"two\":")); client.print(wasCal.twoWheel);
    client.print(F(",\"wMask\":")); client.print(wasCal.wheelMask);
    client.print(F(",\"kind\":")); client.print(wasCal.flags & WC_F_KIND);
    client.print(F(",\"rear\":")); client.print((wasCal.flags & WC_F_REAR) ? 1 : 0);
    client.print(F(",\"sensL\":")); client.print((wasCal.flags & WC_F_SENS_LEFT) ? 1 : 0);
    client.print(F(",\"tWR\":")); wcPrintArr(client, wasCal.wR, wasCal.wheelMask ? wasCal.nPts : 0, 2);
    client.print(F(",\"tWL\":")); wcPrintArr(client, wasCal.wL, wasCal.wheelMask ? wasCal.nPts : 0, 2);
    client.print(F(",\"raw0\":")); client.print(wasCal.nPts >= 2 ? wcRawZeroNow() : 0.0f, 1);
    client.print(F(",\"tAng\":")); wcPrintArr(client, wasCal.ang, wasCal.nPts, 2);
    client.print(F(",\"tRaw\":")); wcPrintArr(client, wasCal.raw, wasCal.nPts, 1);
    client.print(F(",\"zShift\":")); client.print(wasCal.zeroShift, 1);
    client.print(F(",\"azShift\":")); client.print(wasCal.azShift, 1);
    client.print(F(",\"adsAzOff\":")); client.print(moduleConfig.adsAutoOffset, 3);
    client.print(F(",\"adsAz\":")); client.print(moduleConfig.adsAzEnable);
    client.print(F(",\"kTpdL\":")); client.print(wasCal.keyaTpdL, 2);
    client.print(F(",\"kTpdR\":")); client.print(wasCal.keyaTpdR, 2);
    client.print(F(",\"kMaxL\":")); client.print(wasCal.keyaMaxL, 1);
    client.print(F(",\"kMaxR\":")); client.print(wasCal.keyaMaxR, 1);
    client.print(F(",\"blend\":")); client.print(wasCal.blendDeg, 1);
    client.print(F(",\"hand\":")); client.print(wasCal.handX10 / 10.0f, 1);
    client.print(F(",\"maxL\":")); client.print(moduleConfig.keyaMaxAngleLeft, 1);
    client.print(F(",\"maxR\":")); client.print(moduleConfig.keyaMaxAngleRight, 1);
    client.print(F(",\"rms\":")); client.print(wasCal.rms, 2);
    client.print(F(",\"hyst\":")); client.print(wasCal.hyst, 2);
    client.print(F(",\"zyaw\":")); client.print(wasCal.zYawMax, 2);
    client.print(F(",\"zspd\":")); client.print(wasCal.zSpeedMin, 1);
    client.print(F(",\"ztime\":")); client.print(wasCal.zTimeMs / 1000.0f, 1);

    // recording
    client.print(F(",\"act\":")); client.print(wcCal.active);
    client.print(F(",\"ses\":["));
    for (int i = 0; i < 2; i++) {
        const WcSession &s = wcCal.ses[i];
        if (i) client.print(',');
        client.print(F("{\"st\":")); client.print(s.state);
        client.print(F(",\"n\":")); client.print(s.nOk);
        client.print(F(",\"fast\":")); client.print(s.nFast);
        client.print(F(",\"stale\":")); client.print(s.nStale);
        client.print(F(",\"rate\":")); client.print(s.rate, 1);
        client.print(F(",\"sign\":")); client.print(s.wSign);
        client.print(F(",\"w\":")); client.print(s.wSign ? (refWheelAngle - s.refCenter) * s.wSign : (refWheelAngle - s.refCenter), 1);
        client.print(F(",\"ticks\":")); client.print(s.ticks ? 1 : 0);
        client.print(F(",\"cov\":["));
        for (int g = 0; g < WC_NGRID; g++) { if (g) client.print(','); client.print(s.cov[g]); }
        client.print(F("]}"));
    }
    client.print(']');

    // computed (not yet applied)
    client.print(F(",\"res\":"));
    if (!wcResValid) client.print(F("null"));
    else {
        client.print(F("{\"nPts\":")); client.print(wcRes.nPts);
        client.print(F(",\"two\":")); client.print(wcRes.twoWheel);
        client.print(F(",\"ang\":")); wcPrintArr(client, wcRes.ang, wcRes.nPts, 2);
        client.print(F(",\"raw\":")); wcPrintArr(client, wcRes.raw, wcRes.nPts, 1);
        client.print(F(",\"rms\":")); client.print(wcRes.rms, 2);
        client.print(F(",\"hyst\":")); client.print(wcRes.hystMean, 2);
        client.print(F(",\"hystMax\":")); client.print(wcRes.hystMax, 2);
        client.print(F(",\"rel\":")); client.print(wcRes.relOffset, 2);
        client.print(F(",\"inv\":")); client.print(wcRes.inversions);
        client.print(F(",\"sat\":")); client.print(wcRes.nSat);
        client.print(F(",\"kOk\":")); client.print(wcRes.keyaOk ? 1 : 0);
        client.print(F(",\"kTpdL\":")); client.print(wcRes.keyaTpdL, 2);
        client.print(F(",\"kTpdR\":")); client.print(wcRes.keyaTpdR, 2);
        client.print(F(",\"kMaxL\":")); client.print(wcRes.keyaMaxL, 1);
        client.print(F(",\"kMaxR\":")); client.print(wcRes.keyaMaxR, 1);
        if (wcHas(req, "curve=1")) {
            client.print(F(",\"cRaw\":")); wcPrintArr(client, wcRes.curveRaw, wcRes.nCurve, 0);
            client.print(F(",\"cBike\":")); wcPrintArr(client, wcRes.curveBike, wcRes.nCurve, 2);
        }
        client.print('}');
    }

    // manual captures
    client.print(F(",\"man\":{\"have\":")); client.print(wcMan.have);
    client.print(F(",\"tk\":")); client.print(wcMan.tkOk);
    client.print(F(",\"raw\":")); wcPrintArr(client, wcMan.raw, 3, 0);
    client.print(F(",\"ang\":")); wcPrintArr(client, wcMan.ang, 3, 1);
    client.print('}');

    // straight zero
    client.print(F(",\"z\":{\"run\":")); client.print(wcZero.running);
    client.print(F(",\"prog\":")); client.print(wcZero.progress, 2);
    client.print(F(",\"n\":")); client.print(wcZero.nPass);
    client.print(F(",\"tbl\":")); client.print(wcZero.tableMode);
    client.print(F(",\"ang\":")); wcPrintArr(client, wcZero.passAng, wcZero.nPass, 2);
    client.print(F(",\"sd\":")); wcPrintArr(client, wcZero.passStd, wcZero.nPass, 2);
    client.print(F(",\"mean\":")); client.print(wcZeroMeanAng(wcZero), 2);
    client.print(F(",\"msg\":\"")); client.print(wcZero.msg); client.print(F("\"}"));
    client.print('}');
}
