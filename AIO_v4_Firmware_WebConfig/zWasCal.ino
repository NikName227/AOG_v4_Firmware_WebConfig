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

static void wcSetMsg(const char *m) { strncpy(wcMsg, m, sizeof(wcMsg) - 1); wcMsg[sizeof(wcMsg) - 1] = 0; webLog(m); }

void wasCalLoad()
{
    EEPROM.get(EEP_WASCAL_ADDR, wasCal);
    bool bad = (wasCal.magic != WC_STORE_MAGIC) || wasCal.nPts > WC_NPTS
            || isnan(wasCal.zeroShift) || isnan(wasCal.azShift) || isnan(wasCal.blendDeg);
    for (uint8_t i = 0; !bad && i < wasCal.nPts; i++)
        if (isnan(wasCal.ang[i]) || isnan(wasCal.raw[i])) bad = true;
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
    return a;
}

// Every autosteer loop (~40 Hz) when an ADS is present: recording + straight zero.
// angPreAck = the ADS angle before the Ackermann stage (what the zero must null).
void wasCalLoop(float angPreAck)
{
    static elapsedMillis dtT;
    float dt = dtT / 1000.0f;
    dtT = 0;

    wcAngAog   = adsAogAngle(adsRawCounts);
    wcAngTable = (wasCal.nPts >= 2) ? wcAdsAngle(wasCal, (float)adsRawCounts, 0) : 0;

    if (wcCal.active >= 0) {
        if (gpsSpeed > 0.5f)   { wcCal.stop(); wcSetMsg("WAS cal: vehicle moving - measurement stopped"); }
        else if (wcAutosteerOn()) { wcCal.stop(); wcSetMsg("WAS cal: autosteer engaged - measurement stopped"); }
        else if (refAngleTime < 300 && refAngleValid)
            wcCal.sample(refWheelAngle, adsRawCounts, keyaDetected, keyaEncoderRaw, dt);
        else wcCal.ses[wcCal.active].nStale++;
    }

}

// ── Commands: /api/wascal?... ────────────────────────────────────────────────
static const char *wcArg(const char *req, const char *key) { const char *p = strstr(req, key); return p ? p + strlen(key) : NULL; }

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
    else if (strstr(req, "stop=1"))  { wcCal.stop(); wcSetMsg("WAS cal: measurement stopped"); }
    else if (strstr(req, "discard=1")) { wcCal.reset(); wcResValid = false; wcSetMsg("WAS cal: measurements discarded"); }
    else if (strstr(req, "compute=1")) {
        wcCal.stop();
        wcResValid = wcCal.compute(wcRes, moduleConfig.wheelBase, moduleConfig.keyaTrackT);
        wcSetMsg(wcRes.msg);
    }
    else if (strstr(req, "apply=1")) {
        if (!wcResValid)          err = "nothing to apply";
        else if (wcAutosteerOn()) err = "autosteer engaged";
        else {
            wasCal.nPts = wcRes.nPts; wasCal.twoWheel = wcRes.twoWheel;
            memcpy(wasCal.ang, wcRes.ang, sizeof(wasCal.ang));
            memcpy(wasCal.raw, wcRes.raw, sizeof(wasCal.raw));
            wasCal.keyaTpdL = wcRes.keyaTpdL; wasCal.keyaTpdR = wcRes.keyaTpdR;
            wasCal.keyaMaxL = wcRes.keyaMaxL; wasCal.keyaMaxR = wcRes.keyaMaxR;
            if (!wcRes.keyaOk) wasCal.keyaExtend = 0;
            wasCal.rms = wcRes.rms; wasCal.hyst = wcRes.hystMean;
            wasCal.zeroShift = 0; wasCal.azShift = 0;      // new table → zero again (straight driving)
            wcZeroClear(wcZero);
            memset(&wcKeyaExt, 0, sizeof(wcKeyaExt));
            wasCalSave();
            wcResValid = false;
            wcSetMsg("WAS cal: table applied & saved - now do the straight-driving zero");
        }
    }
    else if (strstr(req, "resettable=1")) {
        if (wcAutosteerOn()) err = "autosteer engaged";
        else {
            WasCalStore keep = wasCal;
            wcStoreDefaults(wasCal);
            wasCal.blendDeg = keep.blendDeg; wasCal.zYawMax = keep.zYawMax;
            wasCal.zSpeedMin = keep.zSpeedMin; wasCal.zTimeMs = keep.zTimeMs;
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

    sendHeaders(client, "text/plain");
    if (err) { client.print(F("ERR ")); client.print(err); }
    else client.print(F("OK"));
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
    client.print(F(",\"region\":")); client.print(wcRegion);
    client.print(F(",\"steerOn\":")); client.print(wcAutosteerOn() ? 1 : 0);
    client.print(F(",\"speed\":")); client.print(gpsSpeed, 1);
    client.print(F(",\"L\":")); client.print(moduleConfig.wheelBase, 2);
    client.print(F(",\"T\":")); client.print(moduleConfig.keyaTrackT, 2);
    client.print(F(",\"msg\":\"")); client.print(wcMsg); client.print('"');

    // stored calibration
    client.print(F(",\"use\":")); client.print(wasCal.useTable);
    client.print(F(",\"kx\":")); client.print(wasCal.keyaExtend);
    client.print(F(",\"nPts\":")); client.print(wasCal.nPts);
    client.print(F(",\"two\":")); client.print(wasCal.twoWheel);
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
        if (strstr(req, "curve=1")) {
            client.print(F(",\"cRaw\":")); wcPrintArr(client, wcRes.curveRaw, wcRes.nCurve, 0);
            client.print(F(",\"cBike\":")); wcPrintArr(client, wcRes.curveBike, wcRes.nCurve, 2);
        }
        client.print('}');
    }

    client.print('}');
}
