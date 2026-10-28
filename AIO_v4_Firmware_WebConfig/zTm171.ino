// zTm171.ino - TM171 tab: status, settings, calibration (web API) and the timers (loop).
// Protocol and state live in TM171_IMU.cpp / zTm171.h. Needs the Teensy TX wired to the TM171 RX.
// The TM171 is only commanded from the page: nothing is written to it on its own.

// "?key=" or "&key=" in the request line -> pointer to the value, else NULL
static const char* tmArg(const char* req, const char* key)
{
    const size_t n = strlen(key);
    for (const char* p = strstr(req, key); p; p = strstr(p + 1, key)) {
        if (p > req && (p[-1] == '?' || p[-1] == '&') && p[n] == '=') return p + n + 1;
    }
    return NULL;
}

static const char* tmCalName(uint8_t s)
{
    switch (s) {
        case TMC_STARTING:  return "starting";
        case TMC_RUNNING:   return "running";
        case TMC_FINISHING: return "finishing";
        case TMC_OK:        return "ok";
        case TMC_FAILED:    return "failed";
        case TMC_NO_ANSWER: return "noanswer";
        case TMC_ABORTED:   return "aborted";
        case TMC_REVOKED:   return "revoked";
        default:            return "idle";
    }
}

// Main loop (only with a TM171): timers, and the safety stop of a running gyro calibration.
void tm171Loop()
{
    static bool logSet = false;
    if (!logSet) { logSet = true; TM171_IMU.logFn = webLog; }     // TM171 events (commands, reports, state changes) into the web log
    TM171_IMU.readAngles();                  // every pass: the receive buffer is small and replies are one-shot
    if (TM171_IMU.calibBusy() && TM171_IMU.calKind == TMC_KIND_GYRO &&
        (watchdogTimer < WATCHDOG_THRESHOLD || gpsSpeed > 1.0f))
        TM171_IMU.calibStop(true);
    TM171_IMU.loop();
}

void handleApiTm171(EthernetClient& client, const char* req)
{
    const char* err = NULL;
    const char* p;
    const bool steer  = watchdogTimer < WATCHDOG_THRESHOLD;
    const bool moving = gpsSpeed > 1.0f;

    if (!useTMxx_IMU) err = "no TM171";
    else if (tmArg(req, "read")) TM171_IMU.requestSettings();
    else if (tmArg(req, "write")) {
        if (steer) err = "autosteer engaged";
        else {
            Tm171Form f;
            f.mag       = (p = tmArg(req, "m"))   && *p == '1';
            f.gyroErr   = (p = tmArg(req, "ge"))  && *p == '1';
            f.selfAdapt = (p = tmArg(req, "sa"))  && *p == '1';
            f.status    = (p = tmArg(req, "st"))  && *p == '1';
            f.save      = (p = tmArg(req, "sv"))  && *p == '1';
            f.raw       = (p = tmArg(req, "rw"))  && *p == '1';
            f.boot      = (p = tmArg(req, "bo"))  ? (uint8_t)atoi(p)  : 0;
            f.acc       = (p = tmArg(req, "acc")) ? (uint16_t)atoi(p) : 0;
            f.magGain   = (p = tmArg(req, "mg"))  ? (uint16_t)atoi(p) : 0;
            f.odr       = (p = tmArg(req, "odr")) ? (uint16_t)atoi(p) : 0;
            const char* why = "";
            if (!TM171_IMU.writeSettings(f, &why)) err = why;
        }
    }
    else if ((p = tmArg(req, "cal")) != NULL) {
        const char* why = "";
        if (*p == 's') TM171_IMU.calibStop(false);
        else if (steer)  err = "autosteer engaged";
        else if (moving) err = "tractor moving";
        else if (*p == 'g') { if (!TM171_IMU.calibStart(TMC_KIND_GYRO, &why)) err = why; }
        else if (*p == 'i') { if (!TM171_IMU.calibStart(TMC_KIND_INSTALL, &why)) err = why; }
        else if (*p == 'u') { if (!TM171_IMU.calibRevoke(&why)) err = why; }
        else err = "unknown calibration";
    }

    const uint32_t now = millis();
    sendHeaders(client, "application/json");
    client.print(F("{\"on\":")); client.print(useTMxx_IMU ? 1 : 0);
    client.print(F(",\"err\":\"")); client.print(err ? err : ""); client.print('"');
    client.print(F(",\"block\":\"")); client.print(steer ? "autosteer engaged" : (moving ? "tractor moving" : "")); client.print('"');
    client.print(F(",\"id\":")); client.print(TM171_IMU.nodeId);
    client.print(F(",\"baud\":")); client.print(TM171_IMU.baudNow());
    client.print(F(",\"rpy\":")); client.print(TM171_IMU.rpyRate);
    client.print(F(",\"temp\":")); client.print(TM171_IMU.temp, 1);
    client.print(F(",\"qos\":")); client.print(TM171_IMU.qos);
    client.print(F(",\"qosTxt\":\"")); client.print(tm171QosText(TM171_IMU.qos)); client.print('"');
    client.print(F(",\"intRate\":")); client.print(TM171_IMU.intRate);
    client.print(F(",\"stAge\":")); client.print(TM171_IMU.statusMs ? (long)((now - TM171_IMU.statusMs) / 1000) : -1L);
    client.print(F(",\"hdg\":\"")); client.print(TM171_IMU.getYawStr()); client.print('"');
    client.print(F(",\"roll\":\"")); client.print(TM171_IMU.getRollStr()); client.print('"');
    client.print(F(",\"pitch\":\"")); client.print(TM171_IMU.getPitchStr()); client.print('"');
    client.print(F(",\"msg\":\"")); client.print(TM171_IMU.msg); client.print('"');
    client.print(F(",\"set\":{\"ok\":")); client.print(TM171_IMU.setValid ? 1 : 0);
    client.print(F(",\"odd\":")); client.print(TM171_IMU.setOdd ? 1 : 0);
    client.print(F(",\"age\":")); client.print(TM171_IMU.setMs ? (long)((now - TM171_IMU.setMs) / 1000) : -1L);
    client.print(F(",\"pending\":")); client.print((TM171_IMU.setReqMs || TM171_IMU.setWriteMs) ? 1 : 0);
    client.print(F(",\"noAnswer\":")); client.print(TM171_IMU.setNoAnswer);
    client.print(F(",\"ack\":")); client.print(TM171_IMU.ackMs ? (long)((now - TM171_IMU.ackMs) / 1000) : -1L);
    if (TM171_IMU.setMs) {
        const uint8_t* s = TM171_IMU.set;
        const uint32_t w = tm171U32(s);
        client.print(F(",\"m\":")); client.print((int)TM171_BIT(w, 2));
        client.print(F(",\"ge\":")); client.print((int)TM171_BIT(w, 3));
        client.print(F(",\"sa\":")); client.print((int)TM171_BIT(w, 4));
        client.print(F(",\"st\":")); client.print((int)TM171_BIT(w, 5));
        client.print(F(",\"rw\":")); client.print((int)TM171_BIT(w, 6));
        client.print(F(",\"bo\":")); client.print((int)((w >> 24) & 3));
        client.print(F(",\"acc\":")); client.print(tm171U16(s + 10));
        client.print(F(",\"mg\":")); client.print(tm171U16(s + 12));
        client.print(F(",\"odr\":")); client.print(tm171U16(s + 18));
        client.print(F(",\"inh\":")); client.print(tm171U16(s + 14));
        client.print(F(",\"raw\":\""));
        for (uint8_t i = 0; i < TM171_SET_LEN; i++) { if (i) client.print(' '); if (s[i] < 16) client.print('0'); client.print(s[i], HEX); }
        client.print('"');
    }
    {   // diagnostics: last frame sent, last calibration report received
        char hx[3 * TM171_FRAME_MAX];
        TM171_IMU.txHex(hx, sizeof(hx));
        client.print(F("},\"dg\":{\"tx\":\"")); client.print(hx);
        client.print(F("\",\"cr\":\"")); client.print(TM171_IMU.calRxHex);
        client.print(F("\",\"crN\":")); client.print(TM171_IMU.calRxN);
        client.print(F(",\"crAge\":")); client.print(TM171_IMU.calRxMs ? (long)((now - TM171_IMU.calRxMs) / 1000) : -1L);
    }
    client.print(F("},\"cal\":{\"state\":\"")); client.print(tmCalName(TM171_IMU.calState));
    client.print(F("\",\"kind\":")); client.print(TM171_IMU.calKind);
    client.print(F(",\"left\":")); client.print(TM171_IMU.calibLeft());
    client.print(F(",\"busy\":")); client.print(TM171_IMU.calibBusy() ? 1 : 0);
    client.print(F(",\"done\":")); client.print(TM171_IMU.calDoneMs ? (long)((now - TM171_IMU.calDoneMs) / 1000) : -1L);
    client.print(F("}}"));
}
