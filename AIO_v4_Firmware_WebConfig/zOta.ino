// ─────────────────────────────────────────────────────────────────────────────
// OTA firmware update — firmware glue for zOta.h
//
//   POST /api/ota?cmd=begin&pin=&len=&crc=   start (PIN, vehicle stopped, autosteer off) → session token
//   POST /api/ota?cmd=data&tok=&off=&c=      one binary chunk (body, CRC32 in c) → spare flash at OTA_BUF_ADDR
//   POST /api/ota?cmd=end&tok=               image check, then copy over the firmware from ITCM and restart
//   GET  /api/ota?cmd=status | cmd=cancel&tok= | cmd=pin&old=&new=
//
// The copy runs with every interrupt disabled from RAM (ITCM): nothing may read the flash while it is erased. If the
// power fails during it, the Teensy is recovered with its program button and USB (the bootloader is a separate chip).
// ─────────────────────────────────────────────────────────────────────────────

extern "C" {
    void eepromemu_flash_write(void* addr, const void* data, uint32_t len);   // teensy4 core, eeprom.c (runs from ITCM)
    void eepromemu_flash_erase_sector(void* addr);
}

#define EEP_OTA_ADDR 3400
struct OtaPinStore { uint16_t magic; char pin[OTA_PIN_MAX + 1]; };
#define OTA_PIN_MAGIC 0x07A1
static_assert(EEP_WASCAL_ADDR + sizeof(WasCalStore) <= EEP_OTA_ADDR, "WasCalStore overlaps the OTA PIN block");

static uint8_t otaChunkBuf[OTA_CHUNK_MAX];
static uint32_t otaSecBuf[OTA_SECTOR / 4];   // one sector in RAM: the flash cannot be read while it is being programmed

// Diagnostics (v1.0.23d): record of the last apply in EEPROM, checked at the next boot; reset reason and CrashReport.
#define EEP_OTA_DIAG_ADDR (EEP_OTA_ADDR + 16)
#define OTA_DIAG_MAGIC    0x07D1
struct OtaDiag { uint16_t magic; uint8_t state; uint8_t pad; uint32_t len, crc; };   // state 1 = apply started
static char otaLastTxt[96] = "none";
static char otaBootTxt[640] = "";
class OtaStrPrint : public Print {
public:
    char* b; size_t n, cap;
    OtaStrPrint(char* buf, size_t c) : b(buf), n(0), cap(c) { b[0] = 0; }
    size_t write(uint8_t c) override
    {
        if (n + 1 >= cap) return 1;
        if (c == '\r') return 1;
        b[n++] = (c == '\n' || c == '\t') ? ' ' : (c == '"' ? '\'' : (c == '\\' ? '/' : (char)c));
        b[n] = 0;
        return 1;
    }
};

// At boot: reset reason / crash report / breadcrumbs, and whether the last OTA copy really replaced the firmware.
void otaBootDiag()
{
    OtaStrPrint sp(otaBootTxt, sizeof(otaBootTxt));
    CrashReport.printTo(sp);                     // reset reason, a fault if there was one, OTA breadcrumbs
    if (!otaBootTxt[0]) strcpy(otaBootTxt, "no report");
    OtaDiag d;
    EEPROM.get(EEP_OTA_DIAG_ADDR, d);
    if (d.magic != OTA_DIAG_MAGIC || d.state != 1 || d.len < 2 * OTA_SECTOR || d.len > OTA_MAX_LEN || (d.len % OTA_SECTOR)) {
        strcpy(otaLastTxt, "none"); return;
    }
    uint32_t n = d.len / OTA_SECTOR, diff = 0;
    for (uint32_t sct = 0; sct < n; sct++)
        if (memcmp((const void*)(OTA_FLASH_BASE + sct * OTA_SECTOR), (const void*)(OTA_BUF_ADDR + sct * OTA_SECTOR), OTA_SECTOR) != 0) diff++;
    bool bufOk = otaCrc32(0, (const uint8_t*)OTA_BUF_ADDR, d.len) == d.crc;
    if (diff == 0) snprintf(otaLastTxt, sizeof(otaLastTxt), "copy complete (%lu sectors)", (unsigned long)n);
    else           snprintf(otaLastTxt, sizeof(otaLastTxt), "copy NOT done: %lu of %lu sectors differ%s", (unsigned long)diff, (unsigned long)n, bufOk ? "" : ", spare image damaged");
    webLog(otaLastTxt);
}

void otaPinLoad()
{
    OtaPinStore s;
    EEPROM.get(EEP_OTA_ADDR, s);
    s.pin[OTA_PIN_MAX] = 0;
    if (s.magic == OTA_PIN_MAGIC && otaPinValid(s.pin)) strcpy(otaPin, s.pin);
    else strcpy(otaPin, OTA_PIN_DEFAULT);
}
static void otaPinSave()
{
    OtaPinStore s; memset(&s, 0, sizeof(s));
    s.magic = OTA_PIN_MAGIC; strncpy(s.pin, otaPin, OTA_PIN_MAX);
    EEPROM.put(EEP_OTA_ADDR, s);
}

// Why an update must not run now (NULL = fine)
static const char* otaBlock()
{
    if (watchdogTimer < WATCHDOG_THRESHOLD) return "autosteer engaged";
    if (gpsSpeed > 1.0f)                    return "vehicle moving";
    return NULL;
}

// Update mode: the main loop skips GPS, IMU, CAN, UDP and the autosteer loop (see loop()). The motor is put in its safe state first.
static void otaEnter()
{
    keyaIntendToSteer = false; pwmDrive = 0; motorDrive();
    watchdogTimer = WATCHDOG_FORCE_VALUE;
    otaMode.enter(millis());
}
static void otaLeave(const char* why)
{
    if (!otaMode.on) return;
    otaMode.leave();
    webLog(why);
}

static const char* qArg(const char* req, const char* key)      // pointer to the value of "key=" in the request line
{
    const char* p = strstr(req, key);
    return p ? p + strlen(key) : NULL;
}

static const char* otaFlashWrite(uint32_t addr, const uint8_t* d, uint32_t n)    // addr and n multiples of the page size
{
    for (uint32_t o = 0; o < n; o += OTA_PAGE) {
        uint32_t a = addr + o;
        if ((a % OTA_SECTOR) == 0) eepromemu_flash_erase_sector((void*)a);
        eepromemu_flash_write((void*)a, d + o, OTA_PAGE);
        if (memcmp((const void*)a, d + o, OTA_PAGE) != 0) return "flash write verify failed";
    }
    return NULL;
}

static void otaJson(EthernetClient& c, bool ok, const char* err)
{
    sendHeaders(c, "application/json");
    c.print(F("{\"ok\":")); c.print(ok ? 1 : 0);
    if (!ok) { c.print(F(",\"err\":\"")); c.print(err); c.print('"'); }
    c.print(F(",\"st\":")); c.print(otaSess.st);
    c.print(F(",\"got\":")); c.print((unsigned long)otaSess.got);
    c.print(F(",\"total\":")); c.print((unsigned long)otaSess.total);
    c.print('}');
}

void handleApiOta(EthernetClient& client, const char* req, int contentLen)
{
    const char* a;
    const uint32_t now = millis();

    // the body (if any) is always read to the end, so an error reply does not cut the browser's send
    int bodyN = 0;
    if (contentLen > 0) {
        int want = contentLen > (int)OTA_CHUNK_MAX ? (int)OTA_CHUNK_MAX : contentLen;
        elapsedMillis t = 0;
        while (bodyN < want && client.connected() && t < 500) {
            int av = client.available();
            if (av > 0) {
                if (av > want - bodyN) av = want - bodyN;
                int r = client.read(otaChunkBuf + bodyN, av);
                if (r > 0) { bodyN += r; t = 0; }
            }
        }
    }

    if (strstr(req, "cmd=status")) {
        sendHeaders(client, "application/json");
        client.print(F("{\"st\":")); client.print(otaSess.st);
        client.print(F(",\"got\":")); client.print((unsigned long)otaSess.got);
        client.print(F(",\"total\":")); client.print((unsigned long)otaSess.total);
        client.print(F(",\"err\":\"")); client.print(otaSess.err);
        client.print(F("\",\"blk\":\"")); const char* b = otaBlock(); client.print(b ? b : "");
        client.print(F("\",\"lock\":")); client.print((unsigned long)otaGuard.lockLeftS(now));
        client.print(F(",\"mode\":")); client.print(otaMode.on ? 1 : 0);
        client.print(F(",\"pinDef\":")); client.print(strcmp(otaPin, OTA_PIN_DEFAULT) == 0 ? 1 : 0);
        client.print(F(",\"max\":")); client.print((unsigned long)OTA_MAX_LEN);
        client.print(F(",\"up\":")); client.print((unsigned long)(millis() / 1000));
        client.print(F(",\"last\":\"")); client.print(otaLastTxt);
        client.print(F("\",\"boot\":\"")); client.print(otaBootTxt);
        client.print(F("\",\"ver\":\"" FW_VERSION "\"}"));
        return;
    }

    if (strstr(req, "cmd=pin")) {
        const char* o = qArg(req, "old="); const char* n = qArg(req, "new=");
        char oldP[OTA_PIN_MAX + 2] = "", newP[OTA_PIN_MAX + 2] = "";
        if (o) { int i = 0; while (o[i] && o[i] != '&' && o[i] != ' ' && i < OTA_PIN_MAX + 1) { oldP[i] = o[i]; i++; } oldP[i] = 0; }
        if (n) { int i = 0; while (n[i] && n[i] != '&' && n[i] != ' ' && i < OTA_PIN_MAX + 1) { newP[i] = n[i]; i++; } newP[i] = 0; }
        uint8_t g = otaGuard.check(now, otaPin, oldP);
        if (g == 2) { otaJson(client, false, "locked, try again later"); return; }
        if (g == 1) { otaJson(client, false, "wrong PIN"); return; }
        if (!otaPinValid(newP)) { otaJson(client, false, "new PIN: 4 to 8 digits"); return; }
        strcpy(otaPin, newP); otaPinSave();
        webLog("OTA: PIN changed");
        otaJson(client, true, "");
        return;
    }

    if (strstr(req, "cmd=begin")) {
        const char* p = qArg(req, "pin=");
        char pin[OTA_PIN_MAX + 2] = "";
        if (p) { int i = 0; while (p[i] && p[i] != '&' && p[i] != ' ' && i < OTA_PIN_MAX + 1) { pin[i] = p[i]; i++; } pin[i] = 0; }
        uint8_t g = otaGuard.check(now, otaPin, pin);
        if (g == 2) { otaJson(client, false, "locked, try again later"); return; }
        if (g == 1) { otaJson(client, false, "wrong PIN"); return; }
        const char* blk = otaBlock();
        if (blk) { otaJson(client, false, blk); return; }
        const char* l = qArg(req, "len="); const char* c = qArg(req, "crc=");
        if (!l || !c) { otaJson(client, false, "len and crc needed"); return; }
        uint32_t tok = (micros() * 2654435761UL) ^ millis(); tok |= 1;
        const char* e = otaSess.begin((uint32_t)strtoul(l, NULL, 10), (uint32_t)strtoul(c, NULL, 16), tok);
        if (e) { otaJson(client, false, e); return; }
        otaApplyPending = false;
        otaEnter();
        webLog("OTA: upload started, GPS / IMU / CAN / steering paused");
        sendHeaders(client, "application/json");
        client.print(F("{\"ok\":1,\"tok\":")); client.print((unsigned long)tok);
        client.print(F(",\"chunk\":1024}"));
        return;
    }

    a = qArg(req, "tok=");
    uint32_t tok = a ? (uint32_t)strtoul(a, NULL, 10) : 0;

    if (strstr(req, "cmd=cancel")) {
        if (otaSess.st != OTA_IDLE && tok == otaSess.tok) { otaSess.reset(); otaApplyPending = false; otaLeave("OTA: cancelled, running again"); }
        otaJson(client, true, "");
        return;
    }

    if (strstr(req, "cmd=data")) {
        const char* blk = otaBlock();
        otaMode.touch(now);
        if (blk && otaSess.st == OTA_RECV) { otaSess.fail(blk); otaLeave("OTA: aborted (vehicle moving / autosteer on), running again"); }
        const char* o = qArg(req, "off="); const char* c = qArg(req, "c=");
        if (!o || !c) { otaJson(client, false, "off and c needed"); return; }
        uint32_t off = (uint32_t)strtoul(o, NULL, 10), crc = (uint32_t)strtoul(c, NULL, 16);
        const char* why;
        uint8_t act = (contentLen == bodyN) ? otaSess.chunk(off, (uint32_t)bodyN, tok, &why) : (why = "body incomplete", OTA_CK_ERR);
        if (act == OTA_CK_ERR) { otaJson(client, false, why); return; }
        if (otaCrc32(0, otaChunkBuf, (uint32_t)bodyN) != crc) { otaJson(client, false, "chunk CRC mismatch"); return; }
        if (act == OTA_CK_WRITE) {
            const char* e = otaFlashWrite(OTA_BUF_ADDR + off, otaChunkBuf, (uint32_t)bodyN);
            if (e) { otaSess.fail(e); otaLeave("OTA: flash write failed, running again"); otaJson(client, false, e); return; }
            otaSess.written((uint32_t)bodyN);
        }
        otaJson(client, true, "");
        return;
    }

    if (strstr(req, "cmd=end")) {
        if (otaSess.st != OTA_RECV || tok != otaSess.tok) { otaJson(client, false, "no upload running"); return; }
        if (!otaSess.complete()) { otaJson(client, false, "image not complete"); return; }
        const char* blk = otaBlock();
        if (blk) { otaSess.fail(blk); otaLeave("OTA: not applied, running again"); otaJson(client, false, blk); return; }
        arm_dcache_delete((void*)OTA_BUF_ADDR, otaSess.total);
        const char* e = otaImageCheck((const uint8_t*)OTA_BUF_ADDR, otaSess.total, otaSess.crc);
        if (e) { otaSess.fail(e); webLog(e); otaLeave("OTA: image refused, running again"); otaJson(client, false, e); return; }
        otaSess.st = OTA_READY;
        otaApplyAt = millis() + 800;                 // the reply goes out first
        otaApplyPending = true;
        webLog("OTA: image checked, applying and restarting");
        otaJson(client, true, "");
        return;
    }

    otaJson(client, false, "unknown command");
}

// Copy the image over the running firmware. Runs from ITCM with every interrupt disabled; ends with a restart.
// Breadcrumbs (CrashReport, RAM that survives a restart): #1 stage (1 entered, 2 copying, 3 restart reached),
// #2 sectors done, #3 sectors erased and written, #4 sectors in the image.
FASTRUN static void otaApplyNow(uint32_t nSectors)
{
    CrashReport.breadcrumb(1, 1); CrashReport.breadcrumb(2, 0); CrashReport.breadcrumb(3, 0); CrashReport.breadcrumb(4, nSectors);
    for (int i = 0; i < 160; i++) NVIC_DISABLE_IRQ(i);
    __disable_irq();
    CrashReport.breadcrumb(1, 2);
    uint32_t written = 0;
    for (uint32_t s = 0; s < nSectors; s++) {
        uint8_t* dst = (uint8_t*)(OTA_FLASH_BASE + s * OTA_SECTOR);
        const uint32_t* src = (const uint32_t*)(OTA_BUF_ADDR + s * OTA_SECTOR);
        for (uint32_t i = 0; i < OTA_SECTOR / 4; i++) otaSecBuf[i] = src[i];       // read the spare image before programming
        for (int tries = 0; tries < 3; tries++) {
            bool same = true;
            for (uint32_t i = 0; i < OTA_SECTOR / 4; i++)
                if (((const uint32_t*)dst)[i] != otaSecBuf[i]) { same = false; break; }
            if (same) break;
            eepromemu_flash_erase_sector(dst);
            for (uint32_t p = 0; p < OTA_SECTOR; p += OTA_PAGE) eepromemu_flash_write(dst + p, (const uint8_t*)otaSecBuf + p, OTA_PAGE);
            written++;
        }
        CrashReport.breadcrumb(2, s + 1); CrashReport.breadcrumb(3, written);
    }
    CrashReport.breadcrumb(1, 3);
    SCB_AIRCR = 0x05FA0004;                        // restart into the new firmware
    while (true) {}
}

void otaLoop()
{
    if (otaSess.st == OTA_RECV) {
        const char* blk = otaBlock();
        if (blk) { otaSess.fail(blk); otaLeave("OTA: aborted (vehicle moving / autosteer on), running again"); }
    }
    if (otaApplyPending && (int32_t)(millis() - otaApplyAt) >= 0) {
        otaApplyPending = false;
        const char* blk = otaBlock();
        if (blk) { otaSess.fail(blk); otaLeave("OTA: not applied (vehicle moving / autosteer on), running again"); return; }
        OtaDiag d = { OTA_DIAG_MAGIC, 1, 0, otaSess.total, otaSess.crc };
        EEPROM.put(EEP_OTA_DIAG_ADDR, d);           // checked at the next boot (otaBootDiag)
        Serial.println("OTA: applying"); Serial.flush();
        otaApplyNow(otaSess.total / OTA_SECTOR);
    }
}

// The main loop in update mode: the web server (the upload itself) and the update; no data for 15 s ends the mode.
void otaModeLoop()
{
    handleWebClient();
    otaLoop();
    if (otaMode.expired(millis()) && !otaApplyPending) {
        otaSess.fail("timeout: no data for 15 s");
        otaLeave("OTA: no data for 15 s, running again");
    }
}
