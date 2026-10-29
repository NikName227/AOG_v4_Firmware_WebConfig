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
        client.print(F(",\"pinDef\":")); client.print(strcmp(otaPin, OTA_PIN_DEFAULT) == 0 ? 1 : 0);
        client.print(F(",\"max\":")); client.print((unsigned long)OTA_MAX_LEN);
        client.print(F(",\"ver\":\"" FW_VERSION "\"}"));
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
        webLog("OTA: upload started");
        sendHeaders(client, "application/json");
        client.print(F("{\"ok\":1,\"tok\":")); client.print((unsigned long)tok);
        client.print(F(",\"chunk\":1024}"));
        return;
    }

    a = qArg(req, "tok=");
    uint32_t tok = a ? (uint32_t)strtoul(a, NULL, 10) : 0;

    if (strstr(req, "cmd=cancel")) {
        if (otaSess.st != OTA_IDLE && tok == otaSess.tok) { otaSess.reset(); otaApplyPending = false; webLog("OTA: cancelled"); }
        otaJson(client, true, "");
        return;
    }

    if (strstr(req, "cmd=data")) {
        const char* blk = otaBlock();
        if (blk && otaSess.st == OTA_RECV) { otaSess.fail(blk); webLog("OTA: aborted (vehicle moving / autosteer on)"); }
        const char* o = qArg(req, "off="); const char* c = qArg(req, "c=");
        if (!o || !c) { otaJson(client, false, "off and c needed"); return; }
        uint32_t off = (uint32_t)strtoul(o, NULL, 10), crc = (uint32_t)strtoul(c, NULL, 16);
        const char* why;
        uint8_t act = (contentLen == bodyN) ? otaSess.chunk(off, (uint32_t)bodyN, tok, &why) : (why = "body incomplete", OTA_CK_ERR);
        if (act == OTA_CK_ERR) { otaJson(client, false, why); return; }
        if (otaCrc32(0, otaChunkBuf, (uint32_t)bodyN) != crc) { otaJson(client, false, "chunk CRC mismatch"); return; }
        if (act == OTA_CK_WRITE) {
            const char* e = otaFlashWrite(OTA_BUF_ADDR + off, otaChunkBuf, (uint32_t)bodyN);
            if (e) { otaSess.fail(e); otaJson(client, false, e); return; }
            otaSess.written((uint32_t)bodyN);
        }
        otaJson(client, true, "");
        return;
    }

    if (strstr(req, "cmd=end")) {
        if (otaSess.st != OTA_RECV || tok != otaSess.tok) { otaJson(client, false, "no upload running"); return; }
        if (!otaSess.complete()) { otaJson(client, false, "image not complete"); return; }
        const char* blk = otaBlock();
        if (blk) { otaSess.fail(blk); otaJson(client, false, blk); return; }
        arm_dcache_delete((void*)OTA_BUF_ADDR, otaSess.total);
        const char* e = otaImageCheck((const uint8_t*)OTA_BUF_ADDR, otaSess.total, otaSess.crc);
        if (e) { otaSess.fail(e); webLog(e); otaJson(client, false, e); return; }
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
FASTRUN static void otaApplyNow(uint32_t nSectors)
{
    for (int i = 0; i < 160; i++) NVIC_DISABLE_IRQ(i);
    __disable_irq();
    for (uint32_t s = 0; s < nSectors; s++) {
        uint8_t* dst = (uint8_t*)(OTA_FLASH_BASE + s * OTA_SECTOR);
        const uint8_t* src = (const uint8_t*)(OTA_BUF_ADDR + s * OTA_SECTOR);
        for (int tries = 0; tries < 3; tries++) {
            bool same = true;
            for (uint32_t i = 0; i < OTA_SECTOR; i += 4)
                if (*(const uint32_t*)(dst + i) != *(const uint32_t*)(src + i)) { same = false; break; }
            if (same) break;
            eepromemu_flash_erase_sector(dst);
            for (uint32_t p = 0; p < OTA_SECTOR; p += OTA_PAGE) eepromemu_flash_write(dst + p, src + p, OTA_PAGE);
        }
    }
    SCB_AIRCR = 0x05FA0004;                        // restart into the new firmware
    while (true) {}
}

void otaLoop()
{
    if (otaSess.st == OTA_RECV) {
        const char* blk = otaBlock();
        if (blk) { otaSess.fail(blk); webLog("OTA: aborted (vehicle moving / autosteer on)"); }
    }
    if (otaApplyPending && (int32_t)(millis() - otaApplyAt) >= 0) {
        otaApplyPending = false;
        const char* blk = otaBlock();
        if (blk) { otaSess.fail(blk); webLog("OTA: not applied (vehicle moving / autosteer on)"); return; }
        Serial.println("OTA: applying"); Serial.flush();
        otaApplyNow(otaSess.total / OTA_SECTOR);
    }
}
