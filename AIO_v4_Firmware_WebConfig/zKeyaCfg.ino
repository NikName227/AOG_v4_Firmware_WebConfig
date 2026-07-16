// ─────────────────────────────────────────────────────────────────────────────
// Keya motor parameterization over CAN  (Max Current, Speed Kp/Ki, CAN bitrate)
//
// Protocol (from the Keya config tool):
//   config frames  → ID 0x06000591 (extended) on the Keya CAN port
//   responses      → ID 0x181 :  AA AA ?? <id> ?? <kind> ?? <value>   kind 0=Param 1=RAM 2=ROM
//   enter config   : FA FA 00 00   (motor then streams all params on 0x181)
//   write RAM param: BB BB 00 00 00 <id> 00 <value>
//   store EEPROM   : FA FA 00 08
//   exit config    : FA FA 00 AA
//
// We track only: Max Current(3), Speed Kp(7), Speed Ki(8), CAN BPS(21).
// Value unit confirmed by live read: Max Current 5 = 5 A. CAN BPS 1=125k 2=250k 3=500k 4=1M.
// ─────────────────────────────────────────────────────────────────────────────

#define KEYA_CFG_ID   0x06000591
#define KEYA_RESP_ID  0x181

// slot 0=MaxCurrent(3) 1=SpeedKp(7) 2=SpeedKi(8) 3=CanBps(21)
bool    keyaCfgMode      = false;
int16_t keyaCfgRam[4]    = { -1, -1, -1, -1 };
int16_t keyaCfgRom[4]    = { -1, -1, -1, -1 };

static int keyaCfgSlot(uint8_t id) {
    switch (id) { case 3: return 0; case 7: return 1; case 8: return 2; case 21: return 3; }
    return -1;
}

// Called from KeyaBus_Receive for 0x181 frames
void keyaCfgParse(const CAN_message_t& msg) {
    if (msg.len < 8 || msg.buf[0] != 0xAA || msg.buf[1] != 0xAA) return;
    int slot = keyaCfgSlot(msg.buf[3]);
    if (slot < 0) return;
    uint8_t kind = msg.buf[5];
    uint8_t val  = msg.buf[7];
    if      (kind == 1) keyaCfgRam[slot] = val;
    else if (kind == 2) keyaCfgRom[slot] = val;
}

static void keyaCfgSend(const uint8_t* data, uint8_t len) {
    CAN_message_t msg;
    msg.id = KEYA_CFG_ID;
    msg.flags.extended = true;
    msg.len = len;
    memset(msg.buf, 0, 8);
    memcpy(msg.buf, data, len);
    canWrite(CAN_MODE_KEYA, msg);
}

void keyaCfgEnter() {
    static const uint8_t c[] = { 0xFA, 0xFA, 0x00, 0x00 };
    keyaCfgSend(c, 4);
    keyaCfgMode = true;
    webLog("Keya: enter config (reading params)");
}
void keyaCfgExit() {
    static const uint8_t c[] = { 0xFA, 0xFA, 0x00, 0xAA };
    keyaCfgSend(c, 4);
    keyaCfgMode = false;
    webLog("Keya: exit config");
}
void keyaCfgStore() {
    static const uint8_t c[] = { 0xFA, 0xFA, 0x00, 0x08 };
    keyaCfgSend(c, 4);
    webLog("Keya: store EEPROM");
}
void keyaCfgWrite(uint8_t id, uint8_t val) {
    uint8_t c[8] = { 0xBB, 0xBB, 0x00, 0x00, 0x00, id, 0x00, val };
    keyaCfgSend(c, 8);
    webLogf("Keya: write param %u = %u", id, val);
}

// Test wheel drive (manual, timed pulse). Stationary only. Reuses SteerKeya.
// One click = drive at 'speed' for 'durMs' then auto-stop (firmware-timed, so the
// motor stops itself even if the stop command is lost).
static elapsedMillis keyaTestTimer;
static uint16_t      keyaTestDur   = 0;    // 0 = idle
static int16_t       keyaTestSpeed = 0;    // commanded speed+dir of the running test

void keyaCfgTest(int8_t dir, int16_t speed, uint16_t durMs) {
    if (gpsSpeed > 0.5f) { webLog("Keya test blocked: vehicle moving"); return; }
    if (speed < 0) speed = 0; if (speed > 250) speed = 250;
    if (durMs < 50)  durMs = 50;
    if (durMs > 5000) durMs = 5000;
    keyaTestSpeed = dir * speed;
    keyaTestTimer = 0;
    keyaTestDur   = durMs;
    SteerKeya(keyaTestSpeed, true);
}
void keyaCfgTestStop() { keyaTestDur = 0; SteerKeya(0, false); }
bool keyaTestActive()  { return keyaTestDur > 0; }

// Call from the main loop: sends a heartbeat while the test pulse is active,
// then auto-stops when the duration elapses.
void keyaCfgTestLoop() {
    if (!keyaTestDur) return;
    if (keyaTestTimer > keyaTestDur) {
        keyaTestDur = 0;
        SteerKeya(0, false);
        return;
    }
    static elapsedMillis hb;
    if (hb >= 50) { hb = 0; SteerKeya(keyaTestSpeed, true); }
}
