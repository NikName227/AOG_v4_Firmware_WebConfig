
// ── CAN Steer (steer-ready tractor) ───────────────────────────────────────────
// V_Bus  = port configured as CAN_MODE_VBUS (250 kbps) — steering valve TX/RX
// ISO_Bus = port configured as CAN_MODE_ISO  (250 kbps) — engage signals (Fendt), hitch
// K_Bus  = port configured as CAN_MODE_KBUS (250/500 kbps) — Fendt/FendtOne/CaseIH
//
// Brand IDs:  0=Claas        1=Valtra/Massey/McCormick/MF  2=CaseIH/NH
//             3=Fendt        4=JCB                          5=FendtOne
//             6=Lindner      7=AgOpenGPS
//             8=Cat MT Late  9=Cat MT Early
//
// Engage message bus per brand:
//   V_Bus:  0(Claas), 1(Valtra/McC/MF), 4(JCB), 6(Lindner), 8, 9(Cat)
//   ISO:    3(Fendt)
//   K_Bus:  2(CaseIH), 3(Fendt arm), 5(FendtOne)

// ── V_Bus steering curve/valve CAN IDs ────────────────────────────────────────
static const uint32_t VBUS_RX_ID[10] = {
    0x0CAC1E13,  // 0 Claas
    0x0CAC1C13,  // 1 Valtra/Massey/McCormick/MF
    0x0CACAA08,  // 2 CaseIH/New Holland
    0x0CEF2CF0,  // 3 Fendt
    0x0CACAB13,  // 4 JCB
    0x0CEF2CF0,  // 5 FendtOne (same as Fendt)
    0x0CACF013,  // 6 Lindner
    0x0CAC1C13,  // 7 AgOpenGPS
    0x18EF1CF0,  // 8 Cat MT Late
    0x0CEFFF76,  // 9 Cat MT Early
};
static const uint32_t VBUS_TX_ID[10] = {
    0x0CAD131E,  // 0 Claas
    0x0CAD131C,  // 1 Valtra/Massey/McCormick/MF
    0x0CAD08AA,  // 2 CaseIH/New Holland
    0x0CEFF02C,  // 3 Fendt
    0x0CAD13AB,  // 4 JCB
    0x0CEFF02C,  // 5 FendtOne
    0x0CAD13F0,  // 6 Lindner
    0x0CAD131C,  // 7 AgOpenGPS
    0x1CEFF01C,  // 8 Cat MT Late
    0x0CEF762C,  // 9 Cat MT Early
};

// ── ISO_Bus engage message IDs (only Fendt uses ISO for engage) ───────────────
static const uint32_t ISO_ENGAGE_ID[10] = {
    0,            // 0 Claas         — V_Bus
    0,            // 1 Valtra/McC/MF — V_Bus
    0,            // 2 CaseIH        — K_Bus
    0x18EF2CF0,   // 3 Fendt         — ISO
    0,            // 4 JCB           — V_Bus
    0,            // 5 FendtOne      — K_Bus
    0,            // 6 Lindner       — V_Bus
    0,            // 7 AgOpenGPS     — none
    0,            // 8 Cat MT Late   — V_Bus
    0,            // 9 Cat MT Early  — V_Bus
};

// ── J1939 address claim (steer-ready tractors expect the module to announce) ──
// Source address per brand; NAME broadcast on V_Bus + ISO_Bus at startup.
static const uint8_t BRAND_SRC_ADDR[10] = {
    0x1E,  // 0 Claas
    0x1C,  // 1 Valtra/Massey/McCormick/MF
    0xAA,  // 2 CaseIH/New Holland
    0x2C,  // 3 Fendt
    0xAB,  // 4 JCB
    0x2C,  // 5 FendtOne
    0xF0,  // 6 Lindner
    0x1C,  // 7 AgOpenGPS
    0x1C,  // 8 Cat MT Late
    0x2C,  // 9 Cat MT Early
};

void sendAddressClaim()
{
    if (moduleConfig.steerBrand > 9) return;
    if (!hasFuncMode(CAN_MODE_VBUS) && !hasFuncMode(CAN_MODE_ISO)) return;  // steer-ready only

    CAN_message_t msg;
    msg.id = 0x18EEFF00UL | BRAND_SRC_ADDR[moduleConfig.steerBrand];
    msg.flags.extended = true;
    msg.len = 8;
    static const uint8_t name[8] = { 0x00, 0x00, 0xC0, 0x0C, 0x00, 0x17, 0x02, 0x20 };
    memcpy(msg.buf, name, 8);

    if (hasFuncMode(CAN_MODE_VBUS)) canWrite(CAN_MODE_VBUS, msg);
    if (hasFuncMode(CAN_MODE_ISO))  canWrite(CAN_MODE_ISO,  msg);
    Serial.printf("J1939 address claim sent: SA=0x%02X\n", BRAND_SRC_ADDR[moduleConfig.steerBrand]);
}

// ── PVED tool runtime state ────────────────────────────────────────────────────
bool     pvedValveDetected = false;
uint16_t pvedLastRead64007 = 0xFFFF;  // most recent value received from valve

// ── CAN scan: collect unique IDs across all active ports ──────────────────────
#define CAN_SCAN_MAX 80
struct CanScanEntry { uint32_t id; char bus; uint16_t count; };
CanScanEntry canScanBuf[CAN_SCAN_MAX];
uint8_t      canScanCount = 0;
bool         canScanActive = false;

static void canScanRecord(uint32_t id, char bus) {
    if (!canScanActive) return;
    for (uint8_t i = 0; i < canScanCount; i++) {
        if (canScanBuf[i].id == id && canScanBuf[i].bus == bus) {
            if (canScanBuf[i].count < 9999) canScanBuf[i].count++;
            return;
        }
    }
    if (canScanCount < CAN_SCAN_MAX)
        canScanBuf[canScanCount++] = { id, bus, 1 };
}

// ── CAN plot ring buffer write ─────────────────────────────────────────────────
extern char     canRawBuf[];
extern uint16_t canRawWrite;
extern bool     canRawWrapped;

static void canRawWrite_str(const char* s) {
    while (*s) {
        canRawBuf[canRawWrite] = *s++;
        canRawWrite = (canRawWrite + 1) % CAN_RAW_BUF_SIZE;
        if (canRawWrite == 0) canRawWrapped = true;
    }
}

void canRawPrint(const CAN_message_t& msg, const char* bus) {
    canScanRecord(msg.id, bus[0]);
    if (!showCANData) return;
    char line[80];
    snprintf(line, sizeof(line),
        "[%s] %08lX L%d %02X%02X%02X%02X%02X%02X%02X%02X\n",
        bus, (unsigned long)msg.id, msg.len,
        msg.buf[0], msg.buf[1], msg.buf[2], msg.buf[3],
        msg.buf[4], msg.buf[5], msg.buf[6], msg.buf[7]);
    canRawWrite_str(line);
}

// ── Helper: fire tractor CAN engage (toggle steerSwitch) ──────────────────────
static void handleCanEngage() {
    uint32_t now = millis();
    if (now - canEngageTime < 1000) return;  // debounce 1 s
    canEngageTime = now;
    engageCAN = true;
}

// ── V_Bus setup: log brand info (bus already started by CAN_Setup) ────────────
void VBus_Setup() {
    Serial.print("CAN Steer: brand="); Serial.println(moduleConfig.steerBrand);
    webLogf("CAN Steer: brand %u  RX:0x%08lX TX:0x%08lX",
        moduleConfig.steerBrand,
        (unsigned long)VBUS_RX_ID[moduleConfig.steerBrand],
        (unsigned long)VBUS_TX_ID[moduleConfig.steerBrand]);
}

// ── V_Bus receive: wheel angle + valve state + engage (most brands) ────────────
void VBus_Receive() {
    CAN_message_t msg;
    uint8_t brand = moduleConfig.steerBrand;
    while (canRead(CAN_MODE_VBUS, msg)) {
        canRawPrint(msg, "V");
        switch (brand) {

        case 0:  // Claas
            if (msg.id == 0x0CAC1E13) {
                estCurve = (uint16_t)(msg.buf[0] | ((uint16_t)msg.buf[1] << 8));
                steeringValveReady = msg.buf[2];
            } else if (msg.id == 0x18EF1CD2) {
                if ((msg.buf[0] == 39 && msg.buf[2] == 241)     // MR models
                 || (msg.buf[0] == 39 && msg.buf[2] == 125)     // Non-MR
                 || ((msg.buf[0] & 0x04) && msg.buf[1] == 0 && msg.buf[2] == 0))  // Stage5
                    handleCanEngage();
            }
            break;

        case 1:  // Valtra / Massey / McCormick / MF — same V_Bus, different engage IDs
            if (msg.id == 0x0CAC1C13) {
                estCurve = (uint16_t)(msg.buf[0] | ((uint16_t)msg.buf[1] << 8));
                steeringValveReady = msg.buf[2];
            } else if (msg.id == 0x18EF1C32) {  // Valtra
                if (msg.buf[0] == 15 && msg.buf[1] == 96 && msg.buf[2] == 1) handleCanEngage();
            } else if (msg.id == 0x18EF1CFC) {  // McCormick
                if (msg.buf[0] == 15 && msg.buf[1] == 96 && msg.buf[3] == 255) handleCanEngage();
            } else if (msg.id == 0x18EF1C00) {  // MF
                if (msg.buf[0] == 15 && msg.buf[1] == 96 && msg.buf[2] == 1) handleCanEngage();
            } else if (msg.id == 0x18FF8306) {  // McCormick joystick
                if (bitRead(msg.buf[5], 3)) handleCanEngage();
            }
            break;

        case 2:  // CaseIH/New Holland — engage via K_Bus
            if (msg.id == 0x0CACAA08) {
                estCurve = (uint16_t)(msg.buf[0] | ((uint16_t)msg.buf[1] << 8));
                steeringValveReady = msg.buf[2];
            }
            break;

        case 3:  case 5:  // Fendt / FendtOne
            if (msg.buf[0] == 5 && msg.buf[1] == 10) {
                FendtEstCurve = (int16_t)(((uint16_t)msg.buf[5] << 8) | msg.buf[4]);
                estCurve = (uint16_t)(32128 + FendtEstCurve);
                steeringValveReady = 16;
            }
            break;

        case 4:  // JCB
            if (msg.id == 0x0CACAB13) {
                estCurve = (uint16_t)(msg.buf[0] | ((uint16_t)msg.buf[1] << 8));
                steeringValveReady = msg.buf[2];
            } else if (msg.id == 0x18EFAB27) {
                if (msg.buf[0] == 15 && msg.buf[1] == 96 && msg.buf[2] == 1) handleCanEngage();
            }
            break;

        case 6:  // Lindner
            if (msg.id == 0x0CACF013) {
                estCurve = (uint16_t)(msg.buf[0] | ((uint16_t)msg.buf[1] << 8));
                steeringValveReady = msg.buf[2];
            } else if (msg.id == 0x0CEFF021) {
                handleCanEngage();
            }
            break;

        case 7:  // AgOpenGPS remote module
            if (msg.id == 0x0CAC1C13) {
                estCurve = (uint16_t)(msg.buf[0] | ((uint16_t)msg.buf[1] << 8));
                steeringValveReady = msg.buf[2];
            }
            break;

        case 8:  // Cat MT Late — all messages on one ID
            if (msg.id == 0x18EF1CF0) {
                if (msg.buf[0] == 0xF0 && msg.buf[1] == 0x20) {
                    estCurve = (uint16_t)((msg.buf[2] << 8) | msg.buf[3]);
                    steeringValveReady = (msg.buf[4] == 5) ? 16 : 80;
                } else if (msg.buf[0] == 0x0F && msg.buf[1] == 0x60 && msg.buf[2] == 0x01) {
                    handleCanEngage();
                }
            }
            break;

        case 9:  // Cat MT Early — all messages on one ID
            if (msg.id == 0x0CEFFF76) {
                if (msg.buf[0] == 0xF0 && msg.buf[1] == 0x20) {
                    estCurve = (uint16_t)((msg.buf[2] << 8) | msg.buf[3]);
                    steeringValveReady = (msg.buf[4] == 5) ? 16 : 80;
                } else if (msg.buf[0] == 0x0F && msg.buf[1] == 0x60 && msg.buf[2] == 0x01) {
                    handleCanEngage();
                }
            }
            break;
        }
    }
}

// ── V_Bus send: steering command ───────────────────────────────────────────────
void VBus_Send() {
    uint8_t brand = moduleConfig.steerBrand;
    CAN_message_t msg;
    msg.id = VBUS_TX_ID[brand];
    msg.flags.extended = true;

    if (brand == 3 || brand == 5) {
        // Fendt: [0]=5 [1]=9 [2]=3(steer)/2(idle) [3]=10 [4-5]=FendtSetCurve LE
        msg.len = 6;
        FendtSetCurve = (int16_t)(setCurve - 32128);
        msg.buf[0] = 5;
        msg.buf[1] = 9;
        msg.buf[2] = canSteerIntend ? 3 : 2;
        msg.buf[3] = 10;
        msg.buf[4] = (uint8_t)(FendtSetCurve & 0xFF);
        msg.buf[5] = (uint8_t)((FendtSetCurve >> 8) & 0xFF);
    } else if (brand == 8 || brand == 9) {
        // Cat MT: [0]=0xF0 [1]=0x1F [2]=hi(curve) [3]=lo(curve) [4]=253/252 [5-7]=0xFF
        msg.len = 8;
        msg.buf[0] = 0xF0;
        msg.buf[1] = 0x1F;
        msg.buf[2] = (uint8_t)((setCurve >> 8) & 0xFF);
        msg.buf[3] = (uint8_t)(setCurve & 0xFF);
        msg.buf[4] = canSteerIntend ? 0xFD : 0xFC;
        msg.buf[5] = msg.buf[6] = msg.buf[7] = 0xFF;
    } else {
        // PVED / standard: [0-1]=setCurve LE [2]=0xFD(steer)/0xFC(idle) [3-7]=fill
        msg.len = 8;
        msg.buf[0] = (uint8_t)(setCurve & 0xFF);
        msg.buf[1] = (uint8_t)((setCurve >> 8) & 0xFF);
        msg.buf[2] = canSteerIntend ? 0xFD : 0xFC;
        uint8_t fill = (brand == 0) ? 0x00 : 0xFF;  // Claas uses 0x00, others 0xFF
        msg.buf[3] = msg.buf[4] = msg.buf[5] = msg.buf[6] = msg.buf[7] = fill;
    }
    canWrite(CAN_MODE_VBUS, msg);
}

// ── ISO_Bus receive: Fendt engage + rear hitch ─────────────────────────────────
void ISO_Receive() {
    CAN_message_t msg;
    uint8_t brand = moduleConfig.steerBrand;
    uint32_t engID = ISO_ENGAGE_ID[brand];

    while (canRead(CAN_MODE_ISO, msg)) {
        canRawPrint(msg, "I");

        // Rear hitch (PGN 65093 = 0xFEBB): source address varies by brand
        if ((msg.id & 0x00FFFF00) == 0x00FEBB00)
            ISORearHitch = msg.buf[1];

        // PVED param read response: Danfoss proprietary [0x1C 0xFA param_lo param_hi value_lo value_hi ...]
        if (msg.buf[0] == 0x1C && msg.buf[1] == 0xFA &&
            msg.buf[2] == (uint8_t)(64007 & 0xFF) && msg.buf[3] == (uint8_t)((64007 >> 8) & 0xFF)) {
            pvedValveDetected = true;
            pvedLastRead64007 = (uint16_t)msg.buf[4] | ((uint16_t)msg.buf[5] << 8);
            // Save factory value once — only if unknown and not already our AIO address
            if (moduleConfig.pvedParam64007Factory == 0xFFFF && pvedLastRead64007 != 0x1E) {
                moduleConfig.pvedParam64007Factory = pvedLastRead64007;
                moduleConfigSave();
                webLogf("PVED: factory value saved: 0x%02X", pvedLastRead64007);
            }
        }

        // Only Fendt (brand 3) uses ISO for engage
        if (engID == 0 || msg.id != engID) continue;

        if (brand == 3) {  // Fendt
            if (msg.buf[0] == 0x0F && msg.buf[1] == 0x60 && msg.buf[2] == 0x01)
                handleCanEngage();
        }
    }
}

// ── K_Bus receive: Fendt/FendtOne/CaseIH engage ───────────────────────────────
void KBus_Receive() {
    CAN_message_t msg;
    uint8_t brand = moduleConfig.steerBrand;
    while (canRead(CAN_MODE_KBUS, msg)) {
        canRawPrint(msg, "K");

        // FendtOne engage: ID 0x0CFFD899 [3]=0xF6
        if (msg.id == 0x0CFFD899 && msg.buf[3] == 0xF6)
            handleCanEngage();

        // Fendt SCR engage (backup via K_Bus)
        if (msg.id == 0x18EF2CF0
            && msg.buf[0] == 0x0F && msg.buf[1] == 0x60 && msg.buf[2] == 0x01)
            handleCanEngage();

        // CaseIH engage via K_Bus
        if (brand == 2) {
            if (msg.id == 0x14FF7706) {
                if ((msg.buf[0] == 130 && msg.buf[1] == 1) ||
                    (msg.buf[0] == 178 && msg.buf[1] == 4))
                    handleCanEngage();
            }
            // CaseIH rear hitch from K_Bus
            if (msg.id == 0x18FE4523)
                ISORearHitch = msg.buf[0];
        }
    }
}

// ── PVED tool: read parameter (sends J1939 request via ISO port) ───────────────
void pvedReadParam(uint16_t param) {
    CAN_message_t msg;
    msg.id = 0x18EAFFFE;
    msg.flags.extended = true;
    msg.len = 3;
    msg.buf[0] = (uint8_t)(param & 0xFF);
    msg.buf[1] = (uint8_t)((param >> 8) & 0xFF);
    msg.buf[2] = 0xFF;
    canWrite(CAN_MODE_ISO, msg);
    webLogf("PVED read req: param %u", param);
}

// ── PVED tool: write any parameter value ───────────────────────────────────────
static void pvedWriteParamValue(uint16_t param, uint16_t value) {
    CAN_message_t msg;
    msg.id = 0x1CEFFF1E;
    msg.flags.extended = true;
    msg.len = 8;
    msg.buf[0] = 0x1C; msg.buf[1] = 0xFA;
    msg.buf[2] = (uint8_t)(param & 0xFF); msg.buf[3] = (uint8_t)((param >> 8) & 0xFF);
    msg.buf[4] = (uint8_t)(value & 0xFF); msg.buf[5] = (uint8_t)((value >> 8) & 0xFF);
    msg.buf[6] = 0x00; msg.buf[7] = 0x00;
    canWrite(CAN_MODE_ISO, msg);
}

// ── PVED tool: write parameter 64007 (setpoint controller addr = 0x1E) ────────
void pvedWriteParam() {
    pvedWriteParamValue(64007, 0x1E);
    webLog("PVED write: param 64007 = 0x1E");
}

// ── PVED tool: restore param 64007 to saved factory value ─────────────────────
void pvedRestoreParam64007() {
    if (moduleConfig.pvedParam64007Factory == 0xFFFF) {
        webLog("PVED restore: no factory value saved — read param 64007 first");
        return;
    }
    pvedWriteParamValue(64007, moduleConfig.pvedParam64007Factory);
    webLogf("PVED restore: param 64007 = 0x%02X", moduleConfig.pvedParam64007Factory);
    pvedCommit();
}

// ── PVED tool: commit changes ──────────────────────────────────────────────────
void pvedCommit() {
    CAN_message_t msg;
    msg.id = 0x1CEFFF1E;
    msg.flags.extended = true;
    msg.len = 8;
    msg.buf[0] = 0x18; msg.buf[1] = 0xFA;
    msg.buf[2] = 0x00; msg.buf[3] = 0x00;
    msg.buf[4] = 0x00; msg.buf[5] = 0x00; msg.buf[6] = 0x00; msg.buf[7] = 0x00;
    canWrite(CAN_MODE_ISO, msg);
    webLog("PVED commit sent");
}

// ── PVED read all standard parameters ─────────────────────────────────────────
void pvedReadAll() {
    static const uint16_t params[] = {
        508, 706, 707, 729, 737, 738, 747, 748, 758,
        1027, 5027, 64007, 64022, 64023,
        65080, 65083, 65086, 65099, 65100, 65101, 65104, 65112
    };
    for (uint8_t i = 0; i < sizeof(params)/sizeof(params[0]); i++)
        pvedReadParam(params[i]);
    webLog("PVED read all: requests sent — responses in CAN plot");
}
