#include "TM171_IMU.h"

TM171_IMU::TM171_IMU(HardwareSerial &serial, uint16_t timeout)
    : _serial(&serial), timeoutMs(timeout)
{
}

void TM171_IMU::setSerial(HardwareSerial *serial)
{
    if (serial) _serial = serial;
}

// The TM171 sends RPY + status + (often) raw data, about 95 bytes every 20 ms at 50 Hz. The Teensy's own receive
// buffer is 64 bytes, so a longer gap between two reads loses frames, and a lost one-shot reply (settings,
// calibration report) looks like "no reply". 1 KB of extra receive memory, and readAngles() every loop pass.
static uint8_t tm171RxMem[1024];

void TM171_IMU::begin(uint32_t baud)
{
    _baud = baud;
    _serial->addMemoryForRead(tm171RxMem, sizeof(tm171RxMem));
    _serial->begin(baud);
}

uint16_t TM171_IMU::crc16(uint8_t *data, uint16_t len)
{
    return tm171Crc16(data, len);
}

bool TM171_IMU::parsePacket()
{
    bool found = false;
    uint16_t idx = 0;

    while (idx < bufferLen - 3)
    {
        if (buffer[idx] == 0xAA && buffer[idx + 1] == 0x55)
        {
            uint8_t length = buffer[idx + 2];
            uint16_t packetSize = 2 + 1 + length + 2;

            if (idx + packetSize > bufferLen)
                break;

            uint16_t recvCRC =
                buffer[idx + packetSize - 2] |
                (buffer[idx + packetSize - 1] << 8);

            uint16_t calcCRC = crc16(&buffer[idx + 2], 1 + length);

            if (recvCRC == calcCRC)
            {
                const uint8_t cmd = buffer[idx + 3] & 0x7F;
                const uint16_t from = tm171FromId(&buffer[idx]);
                if (from) nodeId = from;

                if (cmd == TM171_CMD_RPY && length == 20)
                {
                    float r, p, y;

                    memcpy(&r, &buffer[idx + 3 + 8], 4);
                    memcpy(&p, &buffer[idx + 3 + 12], 4);
                    memcpy(&y, &buffer[idx + 3 + 16], 4);

                    roll  = (int32_t)(r * 100.0f);
                    pitch = (int32_t)(p * 100.0f);
                    yaw   = (int32_t)(y * 100.0f);

                    lastUpdate = millis();
                    rpyCount++;

                    found = true;
                }
                else if (length >= 4)
                {
                    onFrame(cmd, &buffer[idx + 7], (uint8_t)(length - 4));
                }
            }

            idx += packetSize;
        }
        else
        {
            idx++;
        }
    }

    if (idx < bufferLen)
    {
        memmove(buffer, &buffer[idx], bufferLen - idx);
        bufferLen -= idx;
    }
    else
    {
        bufferLen = 0;
    }

    return found;
}

void TM171_IMU::readAngles()
{
    while (_serial->available())
    {
        if (bufferLen >= sizeof(buffer))
        {
            parsePacket();                       // make room: whole frames out, a partial one stays
            if (bufferLen >= sizeof(buffer)) bufferLen = 0;
        }
        buffer[bufferLen++] = _serial->read();
    }

    parsePacket();
}

bool TM171_IMU::isValid()
{
    return (millis() - lastUpdate) < timeoutMs;
}

bool TM171_IMU::detect(uint16_t timeout)
{
    uint32_t start = millis();

    bufferLen = 0;

    while (millis() - start < timeout)
    {
        readAngles();

        if (isValid())
            return true;
    }

    return false;
}

int32_t TM171_IMU::getRoll()  { return roll; }
int32_t TM171_IMU::getPitch() { return pitch; }
int32_t TM171_IMU::getYaw()   { return yaw; }

const char* TM171_IMU::getRollStr()
{
    itoa(roll / 10, rollStr, 10);
    return rollStr;
}

const char* TM171_IMU::getPitchStr()
{
    itoa(pitch / 10, pitchStr, 10);
    return pitchStr;
}

const char* TM171_IMU::getYawStr()
{
    itoa(yaw / 10, yawStr, 10);
    return yawStr;
}

// ---------------------------------------------------------------------------------------------------------------
// Commands to the TM171 and its answers. Needs the Teensy TX wired to the TM171 RX.

void TM171_IMU::onFrame(uint8_t cmd, const uint8_t* c, uint8_t n)
{
    switch (cmd)
    {
    case TM171_CMD_STATUS:
        if (tm171ParseStatus(c, n, &temp, &intRate, &qos)) statusMs = millis() | 1;
        break;
    case TM171_CMD_SETTING:
        if (n == TM171_SET_LEN)
        {
            memcpy(set, c, TM171_SET_LEN);
            const bool ok = tm171SetLayoutOk(set, _baud);
            setMs = millis() | 1;
            setReqMs = 0;
            setNoAnswer = 0;
            setValid = ok;
            setOdd = !ok;
        }
        break;
    case TM171_CMD_ACK:
        if (n >= 1 && c[0] == TM171_CMD_SETTING) ackMs = millis() | 1;
        break;
    case TM171_CMD_CALIB:
        if (n >= 2) calibReport(c[0], c[1]);
        break;
    default: break;
    }
}

void TM171_IMU::send(uint8_t cmd, const uint8_t* content, uint8_t n)
{
    if (n > 32) return;
    lastTxLen = tm171BuildFrame(lastTx, cmd, nodeId, content, n);
    _serial->write(lastTx, lastTxLen);
}

void TM171_IMU::txHex(char* out, uint16_t size) const
{
    uint16_t o = 0;
    for (uint8_t i = 0; i < lastTxLen && o + 3 < size; i++) o += snprintf(out + o, size - o, i ? " %02X" : "%02X", lastTx[i]);
    out[o] = '\0';
}

bool TM171_IMU::requestSettings()
{
    const uint8_t c[4] = { TM171_CMD_SETTING, 0, 0, 0 };
    send(TM171_CMD_REQUEST, c, sizeof(c));
    setReqMs = millis() | 1;
    return true;
}

bool TM171_IMU::writeSettings(const Tm171Form& f, const char** why)
{
    if (!setValid) { *why = setOdd ? "settings layout not recognised, writing is disabled" : "settings not read yet"; return false; }
    uint8_t s[TM171_SET_LEN];
    if (!tm171MakeSetting(set, f, s, why)) return false;
    send(TM171_CMD_SETTING, s, TM171_SET_LEN);
    _serial->flush();
    ackMs = 0;
    setWriteMs = millis() | 1;
    snprintf(msg, sizeof(msg), f.save ? "settings written, saved to flash" : "settings written, this session only");
    return true;
}

void TM171_IMU::calibCmd(uint8_t type, uint8_t ctrl, uint16_t key, uint32_t p2, uint32_t p3)
{
    uint8_t c[12];
    tm171CalibContent(c, type, ctrl, key, p2, p3);
    send(TM171_CMD_CALIB, c, sizeof(c));
}

#define TM171_CALIB_S      20       // seconds of data (SYD: at least 12 s is a full calibration)
#define TM171_CALIB_LIMIT  50       // accepted gyro error x0.01 deg/s (5..500)
#define TM171_ANSWER_MS    3000
#define TM171_INSTALL_MS   60000

bool TM171_IMU::calibStart(uint8_t kind, const char** why)
{
    if (calibBusy()) { *why = "a calibration is already in progress"; return false; }
    calKind = kind;
    calMs = millis();
    if (kind == TMC_KIND_INSTALL)
    {
        calibCmd(TM171_CAL_INSTALL_TYPE, TM171_CAL_INSTALL_START, TM171_CAL_INSTALL_KEY, 0, 0);
        calState = TMC_RUNNING;               // the TM171 sends no "started": only the result
        snprintf(msg, sizeof(msg), "new install calibration started (about 20 s, do not cut the power)");
    }
    else
    {
        calibCmd(TM171_CAL_GYRO_TYPE, TM171_CAL_GYRO_START, TM171_CAL_GYRO_KEY, TM171_CALIB_LIMIT, 0);
        calState = TMC_STARTING;
        snprintf(msg, sizeof(msg), "gyro calibration started (%d s, keep the tractor still)", TM171_CALIB_S);
    }
    return true;
}

void TM171_IMU::calibStop(bool aborted)
{
    if (!calibBusy()) return;
    if (calKind == TMC_KIND_INSTALL) { snprintf(msg, sizeof(msg), "new install calibration cannot be stopped"); return; }
    calibCmd(TM171_CAL_GYRO_TYPE, TM171_CAL_GYRO_STOP, TM171_CAL_GYRO_KEY, 0, 0);
    calMs = millis();
    if (aborted) { calState = TMC_ABORTED; calDoneMs = calMs | 1; snprintf(msg, sizeof(msg), "gyro calibration stopped (tractor moving or autosteer on)"); }
    else         { calState = TMC_FINISHING; snprintf(msg, sizeof(msg), "finishing, the TM171 uses what it collected"); }
}

bool TM171_IMU::calibRevoke(const char** why)
{
    if (calibBusy()) { *why = "a calibration is in progress"; return false; }
    calibCmd(TM171_CAL_GYRO_TYPE, TM171_CAL_GYRO_REVOKE, TM171_CAL_GYRO_KEY, 0, 0);
    calKind = TMC_KIND_GYRO;
    calState = TMC_FINISHING;                 // waits for the revoke report (0x15/0x05)
    calMs = millis();
    snprintf(msg, sizeof(msg), "undoing the gyro calibration");
    return true;
}

void TM171_IMU::calibReport(uint8_t type, uint8_t ctrl)
{
    const uint32_t now = millis();
    if (type == TM171_REP_GYRO && calKind == TMC_KIND_GYRO)
    {
        if (ctrl == 0x00 && calState == TMC_STARTING) { calState = TMC_RUNNING; calMs = now; return; }
        if (!calibBusy()) return;
        if (ctrl == 0x01)      { calState = TMC_OK;      snprintf(msg, sizeof(msg), "gyro calibration done (kept until the TM171 loses power)"); }
        else if (ctrl == 0x05) { calState = TMC_REVOKED; snprintf(msg, sizeof(msg), "gyro calibration undone"); }
        else if (ctrl == 0xFF) { calState = TMC_FAILED;  snprintf(msg, sizeof(msg), "gyro calibration FAILED (tractor moved or too much vibration), previous values kept"); }
        else return;
        calMs = now; calDoneMs = now | 1;
    }
    else if (type == TM171_REP_INSTALL && calKind == TMC_KIND_INSTALL && calibBusy())
    {
        if (ctrl == 0x01)      { calState = TMC_OK;     snprintf(msg, sizeof(msg), "new install calibration saved, switch the module off and on"); }
        else if (ctrl == 0xFF) { calState = TMC_FAILED; snprintf(msg, sizeof(msg), "new install calibration FAILED, switch the module off and on and try again"); }
        else return;
        calMs = now; calDoneMs = now | 1;
    }
}

int TM171_IMU::calibLeft() const
{
    if (calState != TMC_RUNNING || calKind != TMC_KIND_GYRO) return -1;
    const uint32_t el = (millis() - calMs) / 1000UL;
    return el >= TM171_CALIB_S ? 0 : (int)(TM171_CALIB_S - el);
}

// signed compare: a timer set a millisecond after "now" must not look expired (unsigned wrap)
static bool tmElapsed(uint32_t now, uint32_t since, uint32_t ms) { return (int32_t)(now - since) > (int32_t)ms; }

void TM171_IMU::loop()
{
    const uint32_t now = millis();
    if (!firstMs) { firstMs = now | 1; rateMs = now; }
    if (!asked && tmElapsed(now, firstMs, 2000)) { asked = true; requestSettings(); }
    if (setWriteMs && tmElapsed(now, setWriteMs, 500)) { setWriteMs = 0; requestSettings(); }
    if (setReqMs && tmElapsed(now, setReqMs, TM171_ANSWER_MS))
    {
        setReqMs = 0;
        if (setNoAnswer < 100) setNoAnswer++;
        if (!setMs && setNoAnswer <= 4) requestSettings();             // a few more tries before "no reply" stays
    }
    if (now - rateMs >= 1000) { rateMs = now; rpyRate = (uint16_t)rpyCount; rpyCount = 0; }

    if (!calibBusy()) return;
    if (calKind == TMC_KIND_INSTALL)
    {
        if (tmElapsed(now, calMs, TM171_INSTALL_MS)) { calState = TMC_NO_ANSWER; calMs = now; calDoneMs = now | 1; snprintf(msg, sizeof(msg), "no result from the new install calibration (is TX wired to the TM171 RX?)"); }
        return;
    }
    switch (calState)
    {
    case TMC_STARTING:
        if (tmElapsed(now, calMs, TM171_ANSWER_MS)) { calState = TMC_NO_ANSWER; calMs = now; calDoneMs = now | 1; snprintf(msg, sizeof(msg), "no reply to the calibration command (is TX wired to the TM171 RX?)"); }
        break;
    case TMC_RUNNING:
        if (tmElapsed(now, calMs, (uint32_t)TM171_CALIB_S * 1000UL)) { calibCmd(TM171_CAL_GYRO_TYPE, TM171_CAL_GYRO_STOP, TM171_CAL_GYRO_KEY, 0, 0); calState = TMC_FINISHING; calMs = now; }
        break;
    case TMC_FINISHING:
        if (tmElapsed(now, calMs, TM171_ANSWER_MS)) { calState = TMC_NO_ANSWER; calMs = now; calDoneMs = now | 1; snprintf(msg, sizeof(msg), "no reply after the calibration, result unknown"); }
        break;
    default: break;
    }
}
