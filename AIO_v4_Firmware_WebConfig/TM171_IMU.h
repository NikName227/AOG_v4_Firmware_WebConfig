#ifndef TM171_IMU_H
#define TM171_IMU_H

#include <Arduino.h>
#include "zTm171.h"

// Gyro / new-install calibration state (TM171 tab)
enum Tm171CalState : uint8_t { TMC_IDLE, TMC_STARTING, TMC_RUNNING, TMC_FINISHING, TMC_OK, TMC_FAILED,
                               TMC_NO_ANSWER, TMC_ABORTED, TMC_REVOKED };
#define TMC_KIND_GYRO    0
#define TMC_KIND_INSTALL 1

class TM171_IMU
{
public:

    TM171_IMU(HardwareSerial &serial, uint16_t timeout = 500);

    void setSerial(HardwareSerial *serial);   // override port before begin()
    void begin(uint32_t baud);

    void readAngles();              // reads new data
    bool isValid();                 // timeout check
    bool detect(uint16_t timeout);  // detection in setup

    int32_t getRoll();
    int32_t getPitch();
    int32_t getYaw();

    const char* getRollStr();
    const char* getPitchStr();
    const char* getYawStr();

    // ---- what the TM171 reports (status object) -------------------------------------------------------------
    float    temp      = 0;         // chip temperature, deg C
    uint16_t intRate   = 0;         // internal update rate, Hz
    uint8_t  qos       = 0;         // quality of service 0..5
    uint32_t statusMs  = 0;         // when the last status object came (0 = never)
    uint16_t nodeId    = 0;         // the TM171's own ID, taken from its frames (commands go there)
    uint16_t rpyRate   = 0;         // roll-pitch-yaw packets per second

    // ---- settings (object 21) -------------------------------------------------------------------------------
    uint8_t  set[TM171_SET_LEN];    // as read
    bool     setValid  = false;     // read, and the layout check passed
    bool     setOdd    = false;     // read, but the layout check failed (writing is refused)
    uint32_t setMs     = 0;         // when read (0 = never)
    uint32_t ackMs     = 0;         // the TM171 acknowledged our last write
    uint8_t  setNoAnswer = 0;       // requests that got no answer (TX wire?)
    uint32_t setReqMs  = 0;         // request sent, no answer yet
    uint32_t setWriteMs = 0;        // our write, to read back after it

    bool requestSettings();
    bool writeSettings(const Tm171Form& f, const char** why);

    // ---- calibration ------------------------------------------------------------------------------------------
    uint8_t  calState  = TMC_IDLE;
    uint8_t  calKind   = TMC_KIND_GYRO;
    uint32_t calMs     = 0;         // when the state was entered
    uint32_t calDoneMs = 0;         // last result (0 = none yet)
    bool calibStart(uint8_t kind, const char** why);
    void calibStop(bool aborted);   // gyro: finish now (the TM171 uses what it has); install cannot be stopped
    bool calibRevoke(const char** why);
    bool calibBusy() const { return calState == TMC_STARTING || calState == TMC_RUNNING || calState == TMC_FINISHING; }
    int  calibLeft() const;         // seconds of data collection left, -1 when not running

    char     msg[100] = "";          // last message for the page

    void loop();                    // timers; call often (main loop)
    void txHex(char* out, uint16_t size) const;   // last frame sent, "AA 55 ..." (for checking against ImuAssistant)
    uint32_t baudNow() const { return _baud; }

private:

    HardwareSerial *_serial;

    uint16_t timeoutMs;
    uint32_t lastUpdate = 0;
    uint32_t _baud = 115200;

    int32_t roll  = 0;
    int32_t pitch = 0;
    int32_t yaw   = 0;

    char rollStr[12];
    char pitchStr[12];
    char yawStr[12];

    uint8_t buffer[128];
    uint16_t bufferLen = 0;

    uint8_t  lastTx[TM171_FRAME_MAX];
    uint8_t  lastTxLen = 0;
    uint32_t rpyCount = 0, rateMs = 0, firstMs = 0;
    bool     asked = false;

    uint16_t crc16(uint8_t *data, uint16_t len);
    bool parsePacket();
    void onFrame(uint8_t cmd, const uint8_t* c, uint8_t n);
    void calibReport(uint8_t type, uint8_t ctrl);
    void send(uint8_t cmd, const uint8_t* content, uint8_t n);
    void calibCmd(uint8_t type, uint8_t ctrl, uint16_t key, uint32_t p2, uint32_t p3);
};

#endif
