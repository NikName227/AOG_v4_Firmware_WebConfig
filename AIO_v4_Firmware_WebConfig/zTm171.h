// zTm171.h - TM171 (SYD Dynamics TransducerM) EasyProtocol helpers.
// Pure functions, no Arduino dependency, so testing/tm171 can run them on a PC.
//
// Frame (TransducerM User Guide V1.35, "EasyProtocol"):
//   AA 55 | length (4 + content) | payload information, 32 bit little-endian | content | CRC16 (Modbus)
//   payload information: CMD 7 bits (0..6), reserved 3 bits (0), FROM_ID 11 bits (10..20), TO_ID 11 bits (21..31)
//   CRC16 (poly 0xA001, init 0xFFFF, low byte first) over length + payload information + content.
//   The host is ID 2, a TM171 has a node ID of 100..2047, 0 is "everyone". A real TM171 answers a request sent to
//   0 but ignores a settings write sent that way (found by renatorgr/Boards on 24 Oct 2026), so commands go to the
//   ID the TM171 puts in its own frames.
//
// Objects used: 35 roll-pitch-yaw, 22 status (temperature, update rate, QoS), 21 setting, 23 calibration,
// 13 acknowledge, 12 request.
#ifndef Z_TM171_H
#define Z_TM171_H

#include <stdint.h>
#include <string.h>

#define TM171_CMD_REQUEST  12
#define TM171_CMD_ACK      13
#define TM171_CMD_SETTING  21
#define TM171_CMD_STATUS   22
#define TM171_CMD_CALIB    23
#define TM171_CMD_RPY      35

#define TM171_SET_LEN      20      // Setting object content, bytes
#define TM171_FRAME_MAX    80

inline uint16_t tm171Crc16(const uint8_t* d, uint16_t n)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (uint8_t j = 0; j < 8; j++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

inline uint16_t tm171U16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline uint32_t tm171U32(const uint8_t* p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
inline void tm171Put16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
inline void tm171Put32(uint8_t* p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }

// Frame from the host (ID 2) to toId. f needs 9 + n bytes. Returns the frame length.
inline uint8_t tm171BuildFrame(uint8_t* f, uint8_t cmd, uint16_t toId, const uint8_t* content, uint8_t n)
{
    f[0] = 0xAA; f[1] = 0x55; f[2] = (uint8_t)(4 + n);
    tm171Put32(f + 3, (uint32_t)(cmd & 0x7F) | (2UL << 10) | ((uint32_t)(toId & 0x7FF) << 21));
    if (n) memcpy(f + 7, content, n);
    tm171Put16(f + 7 + n, tm171Crc16(f + 2, (uint16_t)(5 + n)));
    return (uint8_t)(9 + n);
}

// Sender ID of a received frame (f points at AA)
inline uint16_t tm171FromId(const uint8_t* f) { return (uint16_t)((tm171U32(f + 3) >> 10) & 0x7FF); }

// Calibration object content (12 bytes): type, control, key (u16), parameter 2, parameter 3
inline void tm171CalibContent(uint8_t* c, uint8_t type, uint8_t ctrl, uint16_t key, uint32_t p2, uint32_t p3)
{
    c[0] = type; c[1] = ctrl; tm171Put16(c + 2, key); tm171Put32(c + 4, p2); tm171Put32(c + 8, p3);
}
#define TM171_CAL_GYRO_TYPE     0x05     // run-time static (forced) calibration, key 0xAA65, result in RAM
#define TM171_CAL_GYRO_KEY      0xAA65
#define TM171_CAL_GYRO_START    0x03     // continuous: p2 = accepted error x0.01 deg/s (5..500), p3 = confidence x0.01 % (0 = TM171 decides)
#define TM171_CAL_GYRO_STOP     0x04
#define TM171_CAL_GYRO_REVOKE   0x01
#define TM171_CAL_INSTALL_TYPE  0x04     // new install ("CalibB"), key 0xAA35, written to the TM171's flash
#define TM171_CAL_INSTALL_KEY   0xAA35
#define TM171_CAL_INSTALL_START 0x02
// reports (content[0] = type, content[1] = control)
#define TM171_REP_INSTALL       0x14
#define TM171_REP_GYRO          0x15

// Status object content: time stamp u32, temperature float, update rate u16, system status u16 (QoS = bits 0..2)
inline bool tm171ParseStatus(const uint8_t* c, uint8_t n, float* temp, uint16_t* rate, uint8_t* qos)
{
    if (n < 12) return false;
    memcpy(temp, c + 4, 4);
    *rate = tm171U16(c + 8);
    *qos = (uint8_t)(c[10] & 7);
    return true;
}
inline const char* tm171QosText(uint8_t q)
{
    switch (q) {
        case 0: return "booting";
        case 1: return "fault";
        case 2: return "limited";
        case 3: return "basic";
        case 4: return "fine";
        case 5: return "very good";
        default: return "?";
    }
}

// ---- Setting object (20 bytes) -------------------------------------------------------------------------------
// 0..3 switches (bit fields, below), 4..5 reserved (1152, never changed), 6..7 UART1 baud (x100 bps), 8..9 CAN baud,
// 10..11 accelerometer gain (x0.01), 12..13 magnetometer gain (x0.01), 14..15 inhibit time (ms),
// 16..19 silent time (u32 s) per the guide; a real TM171 shows its output rate in Hz at 18..19 (read on 24 Oct 2026).
// Switch bits: 0 gyro, 1 accelerometer, 2 magnetometer, 3 GyroErrFilter, 4 Self-Adapt, 5 status output, 6 raw data,
// 7 reserved 0, 8 quaternion, 9 reserved 0, 10 Euler, 11 roll-pitch-yaw, 12 gravity, 13 reserved 1, 14..16 reserved 2,
// 17 request acknowledge, 18 save to flash, 19 UART1 output, 20 reserved 0, 21 CAN output, 22..23 reserved 0,
// 24..25 boot mode (0 auto, 1 static, 2 dynamic), 26..31 unused.
#define TM171_BIT(w, n)  (((w) >> (n)) & 1UL)

// Written only when the bytes read from the TM171 look like this layout, so a firmware with another one is not hit.
inline bool tm171SetLayoutOk(const uint8_t* s, uint32_t baud)
{
    const uint32_t w = tm171U32(s);
    const uint16_t odr = tm171U16(s + 18);
    return TM171_BIT(w, 13) == 1 && ((w >> 14) & 7UL) == 2 && tm171U16(s + 6) == baud / 100 && odr >= 1 && odr <= 1000;
}

struct Tm171Form {
    bool     mag, gyroErr, selfAdapt, status, save;
    uint8_t  boot;                 // 0 auto, 1 static, 2 dynamic
    uint16_t acc, magGain;         // x0.01
    uint16_t odr;                  // Hz: 25, 50 or 100
};

// New Setting content from the one read (cur) and the form. Gyro, accelerometer, roll-pitch-yaw and UART1 output stay
// on, the UART baud and every word the firmware does not know stay as read. False with the reason.
inline bool tm171MakeSetting(const uint8_t* cur, const Tm171Form& f, uint8_t* out, const char** why)
{
    if (f.acc < 1 || f.acc > 2000 || f.magGain < 1 || f.magGain > 2000) { *why = "gain out of range (0.01 - 20.00)"; return false; }
    if (!(f.odr == 25 || f.odr == 50 || f.odr == 100))                     { *why = "output rate: 25, 50 or 100 Hz"; return false; }
    if (f.boot > 2)                                                         { *why = "boot mode out of range"; return false; }
    memcpy(out, cur, TM171_SET_LEN);
    uint32_t w = tm171U32(out);
    #define TM171_SETBIT(n, v) do { if (v) w |= (1UL << (n)); else w &= ~(1UL << (n)); } while (0)
    TM171_SETBIT(0, true); TM171_SETBIT(1, true);
    TM171_SETBIT(2, f.mag); TM171_SETBIT(3, f.gyroErr); TM171_SETBIT(4, f.selfAdapt); TM171_SETBIT(5, f.status);
    TM171_SETBIT(11, true);
    TM171_SETBIT(17, true);
    TM171_SETBIT(18, f.save);
    TM171_SETBIT(19, true);
    #undef TM171_SETBIT
    w = (w & ~(3UL << 24)) | ((uint32_t)f.boot << 24);
    tm171Put32(out, w);
    tm171Put16(out + 10, f.acc);
    tm171Put16(out + 12, f.magGain);
    tm171Put16(out + 18, f.odr);
    return true;
}

#endif
