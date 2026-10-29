// zOta.h - OTA firmware update: pure logic (no Arduino dependency, so testing/ota runs it on a PC).
//
// The browser converts the .hex into a binary image (Intel HEX parsed in JavaScript, line checksums checked there),
// pads it to 4 KB and sends it in chunks. The Teensy keeps the image in a spare area of its flash (OTA_BUF_ADDR),
// checks it, and only then copies it over the running firmware from ITCM and restarts (zOta.ino).
//
// Teensy 4.1 flash image (imxrt1062 core, bootdata.c): 0x000 FlexSPI config block (tag "FCFB", flash size at 0x50),
// 0x1000 image vector table (header 0x432000D1, entry point at +4), code after it.
#ifndef Z_OTA_H
#define Z_OTA_H

#include <stdint.h>
#include <string.h>

#define OTA_FLASH_BASE   0x60000000UL
#define OTA_BUF_ADDR     0x60400000UL      // spare flash: the running image is < 1 MB, the EEPROM emulation starts at 0x607C0000
#define OTA_MAX_LEN      0x200000UL        // 2 MB
#define OTA_SECTOR       4096UL
#define OTA_PAGE         256UL
#define OTA_CHUNK_MAX    2048UL
#define OTA_MARKER       "AIO4-OTA-FW-T41"  // present in every firmware that can update itself; a foreign image is refused
#define OTA_PIN_DEFAULT  "1234"
#define OTA_PIN_MIN      4
#define OTA_PIN_MAX      8
#define OTA_MAX_FAILS    3
#define OTA_LOCK_MS      60000UL

inline uint32_t otaCrc32(uint32_t crc, const uint8_t* d, uint32_t n)      // zlib CRC-32; start with 0
{
    crc = ~crc;
    for (uint32_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (uint8_t j = 0; j < 8; j++) crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
    }
    return ~crc;
}

inline uint32_t otaU32(const uint8_t* p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

inline bool otaPinValid(const char* p)
{
    size_t n = strlen(p);
    if (n < OTA_PIN_MIN || n > OTA_PIN_MAX) return false;
    for (size_t i = 0; i < n; i++) if (p[i] < '0' || p[i] > '9') return false;
    return true;
}

// Wrong-PIN guard: 3 wrong PINs in a row lock every PIN check for 60 s.
struct OtaGuard {
    uint8_t  fails = 0;
    uint32_t lockStart = 0;
    bool     locked = false;
    uint32_t lockLeftS(uint32_t now) const
    {
        if (!locked) return 0;
        uint32_t el = now - lockStart;
        return el >= OTA_LOCK_MS ? 0 : (OTA_LOCK_MS - el + 999) / 1000;
    }
    // 0 = PIN ok, 1 = wrong PIN, 2 = locked
    uint8_t check(uint32_t now, const char* stored, const char* given)
    {
        if (locked) { if (lockLeftS(now) > 0) return 2; locked = false; fails = 0; }
        if (strcmp(stored, given) == 0) { fails = 0; return 0; }
        if (++fails >= OTA_MAX_FAILS) { locked = true; lockStart = now; }
        return 1;
    }
};

// Update mode: from a started upload until it ends, fails or stops (no data for 15 s) the main loop runs only the web
// server and the update itself; GPS, IMU, CAN, UDP and steering are paused so nothing disturbs the upload.
#define OTA_MODE_TIMEOUT_MS 15000UL
struct OtaMode {
    bool     on = false;
    uint32_t last = 0;
    void enter(uint32_t now) { on = true; last = now; }
    void touch(uint32_t now) { last = now; }
    void leave() { on = false; }
    bool expired(uint32_t now) const { return on && (int32_t)(now - last) > (int32_t)OTA_MODE_TIMEOUT_MS; }
};

enum { OTA_IDLE = 0, OTA_RECV = 1, OTA_READY = 2, OTA_ERROR = 3 };
enum { OTA_CK_WRITE = 0, OTA_CK_DUP = 1, OTA_CK_ERR = 2 };

struct OtaSession {
    uint8_t  st = OTA_IDLE;
    uint32_t total = 0, got = 0, crc = 0, tok = 0;
    char     err[48] = "";

    void fail(const char* m) { st = OTA_ERROR; strncpy(err, m, sizeof(err) - 1); err[sizeof(err) - 1] = 0; }
    void reset() { *this = OtaSession(); }

    // NULL = started
    const char* begin(uint32_t len, uint32_t imageCrc, uint32_t token)
    {
        if (len < 2 * OTA_SECTOR || len > OTA_MAX_LEN || (len % OTA_SECTOR)) return "bad image length";
        reset();
        st = OTA_RECV; total = len; crc = imageCrc; tok = token;
        return NULL;
    }
    // what to do with a chunk at off of n bytes (CRC of the chunk already checked by the caller)
    uint8_t chunk(uint32_t off, uint32_t n, uint32_t token, const char** why)
    {
        *why = "";
        if (st != OTA_RECV)                   { *why = "no upload running"; return OTA_CK_ERR; }
        if (token != tok)                     { *why = "bad session"; return OTA_CK_ERR; }
        if (n == 0 || n > OTA_CHUNK_MAX || (n % OTA_PAGE)) { *why = "bad chunk size"; return OTA_CK_ERR; }
        if (off % OTA_PAGE)                   { *why = "bad chunk offset"; return OTA_CK_ERR; }
        if (off + n > total)                  { *why = "chunk past the image end"; return OTA_CK_ERR; }
        if (off + n <= got)                   return OTA_CK_DUP;          // answer to it was lost, the browser sent it again
        if (off != got)                       { *why = "chunk out of order"; return OTA_CK_ERR; }
        return OTA_CK_WRITE;
    }
    void written(uint32_t n) { got += n; }
    bool complete() const { return st == OTA_RECV && got == total; }
};

// Checks the whole image as it lies in flash. NULL = good.
inline const char* otaImageCheck(const uint8_t* img, uint32_t len, uint32_t expectCrc)
{
    if (len < 2 * OTA_SECTOR) return "image too short";
    if (otaCrc32(0, img, len) != expectCrc)          return "image CRC mismatch";
    if (otaU32(img) != 0x42464346UL)                  return "no FlexSPI config block (FCFB)";
    if (otaU32(img + 0x50) != 0x00800000UL)           return "not a Teensy 4.1 image (flash size)";
    if (otaU32(img + 0x1000) != 0x432000D1UL)         return "no image vector table";
    uint32_t entry = otaU32(img + 0x1004);
    if (entry < OTA_FLASH_BASE + 0x1000 || entry >= OTA_FLASH_BASE + len) return "entry point outside the image";
    const size_t m = strlen(OTA_MARKER);
    bool found = false;
    for (uint32_t i = 0; i + m <= len && !found; i++)
        if (img[i] == (uint8_t)OTA_MARKER[0] && memcmp(img + i, OTA_MARKER, m) == 0) found = true;
    if (!found) return "not an AIO v4 WebConfig image";
    return NULL;
}

#endif
