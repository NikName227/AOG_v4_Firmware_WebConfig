
// ── J1939 / NMEA 2000 GPS broadcast ───────────────────────────────────────────
//
// Transmits GPS position on whichever CAN port is set to CAN_MODE_J1939.
// Two PGN options (independently enable/rate-configurable via web UI):
//
//   PGN 65267  — GPS position    (pivot Lat/Lon, 8 bytes)
//   PGN 65256  — direction/speed (heading, speed, pitch, altitude, 8 bytes)
//   PGN 129029 — NMEA 2000 fast-packet GNSS position data (48 bytes / 7 frames)
//
// GPS values are updated by j1939UpdateFromGGA() / j1939UpdateFromVTG()
// called at end of GGA_Handler() and VTG_Handler() in zHandlers.ino.

// ── Parsed GPS floats (populated from NMEA handlers) ──────────────────────────
float    j1939Lat      = 0.0f;
float    j1939Lon      = 0.0f;
float    j1939Alt      = 0.0f;
float    j1939Geoid    = 0.0f;
float    j1939Heading  = 0.0f;   // degrees true (from VTG)
float    j1939SpeedKmh = 0.0f;   // km/h (from VTG knots × 1.852)
float    j1939HDOP     = 1.0f;
float    j1939RtkAge   = 0.0f;
float    j1939UtcSec   = 0.0f;   // seconds since midnight
uint16_t j1939Days     = 0;      // days since 1970-01-01 (0 = unknown → fallback)
uint8_t  j1939FixType  = 0;
uint8_t  j1939Sats     = 0;

// ── NMEA DDmm.mmmm → decimal degrees ──────────────────────────────────────────
static float parseNMEACoord(const char* coord, const char* dir) {
    if (!coord || coord[0] == '\0') return 0.0f;
    float raw = atof(coord);
    int   deg = (int)(raw * 0.01f);
    float dec = deg + (raw - deg * 100.0f) / 60.0f;
    if (dir[0] == 'S' || dir[0] == 'W') dec = -dec;
    return dec;
}

// ── NMEA HHMMSS.SS → seconds since midnight ────────────────────────────────────
static float parseNMEATime(const char* t) {
    if (!t || t[0] == '\0') return 0.0f;
    int hh = (t[0] - '0') * 10 + (t[1] - '0');
    int mm = (t[2] - '0') * 10 + (t[3] - '0');
    return hh * 3600.0f + mm * 60.0f + atof(t + 4);
}

// ── Called from GGA_Handler() in zHandlers.ino ────────────────────────────────
void j1939UpdateFromGGA() {
    j1939Lat     = parseNMEACoord(latitude, latNS);
    j1939Lon     = parseNMEACoord(longitude, lonEW);
    j1939Alt     = atof(altitude);
    j1939Geoid   = atof(geoidalSep);
    j1939FixType = atoi(fixQuality);
    j1939Sats    = atoi(numSats);
    j1939HDOP    = atof(HDOP);
    j1939RtkAge  = atof(ageDGPS);
    j1939UtcSec  = parseNMEATime(fixTime);
}

// ── Called from VTG_Handler() in zHandlers.ino ────────────────────────────────
void j1939UpdateFromVTG() {
    j1939Heading   = headingVTG;
    j1939SpeedKmh  = atof(speedKnots) * 1.852f;
}

// ── Encode J1939 29-bit extended CAN ID ───────────────────────────────────────
static uint32_t j1939Encode(uint32_t pgn, uint8_t priority, uint8_t src, uint8_t dst) {
    uint32_t id = (uint32_t)(priority & 0x07) << 26;
    if ((pgn > 0 && pgn <= 0xEFFF) || (pgn > 0x10000 && pgn <= 0x1EFFF)) {
        pgn = (pgn & 0x01FF00) | dst;
    }
    id |= (pgn << 8);
    id |= src;
    return id;
}

// ── PGN 65267 + 65256: simple J1939 position & direction ──────────────────────
void sendJ1939_65267_65256() {
    uint8_t src = moduleConfig.j1939SrcAddr;
    CAN_message_t msg;
    msg.flags.extended = true;
    msg.len = 8;

    // PGN 65267 — GPS position (Lat/Lon, offset +210 deg, resolution 1e-7)
    msg.id = j1939Encode(65267, 6, src, 255);
    uint32_t latRaw = (uint32_t)((j1939Lat + 210.0f) * 10000000.0f);
    uint32_t lonRaw = (uint32_t)((j1939Lon + 210.0f) * 10000000.0f);
    msg.buf[0] = latRaw        & 0xFF; msg.buf[1] = (latRaw >> 8)  & 0xFF;
    msg.buf[2] = (latRaw >> 16) & 0xFF; msg.buf[3] = (latRaw >> 24) & 0xFF;
    msg.buf[4] = lonRaw        & 0xFF; msg.buf[5] = (lonRaw >> 8)  & 0xFF;
    msg.buf[6] = (lonRaw >> 16) & 0xFF; msg.buf[7] = (lonRaw >> 24) & 0xFF;
    canWrite(CAN_MODE_J1939, msg);

    // PGN 65256 — vehicle direction & speed
    msg.id = j1939Encode(65256, 6, src, 255);
    uint16_t hdg    = (uint16_t)(j1939Heading   * 128.0f);
    uint16_t spd    = (uint16_t)(j1939SpeedKmh  * 256.0f);
    uint16_t pitch  = 200 * 128;   // zero pitch
    uint16_t altRaw = (uint16_t)((j1939Alt + 2500.0f) * 8.0f);
    msg.buf[0] = hdg    & 0xFF; msg.buf[1] = (hdg    >> 8) & 0xFF;
    msg.buf[2] = spd    & 0xFF; msg.buf[3] = (spd    >> 8) & 0xFF;
    msg.buf[4] = pitch  & 0xFF; msg.buf[5] = (pitch  >> 8) & 0xFF;
    msg.buf[6] = altRaw & 0xFF; msg.buf[7] = (altRaw >> 8) & 0xFF;
    canWrite(CAN_MODE_J1939, msg);
}

// ── PGN 129029: NMEA 2000 fast-packet GNSS position data (48 bytes, 7 frames) ─
void sendJ1939_129029() {
    uint8_t src = moduleConfig.j1939SrcAddr;
    static uint8_t sessionCnt = 0;

    // Build 48-byte N2K payload
    uint8_t d[48];
    memset(d, 0xFF, sizeof(d));

    d[0] = src;   // SID

    // Bytes 1-2: days since 1970 (fallback to 2023-03-01 = 19417 if unknown)
    uint16_t days = (j1939Days > 0) ? j1939Days : 19417;
    d[1] = days & 0xFF; d[2] = (days >> 8) & 0xFF;

    // Bytes 3-6: UTC time ×10000 (int32 LE)
    int32_t utcRaw = (int32_t)(j1939UtcSec * 10000.0f);
    memcpy(d + 3, &utcRaw, 4);

    // Bytes 7-14: latitude ×1e16 (int64 LE)
    int64_t latRaw = (int64_t)((double)j1939Lat * 1e16);
    memcpy(d + 7, &latRaw, 8);

    // Bytes 15-22: longitude ×1e16 (int64 LE)
    int64_t lonRaw = (int64_t)((double)j1939Lon * 1e16);
    memcpy(d + 15, &lonRaw, 8);

    // Bytes 23-30: altitude ×1e16 (int64 LE)
    int64_t altRaw = (int64_t)((double)j1939Alt * 1e16);
    memcpy(d + 23, &altRaw, 8);

    // Byte 31: fix type (GGA quality indicator)
    d[31] = j1939FixType;

    // Byte 32: integrity 2-bit + reserved 6-bit
    d[32] = 0x01 | 0xFC;

    // Byte 33: number of satellites
    d[33] = j1939Sats;

    // Bytes 34-35: HDOP ×100 (int16 LE)
    int16_t hdopRaw = (int16_t)(j1939HDOP * 100.0f);
    d[34] = hdopRaw & 0xFF; d[35] = (hdopRaw >> 8) & 0xFF;

    // Bytes 36-37: PDOP = copy of HDOP
    d[36] = d[34]; d[37] = d[35];

    // Bytes 38-41: geoidal separation ×100 (int32 LE)
    int32_t geoidRaw = (int32_t)(j1939Geoid * 100.0f);
    memcpy(d + 38, &geoidRaw, 4);

    // Bytes 42-46: reference station + RTK age
    if (j1939RtkAge > 0.0f) {
        d[42] = 0x01;             // one reference station
        d[43] = 0x00; d[44] = 0x02;  // GPS+GLONASS type, ID=0
        int16_t ageRaw = (int16_t)(j1939RtkAge * 100.0f);
        d[45] = ageRaw & 0xFF; d[46] = (ageRaw >> 8) & 0xFF;
    }
    // d[47] stays 0xFF (spare)

    // Send 7 CAN frames (N2K fast-packet format)
    CAN_message_t msg;
    msg.id = j1939Encode(129029, 6, src, 255);
    msg.flags.extended = true;
    msg.len = 8;
    uint8_t sessId = (sessionCnt & 0x07) << 5;

    for (uint8_t frame = 0; frame < 7; frame++) {
        msg.buf[0] = sessId | frame;
        if (frame == 0) {
            msg.buf[1] = 48;           // total payload length
            memcpy(msg.buf + 2, d, 6); // first 6 bytes
        } else {
            memcpy(msg.buf + 1, d + 6 + (frame - 1) * 7, 7); // 7 bytes each
        }
        canWrite(CAN_MODE_J1939, msg);
    }
    sessionCnt++;
}
