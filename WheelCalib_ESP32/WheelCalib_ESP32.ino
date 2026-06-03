// ─────────────────────────────────────────────────────────────────────────────
// WheelCalib_ESP32  —  reference wheel-angle sensor for Keya WAS auto-calibration
//
// Reads a TM171 IMU (UART) mounted flat on the steered wheel, runs as a WiFi
// soft-AP, and broadcasts roll/pitch/yaw over UDP. The laptop bridge app
// (WheelCalib_Bridge) connects to this AP, shows the angles (for flat mounting),
// and forwards the yaw to the Teensy as the reference wheel angle (PGN 0xD6).
//
// Board: any ESP32 dev board.
// TM171: UART2  RX=GPIO16  TX=GPIO17  @115200  (adjust pins below if needed)
// WiFi:  soft-AP  SSID "WheelCalib"  PASS "calib1234"  →  ESP IP 192.168.4.1
// UDP:   broadcasts "roll,pitch,yaw,fresh\n" to 192.168.4.255:9000 @50 Hz
// ─────────────────────────────────────────────────────────────────────────────

#include <WiFi.h>
#include <WiFiUdp.h>

// ── TM171 UART ────────────────────────────────────────────────────────────────
#define TM_RX_PIN 16
#define TM_TX_PIN 17
#define TM_BAUD   115200
HardwareSerial TM(2);

// ── WiFi soft-AP ────────────────────────────────────────────────────────────
const char* AP_SSID = "WheelCalib";
const char* AP_PASS = "calib1234";        // >= 8 chars
WiFiUDP   udp;
const uint16_t UDP_PORT = 9000;
IPAddress bcastIP(192, 168, 4, 255);

// ── TM171 parser state ────────────────────────────────────────────────────────
uint8_t  buf[128];
uint16_t blen = 0;
float    roll = 0, pitch = 0, yaw = 0;
uint32_t lastPkt = 0;

uint16_t crc16(uint8_t* d, uint16_t n) {
    uint16_t c = 0xFFFF;
    for (uint16_t i = 0; i < n; i++) {
        c ^= d[i];
        for (uint8_t j = 0; j < 8; j++) c = (c & 1) ? (c >> 1) ^ 0xA001 : c >> 1;
    }
    return c;
}

void parseTM() {
    while (TM.available()) {
        if (blen < sizeof(buf)) buf[blen++] = TM.read();
        else blen = 0;
    }
    uint16_t idx = 0;
    while (idx + 3 < blen) {
        if (buf[idx] == 0xAA && buf[idx + 1] == 0x55) {
            uint8_t  len = buf[idx + 2];
            uint16_t pk  = 2 + 1 + len + 2;
            if (idx + pk > blen) break;
            uint16_t rc = buf[idx + pk - 2] | (buf[idx + pk - 1] << 8);
            if (rc == crc16(&buf[idx + 2], 1 + len) && len == 20) {
                float r, p, y;
                memcpy(&r, &buf[idx + 3 + 8],  4);
                memcpy(&p, &buf[idx + 3 + 12], 4);
                memcpy(&y, &buf[idx + 3 + 16], 4);
                roll = r; pitch = p; yaw = y;
                lastPkt = millis();
            }
            idx += pk;
        } else {
            idx++;
        }
    }
    if (idx < blen) { memmove(buf, &buf[idx], blen - idx); blen -= idx; }
    else            { blen = 0; }
}

void setup() {
    Serial.begin(115200);
    TM.begin(TM_BAUD, SERIAL_8N1, TM_RX_PIN, TM_TX_PIN);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);          // default IP 192.168.4.1
    udp.begin(UDP_PORT);
    Serial.printf("WheelCalib AP up: %s  IP %s  UDP %u\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str(), UDP_PORT);
}

uint32_t lastSend = 0;
void loop() {
    parseTM();
    if (millis() - lastSend >= 20) {        // 50 Hz
        lastSend = millis();
        bool fresh = (millis() - lastPkt) < 500;
        char msg[64];
        int n = snprintf(msg, sizeof(msg), "%.2f,%.2f,%.2f,%d\n",
                         roll, pitch, yaw, fresh ? 1 : 0);
        udp.beginPacket(bcastIP, UDP_PORT);
        udp.write((const uint8_t*)msg, n);
        udp.endPacket();
    }
}
