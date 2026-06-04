// ─────────────────────────────────────────────────────────────────────────────
// WheelCalib_ESP32  —  reference wheel-angle sensor for Keya WAS auto-calibration
//
// Reads a wheel-mounted IMU (mounted flat on the steered wheel), runs as a WiFi
// soft-AP, and broadcasts roll/pitch/yaw over UDP. The laptop bridge app
// (WheelCalib_Bridge) connects to this AP, shows the angles (for flat mounting),
// and forwards the yaw to the Teensy as the reference wheel angle (PGN 0xD6).
//
// Supports BOTH sensors and AUTO-DETECTS which is connected:
//   • BNO085  on I2C   (SDA=GPIO21, SCL=GPIO22)  — preferred if present
//   • TM171   on UART2 (RX=GPIO16, TX=GPIO17, 115200)
// If neither is found it streams slowly-varying SIMULATED values so the whole
// pipeline (ESP → bridge → Teensy → web GUI) is testable without any IMU.
//
// Board: any ESP32 dev board.
// Library: BNO085 uses the bundled BNO08x_AOG driver (same SparkFun BNO080 fork as
//          the Teensy firmware) — BNO08x_AOG.h/.cpp sit in this sketch folder, so
//          no Library Manager install is needed.
// WiFi:  soft-AP  SSID "WheelCalib"  PASS "calib1234"  →  ESP IP 192.168.4.1
// UDP:   broadcasts "roll,pitch,yaw,imuOk,sensor\n" to 192.168.4.255:9000 @50 Hz
//        imuOk : 1 = real IMU, 0 = simulated
//        sensor: 0 = none/sim, 1 = TM171, 2 = BNO085, 3 = forced-sim (real IMU present)
// CMD:   listens on :9001 for "SIM1"/"SIM0" — force the clean test sinusoid on/off
//        (link-quality check: a smooth sine end-to-end = no drops/jitter).
// ─────────────────────────────────────────────────────────────────────────────

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "BNO08x_AOG.h"

// ── BNO085 (I2C) ────────────────────────────────────────────────────────────
#define I2C_SDA 21
#define I2C_SCL 22
BNO080  bno;
bool    hasBno = false;

// Which sensor is feeding the output: 0 = none/sim, 1 = TM171, 2 = BNO085
uint8_t sensorType = 0;

// ── TM171 UART ────────────────────────────────────────────────────────────────
#define TM_RX_PIN 16
#define TM_TX_PIN 17
#define TM_BAUD   115200
HardwareSerial TM(2);

// ── WiFi soft-AP ────────────────────────────────────────────────────────────
const char* AP_SSID = "WheelCalib";
const char* AP_PASS = "12345678";        // >= 8 chars
WiFiUDP   udp;
const uint16_t UDP_PORT = 9000;
IPAddress bcastIP(192, 168, 4, 255);

// Command channel: the bridge sends "SIM1"/"SIM0" here to force the simulated
// test sinusoid ON/OFF even when a real IMU is connected (link-quality check).
WiFiUDP   udpCmd;
const uint16_t CMD_PORT = 9001;
bool      forceSim = false;

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
                sensorType = 1;                 // TM171
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

    // BNO085 on I2C (preferred). Probe both common addresses.
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    if (bno.begin(0x4A, Wire) || bno.begin(0x4B, Wire)) {
        hasBno = true;
        bno.enableRotationVector(20);           // 50 Hz fused orientation
        Serial.println("IMU: BNO085 detected on I2C");
    } else {
        Serial.println("IMU: no BNO085 on I2C - using TM171 UART if present");
    }

    // TM171 on UART2 (used when no BNO085 is found)
    TM.begin(TM_BAUD, SERIAL_8N1, TM_RX_PIN, TM_TX_PIN);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);          // default IP 192.168.4.1
    udp.begin(UDP_PORT);
    udpCmd.begin(CMD_PORT);                  // listen for SIM1/SIM0 commands
    Serial.printf("WheelCalib AP up: %s  IP %s  UDP %u  CMD %u\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str(), UDP_PORT, CMD_PORT);
}

uint32_t lastSend = 0;
bool     simNotified = false;

void loop() {
    // Poll for SIM1/SIM0 commands from the bridge (force the test sinusoid).
    if (udpCmd.parsePacket()) {
        char c[8] = {0};
        int n = udpCmd.read((uint8_t*)c, sizeof(c) - 1);
        if (n > 0) c[n] = 0;
        if      (strncmp(c, "SIM1", 4) == 0) forceSim = true;
        else if (strncmp(c, "SIM0", 4) == 0) forceSim = false;
    }

    // Read whichever IMU is connected (BNO085 preferred, else TM171).
    if (hasBno && bno.dataAvailable()) {
        roll  = bno.getRoll()  * RAD_TO_DEG;
        pitch = bno.getPitch() * RAD_TO_DEG;
        yaw   = bno.getYaw()   * RAD_TO_DEG;     // heading
        lastPkt = millis();
        sensorType = 2;                          // BNO085
    } else if (!hasBno) {
        parseTM();                               // sets sensorType=1 on a valid frame
    }
    bool realImu = (lastPkt != 0) && (millis() - lastPkt < 1000);
    bool useSim  = forceSim || !realImu;         // forced test, or no IMU at all

    if (millis() - lastSend >= 20) {        // 50 Hz
        lastSend = millis();

        float oRoll = roll, oPitch = pitch, oYaw = yaw;
        if (useSim) {
            // Clean, deterministic sinusoid — sent through the whole pipeline so a
            // smooth trace on the web graph proves the link has no drops/jitter.
            float t = millis() / 1000.0f;
            oRoll  =  2.0f * sinf(t * 0.50f);     // small wobble
            oPitch =  1.5f * sinf(t * 0.30f);
            oYaw   = 15.0f * sinf(t * 0.20f);     // bigger swing, easy to see
            if (!simNotified) {
                Serial.println(forceSim ? "FORCED simulation (link test)"
                                        : "no IMU detected - sending simulated values");
                simNotified = true;
            }
        } else {
            simNotified = false;
        }

        // When forcing sim with a real IMU present, mark sensor=3 so the bridge
        // can show "forced"; otherwise 0 none / 1 TM171 / 2 BNO085.
        uint8_t sensOut = useSim ? (forceSim && realImu ? 3 : 0) : sensorType;

        char msg[64];
        // roll,pitch,yaw,imuOk,sensor  (imuOk 1=real 0=sim; sensor 0=none 1=TM171 2=BNO085 3=forced-sim)
        int n = snprintf(msg, sizeof(msg), "%.2f,%.2f,%.2f,%d,%d\n",
                         oRoll, oPitch, oYaw, useSim ? 0 : 1, sensOut);
        udp.beginPacket(bcastIP, UDP_PORT);
        udp.write((const uint8_t*)msg, n);
        udp.endPacket();
    }
}
