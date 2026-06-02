#include "zConfig.h"
#include "TM171_IMU.h"

// ── Serial ports ───────────────────────────────────────────────────────────────
#define SerialAOG Serial         // AgIO USB
#define SerialRTK Serial3        // RTK radio
HardwareSerial* SerialGPS  = &Serial7;  // main GGA receiver
HardwareSerial* SerialGPS2 = &Serial5;  // dual heading receiver
HardwareSerial* SerialIMU  = &Serial5;  // BNO085 RVC (same port, mutex use)
TM171_IMU TM171_IMU(Serial2);           // TM171 on Serial2

constexpr int serial_buffer_size = 1023;

const int32_t baudGPS = 460800;
const int32_t baudRTK = 115200;

#define ImuWire Wire             // SCL=19  SDA=18
#define RAD_TO_DEG_X_10 572.95779513082320876798154814105

// ── Status LEDs ────────────────────────────────────────────────────────────────
#define GGAReceivedLED        13
#define Power_on_LED           5
#define Ethernet_Active_LED    6
#define GPSRED_LED             9
#define GPSGREEN_LED          10
#define AUTOSTEER_STANDBY_LED 11
#define AUTOSTEER_ACTIVE_LED  12

// ── Includes ───────────────────────────────────────────────────────────────────
#include "zNMEAParser.h"
#include <Wire.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <FlexCAN_T4.h>

// ── ModuleConfig instance ──────────────────────────────────────────────────────
ModuleConfig moduleConfig;

// ── IMU state ─────────────────────────────────────────────────────────────────
bool useTMxx_IMU   = false;
bool useBNO08xI2C  = false;
bool useBNO08xRVC  = false;

elapsedMillis TMxx_IMU_Timer = 0;

#define i2cBNO_REPORT_INTERVAL 20
elapsedMillis i2cBNOTimer = 0;
#include "BNO08x_AOG.h"

#include "BNO_RVC.h"
BNO_rvc      rvc;
BNO_rvcData  bnoData;
elapsedMillis bnoTimer  = 0;
bool          bnoTrigger = false;

const uint8_t  bno08xAddresses[]  = { 0x4A, 0x4B };
const int16_t  nrBNO08xAdresses   = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t        bno08xAddress;
BNO080         bno08x;

// ── Physical CAN bus instances (always declared; started on demand by CAN_Setup) ──
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_256> CanBus1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_256> CanBus2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> CanBus3;

// Map a serial number (1-8) to its Teensy HardwareSerial port
HardwareSerial* serialByNum(uint8_t n) {
    switch (n) {
        case 1: return &Serial1; case 2: return &Serial2;
        case 3: return &Serial3; case 4: return &Serial4;
        case 5: return &Serial5; case 6: return &Serial6;
        case 7: return &Serial7; case 8: return &Serial8;
        default: return &Serial7;
    }
}

// ── CAN dispatch helpers ──────────────────────────────────────────────────────
// Route read/write to whichever physical port is configured for a given function.
inline bool canRead(uint8_t mode, CAN_message_t& msg) {
    if (moduleConfig.can1Mode == mode) return CanBus1.read(msg);
    if (moduleConfig.can2Mode == mode) return CanBus2.read(msg);
    if (moduleConfig.can3Mode == mode) return CanBus3.read(msg);
    return false;
}
// Read directly from a physical CAN port (1/2/3) regardless of assigned mode
inline bool canReadPort(uint8_t port, CAN_message_t& msg) {
    if (port == 1) return CanBus1.read(msg);
    if (port == 2) return CanBus2.read(msg);
    if (port == 3) return CanBus3.read(msg);
    return false;
}
inline void canWrite(uint8_t mode, CAN_message_t& msg) {
    if      (moduleConfig.can1Mode == mode) CanBus1.write(msg);
    else if (moduleConfig.can2Mode == mode) CanBus2.write(msg);
    else if (moduleConfig.can3Mode == mode) CanBus3.write(msg);
}
inline bool hasFuncMode(uint8_t mode) {
    return moduleConfig.can1Mode == mode ||
           moduleConfig.can2Mode == mode ||
           moduleConfig.can3Mode == mode;
}

// ── CAN Steer (steer-ready tractor) ──────────────────────────────────────────
uint16_t setCurve           = 32128;  // steering command sent to valve (center=32128)
uint16_t estCurve           = 32128;  // actual wheel angle from tractor CAN
int16_t  FendtSetCurve      = 0;      // Fendt signed delta from center (TX)
int16_t  FendtEstCurve      = 0;      // Fendt signed delta from center (RX)
uint8_t  steeringValveReady = 0;      // tractor valve state byte
bool     canSteerIntend     = false;  // intent to steer flag for VBus_Send
bool     engageCAN          = false;  // tractor CAN engage message received
uint32_t canEngageTime      = 0;      // millis() of last engage message
uint8_t  ISORearHitch       = 0;      // rear hitch position 0-250 (0-100%)
bool     showCANData        = false;  // CAN plot enable

// ── J1939 send timers ─────────────────────────────────────────────────────────
elapsedMillis j1939Timer65267  = 0;
elapsedMillis j1939Timer129029 = 0;

// CAN plot ring buffer (non-static — shared with zCANSteer.ino via extern)
#define CAN_RAW_BUF_SIZE 2048
char    canRawBuf[CAN_RAW_BUF_SIZE];
uint16_t canRawWrite   = 0;
bool     canRawWrapped = false;

elapsedMillis lastKeyaHeatbeat;
bool    keyaDetected           = false;
bool    keyaIntendToSteer;
int16_t keyaCurrentSetSpeed    = 0;
int16_t keyaCurrentActualSpeed = 0;

// ── Keya encoder accumulator ──────────────────────────────────────────────────
int32_t  keyaEncoderRaw      = 0;
uint16_t keyaEncPrev         = 0;
bool     keyaEncInitDone     = false;
bool     keyaInitialZeroDone = false;  // autosteer blocked until first auto-zero done
float    keyaGpsOffset       = 0.0f;  // runtime drift correction (degrees)

// ── HPR RTK quality monitoring (UM982 second antenna) ────────────────────────
bool          hprRtkLost      = false;  // true when main=RTK but secondary lost RTK
bool          hprCutoffActive = false;  // true when lost >10s, fixQuality forced to 0
elapsedMillis hprLostTimer    = 0;      // time since RTK fix was lost
int           mainFixQuality  = 0;      // raw fix quality from main antenna GGA (before override)

// ── Custom CAN engage runtime state ──────────────────────────────────────────
uint8_t  customEngLastBuf[8] = {0};  // last frame seen on configured ID (for Learn)
bool     customEngSeen   = false;    // a frame with the configured ID has arrived
bool     customEngMatch  = false;    // mask+match currently satisfied
bool     adcConnected    = false;  // true if ADS1115 responded at boot
bool logActive       = true;   // enabled on boot to capture setup messages

// ── Graph sampling (Live tab → Graph) ────────────────────────────────────────
#define GRAPH_BATCH_MAX 20
bool          graphActive   = false;     // sampling enabled
uint8_t       graphSig[4]   = {0,0,0,0}; // selected signal IDs per channel
uint16_t      graphRateMs   = 200;       // sample interval (100=10Hz..1000=1Hz)
float         graphBuf[4][GRAPH_BATCH_MAX];
uint8_t       graphBufCount = 0;         // samples buffered since last poll
elapsedMillis graphSampleTimer = 0;
bool logAutoOffDone  = false;  // one-shot: auto-disables log after 10s
bool gpsRawActive    = false;  // GPS raw capture enabled (web UM98x tab)
elapsedMillis logBootTime = 0;

// ── IMU as WAS state ──────────────────────────────────────────────────────────
float         imuWasRawYaw      = 0.0f;
bool          imuWasReceived    = false;
elapsedMillis imuWasTimeout     = 9999;
bool          imuWasZeroRequest = false;

// ── GPS / dual ─────────────────────────────────────────────────────────────────
int8_t  workingDir   = 1;
float   wheelAngleGPS = 0;

bool useDual        = false;
bool dualReadyGGA   = false;
bool dualReadyRelPos = false;
bool dualReadyHPR   = false;  // HPR sentence arrived, waiting for GGA

elapsedMillis GGAReadyTime    = 10000;
elapsedMillis VTGReadyTime    = 10000;
elapsedMillis HPRReadyTime    = 10000;
elapsedMillis RELPOSReadyTime = 10000;  // reset by relPosDecode on valid fix

// ── Performance / message-rate monitoring ────────────────────────────────────
float    loopTimeMs   = 0;   // duration of one loop() iteration
float    loopTimeMax  = 0;   // peak loop time over ~1s window
uint32_t lastLoopMicros = 0;
uint32_t ggaIntervalMs = 0, lastGgaMs = 0;  // time between GGA messages
uint32_t vtgIntervalMs = 0, lastVtgMs = 0;  // time between VTG messages
uint32_t hprIntervalMs = 0, lastHprMs = 0;  // time between HPR messages
elapsedMillis ethernetLinkCheck = 1000;

double headingcorr = 900;
double baseline    = 0;
double rollDual    = 0;
double relPosD     = 0;
double heading     = 0;
double headingVTG  = 0;
double headingRate = 0;
int8_t GPS_Hz      = 10;
int    solQualityHPR;

byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00};

uint8_t GPSrxbuffer[serial_buffer_size];
uint8_t GPStxbuffer[serial_buffer_size];
uint8_t GPS2rxbuffer[serial_buffer_size];
uint8_t GPS2txbuffer[serial_buffer_size];
uint8_t RTKrxbuffer[serial_buffer_size];

NMEAParser<3> parser;

bool isTriggered = false;
bool blink       = false;

bool Autosteer_running = true;

float roll  = 0;
float pitch = 0;
float yaw   = 0;

// ── Ethernet / UDP ─────────────────────────────────────────────────────────────
struct ConfigIP {
    uint8_t ipOne   = 192;
    uint8_t ipTwo   = 168;
    uint8_t ipThree = 5;
}; ConfigIP networkAddress;

byte Eth_myip[4]  = {0, 0, 0, 0};
byte mac[]        = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};

unsigned int portMy            = 5120;
unsigned int AOGNtripPort      = 2233;
unsigned int AOGAutoSteerPort  = 8888;
unsigned int portDestination   = 9999;

char Eth_NTRIP_packetBuffer[serial_buffer_size];

EthernetUDP Eth_udpPAOGI;
EthernetUDP Eth_udpNtrip;
EthernetUDP Eth_udpAutoSteer;

IPAddress Eth_ipDestination;

byte CK_A = 0;
byte CK_B = 0;
int  relposnedByteCount = 0;

elapsedMillis speedPulseUpdateTimer = 0;
byte velocityPWM_Pin = 36;

extern "C" uint32_t set_arm_clock(uint32_t frequency);

// ══════════════════════════════════════════════════════════════════════════════
void setup()
{
    delay(500);
    set_arm_clock(450000000);
    Serial.println("\r\n** AIO v4 WebConfig - " __DATE__ " **\r\n");
    webLogf("** AIO v4 WebConfig %s **", __DATE__);
    Serial.print("CPU: "); Serial.println(F_CPU_ACTUAL);
    webLogf("CPU: %lu Hz", F_CPU_ACTUAL);

    pinMode(GGAReceivedLED,        OUTPUT);
    pinMode(Power_on_LED,          OUTPUT);
    pinMode(Ethernet_Active_LED,   OUTPUT);
    pinMode(GPSRED_LED,            OUTPUT);
    pinMode(GPSGREEN_LED,          OUTPUT);
    pinMode(AUTOSTEER_STANDBY_LED, OUTPUT);
    pinMode(AUTOSTEER_ACTIVE_LED,  OUTPUT);

    // ── Load module config from EEPROM ──────────────────────────────────────
    moduleConfigLoad();

    // ── Apply configurable serial port assignment ───────────────────────────
    SerialGPS = serialByNum(moduleConfig.gpsSerial);

    // ── NMEA parser ─────────────────────────────────────────────────────────
    parser.setErrorHandler(errorHandler);
    parser.addHandler("G-GGA", GGA_Handler);
    parser.addHandler("G-VTG", VTG_Handler);
    parser.addHandler("G-HPR", HPR_Handler);

    // ── GPS serials ─────────────────────────────────────────────────────────
    SerialGPS->begin(moduleConfig.gpsBaud);
    SerialGPS->addMemoryForRead(GPSrxbuffer,  serial_buffer_size);
    SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

    SerialRTK.begin(baudRTK);
    SerialRTK.addMemoryForRead(RTKrxbuffer, serial_buffer_size);

    // Serial5 defaults to 460800 for dual GPS heading receiver
    SerialGPS2->begin(baudGPS);
    SerialGPS2->addMemoryForRead(GPS2rxbuffer,  serial_buffer_size);
    SerialGPS2->addMemoryForWrite(GPS2txbuffer, serial_buffer_size);

    SLOG("GPS serials initialised");

    // ── Autosteer ───────────────────────────────────────────────────────────
    SLOG("Starting AutoSteer...");
    autosteerSetup();

    // ── Ethernet + Web server ───────────────────────────────────────────────
    SLOG("Starting Ethernet...");
    EthernetStart();
    webServerBegin();

    // ── CAN bus (all ports started by CAN_Setup based on canXMode config) ───
    SLOG("Starting CAN bus...");
    CAN_Setup();

    // ── IMU detection/init ──────────────────────────────────────────────────
    SLOG("Setup IMU...");
    setupIMU();

    SLOG("Setup complete, waiting for GPS...");
}

// ── IMU initialisation (respects moduleConfig.imuType) ────────────────────────
void setupIMU()
{
    bool imuFound = false;

    // ── BNO085 RVC (UART Serial5) ───────────────────────────────────────────
    if (moduleConfig.imuType == IMU_BNO_RVC)
    {
        SLOG("Checking BNO085 RVC (Serial5)...");
        SerialIMU->begin(115200);
        rvc.begin(SerialIMU);
        SLOG("  BNO085 RVC init (detection in main loop)");
    }

    // ── BNO085 I2C ──────────────────────────────────────────────────────────
    if (!imuFound && (moduleConfig.imuType == IMU_AUTO ||
                      moduleConfig.imuType == IMU_BNO_I2C))
    {
        SLOG("Checking BNO085 I2C...");
        ImuWire.begin();
        uint8_t error;
        for (int16_t i = 0; i < nrBNO08xAdresses; i++) {
            bno08xAddress = bno08xAddresses[i];
            ImuWire.beginTransmission(bno08xAddress);
            error = ImuWire.endTransmission();
            if (error == 0) {
                Serial.print("  Found at 0x"); Serial.println(bno08xAddress, HEX);
                webLogf("  Found at 0x%02X", bno08xAddress);
                if (bno08x.begin(bno08xAddress, ImuWire)) {
                    ImuWire.setClock(400000);
                    delay(300);
                    bno08x.enableGameRotationVector(i2cBNO_REPORT_INTERVAL);
                    i2cBNOTimer = 0;
                    useBNO08xI2C = true;
                    imuFound = true;
                    SLOG("  BNO085 I2C initialised!");
                }
            } else {
                Serial.print("  0x"); Serial.print(bno08xAddress, HEX);
                Serial.println(" not found.");
                webLogf("  0x%02X not found.", bno08xAddress);
            }
            if (useBNO08xI2C) break;
        }
    }

    // ── TM171 ───────────────────────────────────────────────────────────────
    if (!imuFound && (moduleConfig.imuType == IMU_AUTO ||
                      moduleConfig.imuType == IMU_TM171))
    {
        SLOG("Checking TM171...");
        TM171_IMU.setSerial(serialByNum(moduleConfig.tm171Serial));
        TM171_IMU.begin(moduleConfig.tm171Baud);
        delay(10);
        if (TM171_IMU.detect(1000)) {
            useTMxx_IMU = true;
            imuFound = true;
            SLOG("  TM171 detected!");
        } else {
            SLOG("  TM171 not found.");
        }
    }

    if (!imuFound)
        SLOG("No IMU found (or IMU_NONE selected).");
}

// ══════════════════════════════════════════════════════════════════════════════
void loop()
{
    // ── Loop time measurement (full loop period) ─────────────────────────────
    uint32_t _nowU = micros();
    loopTimeMs = (_nowU - lastLoopMicros) / 1000.0f;
    lastLoopMicros = _nowU;
    static elapsedMillis loopMaxWin = 0;
    if (loopMaxWin > 1000) { loopTimeMax = 0; loopMaxWin = 0; }
    if (loopTimeMs > loopTimeMax) loopTimeMax = loopTimeMs;

    // ── GPS parsing ── drain full buffer each loop (prevents overflow if loop stalls)
    while (SerialGPS->available()) {
        uint8_t _gc = SerialGPS->read();
        gpsRawByte(_gc);
        parser << (char)_gc;
    }

    // ── RTK passthrough ─────────────────────────────────────────────────────
    while (SerialRTK.available()) SerialGPS->write(SerialRTK.read());

    // ── NTRIP via UDP ───────────────────────────────────────────────────────
    unsigned int pktLen = Eth_udpNtrip.parsePacket();
    if (pktLen > 0) {
        if (pktLen > serial_buffer_size) pktLen = serial_buffer_size;
        Eth_udpNtrip.read(Eth_NTRIP_packetBuffer, pktLen);
        SerialGPS->write(Eth_NTRIP_packetBuffer, pktLen);
    }

    // ── Dual GPS: send when both messages ready ──────────────────────────────
    if (dualReadyGGA && dualReadyRelPos) {
        BuildNmea();
        dualReadyGGA = dualReadyRelPos = false;
    }

    // ── RelPos (dual heading) – Serial5 / SerialGPS2 ── only when RELPOS selected
    if (moduleConfig.headingSource == HDG_SRC_RELPOS && !useBNO08xRVC) {
        while (SerialGPS2->available()) {
            uint8_t c = SerialGPS2->read();
            if (relposnedByteCount < 4 && c == ackPacket[relposnedByteCount]) {
                relposnedByteCount++;
            } else if (relposnedByteCount > 3) {
                ackPacket[relposnedByteCount] = c;
                relposnedByteCount++;
            } else {
                relposnedByteCount = 0;
            }
            if (relposnedByteCount > 71) {
                if (calcChecksum()) {
                    digitalWrite(GPSRED_LED, LOW);
                    relPosDecode();
                }
                relposnedByteCount = 0;
            }
        }
    }

    // ── BNO085 I2C ──────────────────────────────────────────────────────────
    if (useBNO08xI2C && i2cBNOTimer > 20) {
        i2cBNOTimer = 0;
        readBNO();
    }

    // ── TM171 ───────────────────────────────────────────────────────────────
    if (useTMxx_IMU && TMxx_IMU_Timer > 20) {
        TMxx_IMU_Timer = 0;
        readTM171();
    }

    // ── BNO085 RVC ──────────────────────────────────────────────────────────
    if (moduleConfig.imuType == IMU_BNO_RVC) readBNO_RVC();

    // ── IMU as WAS via CAN ──────────────────────────────────────────────────
    if (moduleConfig.wasSource == WAS_SOURCE_IMU_CAN) CAN1_Receive();

    // ── CAN loopback test ────────────────────────────────────────────────────
    if (hasFuncMode(CAN_MODE_CANTEST)) CANtest_Receive();

    // ── J1939/NMEA 2000 GPS broadcast ────────────────────────────────────────
    if (hasFuncMode(CAN_MODE_J1939)) {
        if (moduleConfig.j1939En65267 && j1939Timer65267 >= moduleConfig.j1939Rate65267)
        {
            j1939Timer65267 = 0;
            sendJ1939_65267_65256();
        }
        if (moduleConfig.j1939En129029 && j1939Timer129029 >= moduleConfig.j1939Rate129029)
        {
            j1939Timer129029 = 0;
            sendJ1939_129029();
        }
    }

    // ── Autosteer loop ──────────────────────────────────────────────────────
    if (Autosteer_running) autosteerLoop();
    else ReceiveUdp();

    // ── Graph sampling (high-rate internal buffer, browser polls batches) ────
    if (graphActive && graphSampleTimer >= graphRateMs) {
        graphSampleTimer = 0;
        graphSampleNow();
    }

    // ── Web server (non-blocking – only active when browser connects) ────────
    handleWebClient();

    // ── HPR RTK quality monitor (UM982 second antenna) ───────────────────────
    if (hprRtkLost && moduleConfig.headingSource == HDG_SRC_HPR) {
        static elapsedMillis hprWarnTimer = 4001;
        if (hprWarnTimer > 4000) {
            sendDisplayMessage("Heading antenna: RTK fix failed!", 5, 0);
            hprWarnTimer = 0;
        }
        if (!hprCutoffActive && hprLostTimer > 10000) {
            hprCutoffActive = true;
            sendDisplayMessage("AUTOSTEER DISABLED: RTK fix lost >10s", 6, 0);
            disengageLog("HPR RTK lost >10s (fixQuality forced 0)");
            Serial.println("HPR RTK: cutoff active after 10s");
        }
    }

    // ── Log auto-off 10s after boot (captures setup, then frees CPU) ────────
    if (!logAutoOffDone && logBootTime > 10000) {
        logActive = false;
        logAutoOffDone = true;
        Serial.println("Web log auto-disabled after 10s");
    }

    // ── GPS timeout ─────────────────────────────────────────────────────────
    if (GGAReadyTime > 10000) {
        digitalWrite(GPSRED_LED,   LOW);
        digitalWrite(GPSGREEN_LED, LOW);
        useDual = false;
    }

    // ── Ethernet link LED ───────────────────────────────────────────────────
    if (ethernetLinkCheck > 10000) {
        ethernetLinkCheck = 0;
        if (Ethernet.linkStatus() == LinkON) {
            digitalWrite(Power_on_LED,        LOW);
            digitalWrite(Ethernet_Active_LED, HIGH);
        } else {
            digitalWrite(Power_on_LED,        HIGH);
            digitalWrite(Ethernet_Active_LED, LOW);
        }
    }
}

// ── RelPos checksum ────────────────────────────────────────────────────────────
bool calcChecksum()
{
    CK_A = CK_B = 0;
    for (int i = 2; i < 70; i++) {
        CK_A += ackPacket[i];
        CK_B += CK_A;
    }
    return (CK_A == ackPacket[70] && CK_B == ackPacket[71]);
}
