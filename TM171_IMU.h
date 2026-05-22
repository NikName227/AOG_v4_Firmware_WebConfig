#ifndef TM171_IMU_H
#define TM171_IMU_H

#include <Arduino.h>

class TM171_IMU
{
public:

    TM171_IMU(HardwareSerial &serial, uint16_t timeout = 500);

    void begin(uint32_t baud);

    void readAngles();              // čita nove podatke
    bool isValid();                 // provjera timeout-a
    bool detect(uint16_t timeout);  // detekcija senzora u setupu

    int32_t getRoll();
    int32_t getPitch();
    int32_t getYaw();

    const char* getRollStr();
    const char* getPitchStr();
    const char* getYawStr();

private:

    HardwareSerial &_serial;

    uint16_t timeoutMs;
    uint32_t lastUpdate = 0;

    int32_t roll  = 0;
    int32_t pitch = 0;
    int32_t yaw   = 0;

    char rollStr[12];
    char pitchStr[12];
    char yawStr[12];

    uint8_t buffer[128];
    uint16_t bufferLen = 0;

    uint16_t crc16(uint8_t *data, uint16_t len);
    bool parsePacket();
};

#endif
