#include "TM171_IMU.h"

TM171_IMU::TM171_IMU(HardwareSerial &serial, uint16_t timeout)
    : _serial(&serial), timeoutMs(timeout)
{
}

void TM171_IMU::setSerial(HardwareSerial *serial)
{
    if (serial) _serial = serial;
}

void TM171_IMU::begin(uint32_t baud)
{
    _serial->begin(baud);
}

uint16_t TM171_IMU::crc16(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }

    return crc;
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

            if (recvCRC == calcCRC && length == 20)
            {
                float r, p, y;

                memcpy(&r, &buffer[idx + 3 + 8], 4);
                memcpy(&p, &buffer[idx + 3 + 12], 4);
                memcpy(&y, &buffer[idx + 3 + 16], 4);

                roll  = (int32_t)(r * 100.0f);
                pitch = (int32_t)(p * 100.0f);
                yaw   = (int32_t)(y * 100.0f);

                lastUpdate = millis();

                found = true;
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
        if (bufferLen < sizeof(buffer))
            buffer[bufferLen++] = _serial->read();
        else
            bufferLen = 0;
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
