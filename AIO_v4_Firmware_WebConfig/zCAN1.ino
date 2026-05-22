#define IMU_WAS_CAN_ID  0x300

void CAN1_Receive()
{
    CAN_message_t msg;
    while (canRead(CAN_MODE_IMU, msg)) {
        if (msg.id == IMU_WAS_CAN_ID) {
            uint8_t status = msg.buf[4];
            if (!(status & 0x01)) continue;
            int16_t yaw_cdeg = (int16_t)(((uint16_t)msg.buf[0] << 8) | msg.buf[1]);
            imuWasRawYaw   = yaw_cdeg * 0.01f;
            imuWasReceived = true;
            imuWasTimeout  = 0;
        }
    }
}
