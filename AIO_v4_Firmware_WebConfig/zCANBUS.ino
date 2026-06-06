
// templates for commands with matching responses, only need first 4 bytes
uint8_t keyaDisableCommand[] = { 0x23, 0x0C, 0x20, 0x01 };
uint8_t keyaDisableResponse[] = { 0x60, 0x0C, 0x20, 0x00 };

uint8_t keyaEnableCommand[] = { 0x23, 0x0D, 0x20, 0x01 };
uint8_t keyaEnableResponse[] = { 0x60, 0x0D, 0x20, 0x00 };

uint8_t keyaSpeedCommand[] = { 0x23, 0x00, 0x20, 0x01 };
uint8_t keyaSpeedResponse[] = { 0x60, 0x00, 0x20, 0x00 };

uint8_t keyaCurrentQuery[] = { 0x40, 0x00, 0x21, 0x01 };
uint8_t keyaCurrentResponse[] = { 0x60, 0x00, 0x21, 0x01 };

uint8_t keyaFaultQuery[] = { 0x40, 0x12, 0x21, 0x01 };
uint8_t keyaFaultResponse[] = { 0x60, 0x12, 0x21, 0x01 };

uint8_t keyaVoltageQuery[] = { 0x40, 0x0D, 0x21, 0x02 };
uint8_t keyaVoltageResponse[] = { 0x60, 0x0D, 0x21, 0x02 };

uint8_t keyaTemperatureQuery[] = { 0x40, 0x0F, 0x21, 0x01 };
uint8_t keyaTemperatureResponse[] = { 0x60, 0x0F, 0x21, 0x01 };

uint8_t keyaVersionQuery[] = { 0x40, 0x01, 0x11, 0x11 };
uint8_t keyaVersionResponse[] = { 0x60, 0x01, 0x11, 0x11 };

uint8_t keyaEncoderResponse[] = { 0x60, 0x04, 0x21, 0x01 };

uint8_t keyaEncoderSpeedResponse[] = { 0x60, 0x03, 0x21, 0x01 };

uint64_t KeyaID = 0x06000001; // 0x01 is default ID

uint32_t MillisKeyaHandDetection = 0;

void CAN_Setup()
{
    static const char* modeNames[] = {"Off","Keya","IMU-WAS","V_Bus","K_Bus","ISO","J1939","CANtest","Custom"};
    if (moduleConfig.can1Mode != CAN_MODE_OFF) {
        CanBus1.begin(); CanBus1.setBaudRate(moduleConfig.can1Baud);
        Serial.printf("CAN1: %s  %lu kbps\n", modeNames[moduleConfig.can1Mode], moduleConfig.can1Baud/1000);
    }
    if (moduleConfig.can2Mode != CAN_MODE_OFF) {
        CanBus2.begin(); CanBus2.setBaudRate(moduleConfig.can2Baud);
        Serial.printf("CAN2: %s  %lu kbps\n", modeNames[moduleConfig.can2Mode], moduleConfig.can2Baud/1000);
    }
    if (moduleConfig.can3Mode != CAN_MODE_OFF) {
        CanBus3.begin(); CanBus3.setBaudRate(moduleConfig.can3Baud);
        Serial.printf("CAN3: %s  %lu kbps\n", modeNames[moduleConfig.can3Mode], moduleConfig.can3Baud/1000);
    }

    // Custom engage: ensure its listen port is started even if its mode is OFF
    if (moduleConfig.customEngageEnable) {
        uint8_t p = moduleConfig.customEngageCanPort;
        if (p == 1 && moduleConfig.can1Mode == CAN_MODE_OFF) { CanBus1.begin(); CanBus1.setBaudRate(moduleConfig.can1Baud); }
        if (p == 2 && moduleConfig.can2Mode == CAN_MODE_OFF) { CanBus2.begin(); CanBus2.setBaudRate(moduleConfig.can2Baud); }
        if (p == 3 && moduleConfig.can3Mode == CAN_MODE_OFF) { CanBus3.begin(); CanBus3.setBaudRate(moduleConfig.can3Baud); }
        Serial.printf("Custom engage: listening CAN%u id=0x%lX\n", p, (unsigned long)moduleConfig.customEngageId);
    }

    // J1939 address claim for steer-ready tractors (announce module on the bus)
    delay(300);
    sendAddressClaim();
}

// ── Custom CAN engage — mask+match on a user-defined frame ───────────────────
// Toggle mode: rising edge of match toggles engage (momentary button).
// Level mode:  both edges toggle (latched switch ON→engage, OFF→disengage).
void CustomEngage_Receive()
{
    if (!moduleConfig.customEngageEnable) return;
    CAN_message_t msg;
    static bool prevMatch = false;
    while (canReadPort(moduleConfig.customEngageCanPort, msg)) {
        bool idOk = (msg.id == moduleConfig.customEngageId)
                 && (msg.flags.extended == (moduleConfig.customEngageExt ? 1 : 0));
        if (!idOk) continue;

        // store last frame for Learn capture
        for (uint8_t i = 0; i < 8; i++) customEngLastBuf[i] = msg.buf[i];
        customEngSeen = true;

        bool match = true;
        for (uint8_t i = 0; i < 8; i++) {
            uint8_t m = moduleConfig.customEngageMask[i];
            if (m && ((msg.buf[i] & m) != (moduleConfig.customEngageMatch[i] & m))) { match = false; break; }
        }
        customEngMatch = match;

        if (moduleConfig.customEngageMode == 0) {
            // toggle: pulse engage on rising edge only
            if (match && !prevMatch) { engageCAN = true; disengageLog("custom CAN engage (toggle)"); }
        } else {
            // level: pulse on both edges (engage on press, disengage on release)
            if (match != prevMatch) { engageCAN = true; disengageLog("custom CAN engage (level edge)"); }
        }
        prevMatch = match;
    }
}

void KeyaBus_Receive()
{
    CAN_message_t msg;
    while (canRead(CAN_MODE_KEYA, msg))
    {
        // Keya config response (param dump on enter config)
        if (msg.id == 0x181) { keyaCfgParse(msg); continue; }

        // Heartbeat
        if (msg.id == 0x07000001)
        {
            lastKeyaHeatbeat = 0;

            if (!keyaDetected)
            {
                Serial.println("Keya heartbeat detected! Enabling Keya CANBUS");
                keyaDetected = true;
            }

            // Cumulative delta encoder (Flodu model) — handles uint16 overflow correctly
            uint16_t encTick = ((uint16_t)msg.buf[0] << 8) | (uint16_t)msg.buf[1];
            if (!keyaEncInitDone) {
                keyaEncPrev     = encTick;
                keyaEncInitDone = true;
            } else {
                int16_t delta = (int16_t)(encTick - keyaEncPrev);
                keyaEncoderRaw += delta;
                keyaEncPrev = encTick;
            }

            keyaCurrentActualSpeed = (int16_t)((int16_t)msg.buf[2] << 8 | (int16_t)msg.buf[3]);
            int16_t error = abs(keyaCurrentActualSpeed - keyaCurrentSetSpeed);
            static int16_t counter = 0;

            // NV speed-direction disengage.
            // Standing still (< 0.3 km/h) the motor is barely commanded, so the
            // set-speed gate would block detection — drop it to 0 there so grabbing
            // the wheel still disengages.
            int16_t setMinEff = (gpsSpeed < 0.3f) ? 0 : moduleConfig.keyaSetSpeedMin;
            if (moduleConfig.keyaDisEnable
                && abs(keyaCurrentSetSpeed)    > setMinEff
                && abs(keyaCurrentActualSpeed) > moduleConfig.keyaActSpeedMin
                && steerSwitch == 0)
            {
                if ((int32_t)keyaCurrentActualSpeed * keyaCurrentSetSpeed < 0) {
                    if (millis() - MillisKeyaHandDetection > moduleConfig.speedDiffTimeout) {
                        disengageLog("Keya speed-direction (hand override)");
                        steerSwitch = 1;
                        currentState = 1;
                        previous = 0;
                    }
                } else {
                    MillisKeyaHandDetection = millis();
                }
            }

            if (DBG_KEYA_DIFF) {
                static uint32_t previousMillisKeya = 0;
                if (millis() - previousMillisKeya >= 500) {
                    previousMillisKeya = millis();
                    Serial.print("ActSped: ");  Serial.print(keyaCurrentActualSpeed);
                    Serial.print(" Curset: ");  Serial.print(keyaCurrentSetSpeed);
                    Serial.print(" param: ");   Serial.println(moduleConfig.speedDiffTimeout);
                    webLogf("Keya ActSped:%d Curset:%d timeout:%u ms",
                        keyaCurrentActualSpeed, keyaCurrentSetSpeed,
                        moduleConfig.speedDiffTimeout);
                }
            }

            if (error > abs(keyaCurrentSetSpeed) + 10)
            {
                if (counter++ < 8) { }
                else { sensorReading = abs(abs(keyaCurrentSetSpeed) - error); }
            }
            else
            {
                sensorReading = 0;
                counter = 0;
            }

            if (msg.buf[7] > 1 || msg.buf[6] > 0)
            {
                if (bitRead(msg.buf[7], 1)) Serial.print("Over voltage\t");
                if (bitRead(msg.buf[7], 2)) Serial.print("Hardware protection\t");
                if (bitRead(msg.buf[7], 3)) Serial.print("E2PROM\t");
                if (bitRead(msg.buf[7], 4)) Serial.print("Under voltage\t");
                if (bitRead(msg.buf[7], 5)) Serial.print("N/A\t");
                if (bitRead(msg.buf[7], 6)) Serial.print("Over current\t");
                if (bitRead(msg.buf[7], 7)) Serial.print("Mode failure\t");
                if (bitRead(msg.buf[6], 0)) Serial.print("Less phase\t");
                if (bitRead(msg.buf[6], 1)) Serial.print("Motor stall\t");
                if (bitRead(msg.buf[6], 3)) Serial.print("Hall failure\t");
                if (bitRead(msg.buf[6], 4)) Serial.print("Current sensing\t");
                if (bitRead(msg.buf[6], 6)) Serial.print("No CAN Steer Command\t");
                if (bitRead(msg.buf[6], 7)) Serial.print("Motor stalled\t");

                Serial.println("Kill Autosteer");
                disengageLog("Keya motor fault/error");
                steerSwitch = 1;
                currentState = 1;
                previous = 0;
            }
        }
    }
}

void SteerKeya(int steerSpeed, bool intendToSteer)
{
    if (keyaDetected)
    {
        int16_t actualSpeed;

        if (intendToSteer)
        {
            actualSpeed = steerSpeed * 3.9;
        }
        else
        {
            keyaCommand(keyaDisableCommand);
            actualSpeed = 0;
        }

        keyaCurrentSetSpeed = actualSpeed * 0.1;

        CAN_message_t msg;
        msg.id = KeyaID;
        msg.flags.extended = true;
        msg.len = 8;
        memcpy(msg.buf, keyaSpeedCommand, 4);
        if (steerSpeed < 0)
        {
            msg.buf[4] = highByte(actualSpeed);
            msg.buf[5] = lowByte(actualSpeed);
            msg.buf[6] = 0xff;
            msg.buf[7] = 0xff;
        }
        else
        {
            msg.buf[4] = highByte(actualSpeed);
            msg.buf[5] = lowByte(actualSpeed);
            msg.buf[6] = 0x00;
            msg.buf[7] = 0x00;
        }
        canWrite(CAN_MODE_KEYA, msg);

        if(intendToSteer) keyaCommand(keyaEnableCommand);
    }
}

void keyaCommand(uint8_t command[])
{
    if (keyaDetected)
    {
        CAN_message_t msg;
        msg.id = KeyaID;
        msg.flags.extended = true;
        msg.len = 8;
        memcpy(msg.buf, command, 4);
        canWrite(CAN_MODE_KEYA, msg);
    }
}

// ── CAN loopback test: echo any 0x7E0 frame back as 0x7E1 with data+1 ────────
void CANtest_Receive() {
    CAN_message_t rx, tx;
    while (canRead(CAN_MODE_CANTEST, rx)) {
        canRawPrint(rx, "T");
        if (rx.id == 0x7E0) {
            tx.id            = 0x7E1;
            tx.flags.extended = false;
            tx.len           = rx.len;
            for (uint8_t i = 0; i < rx.len; i++)
                tx.buf[i] = rx.buf[i] + 1;
            canWrite(CAN_MODE_CANTEST, tx);
        }
    }
}
