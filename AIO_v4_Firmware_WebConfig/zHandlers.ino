// Conversion to Hexidecimal
const char* asciiHex = "0123456789ABCDEF";

// the new PANDA sentence buffer
char nmea[100];

// GGA
char fixTime[12];
char latitude[15];
char latNS[3];
char longitude[15];
char lonEW[3];
char fixQuality[2];
char numSats[4];
char HDOP[5];
char altitude[12];
char ageDGPS[10];

// VTG
char vtgHeading[12] = { };
char speedKnots[10] = { };

// IMU
char imuHeading[6];
char imuRoll[6];
char imuPitch[6];
char imuYawRate[6];

// GGA field — geoidal separation (used by J1939 broadcast)
char geoidalSep[12] = { 0 };

// HPR
char solQuality[2];
char umHeading[15];
char umRoll[15];

// If odd characters showed up.
void errorHandler()
{
  //nothing at the moment
}

void GGA_Handler() //Rec'd GGA
{
    // fix time
    parser.getArg(0, fixTime);

    // latitude
    parser.getArg(1, latitude);
    parser.getArg(2, latNS);

    // longitude
    parser.getArg(3, longitude);
    parser.getArg(4, lonEW);

    // fix quality
    parser.getArg(5, fixQuality);
    mainFixQuality = atoi(fixQuality);  // store raw value before any override
    // HPR cutoff: after 10s of lost RTK on second antenna → force invalid GPS → AgIO disengages
    if (hprCutoffActive) itoa(0, fixQuality, 10);

    // satellite #
    parser.getArg(6, numSats);

    // HDOP
    parser.getArg(7, HDOP);

    // altitude
    parser.getArg(8, altitude);

    // geoidal separation
    parser.getArg(10, geoidalSep);

    // time of last DGPS update
    parser.getArg(12, ageDGPS);

    if (blink)
    {
        digitalWrite(GGAReceivedLED, HIGH);
    }
    else
    {
        digitalWrite(GGAReceivedLED, LOW);
    }

    blink = !blink;
    GGAReadyTime = 0;
    { uint32_t _n = millis(); ggaIntervalMs = _n - lastGgaMs; lastGgaMs = _n; }
    if (hasFuncMode(CAN_MODE_J1939)) j1939UpdateFromGGA();

    // ── Source-based routing ──────────────────────────────────────────────────
    bool configuredHPR    = (moduleConfig.headingSource == HDG_SRC_HPR
                          || moduleConfig.rollSource    == ROLL_SRC_HPR);
    bool configuredRELPOS = (moduleConfig.headingSource == HDG_SRC_RELPOS);

    // Timeout fallback: if configured source hasn't sent data in 3s → fall back to IMU
    bool needsHPR    = configuredHPR    && (HPRReadyTime    < 3000);
    bool needsRELPOS = configuredRELPOS && (RELPOSReadyTime < 3000);

    // Warn user in AgIO when configured source is missing (once every 5s)
    if (configuredHPR && !needsHPR) {
        static elapsedMillis hprWarnTimer = 5001;
        if (hprWarnTimer > 5000) {
            sendDisplayMessage("AIO: HPR not received - check GPS $HPR output config", 5, 0);
            hprWarnTimer = 0;
        }
    }
    if (configuredRELPOS && !needsRELPOS) {
        static elapsedMillis relposWarnTimer = 5001;
        if (relposWarnTimer > 5000) {
            sendDisplayMessage("AIO: RELPOS not received - check u-blox F9P config", 5, 0);
            relposWarnTimer = 0;
        }
    }

    if (needsHPR || needsRELPOS) {
        // Defer: wait for HPR sentence or RELPOS packet
        dualReadyGGA = true;
        if (needsHPR && dualReadyHPR) {
            // HPR already arrived before GGA — process now
            imuHandler();
            BuildNmea();
            dualReadyGGA = dualReadyHPR = false;
            useDual = true;
            digitalWrite(GPSGREEN_LED, HIGH);
            digitalWrite(GPSRED_LED,   HIGH);
        }
        return;
    }

    // ── IMU heading/roll path ─────────────────────────────────────────────────
    bool hasIMU = useBNO08xI2C || useBNO08xRVC || useTMxx_IMU;
    if (!hasIMU) {
        // No IMU: 65535 tells AgIO to ignore IMU heading
        itoa(65535, imuHeading, 10);
        itoa(0, imuRoll,    10);
        itoa(0, imuPitch,   10);
        itoa(0, imuYawRate, 10);
        digitalWrite(GPSRED_LED, blink);
        digitalWrite(GPSGREEN_LED, LOW);
    } else {
        if (useBNO08xRVC) { bnoTrigger = true; bnoTimer = 0; }
        digitalWrite(GPSRED_LED, HIGH);
        digitalWrite(GPSGREEN_LED, LOW);
    }
    BuildNmea();
    dualReadyGGA = false;

    if (DBG_GPS) {
        static elapsedMillis gpsDbgTimer = 1001;
        if (gpsDbgTimer > 1000) {
            gpsDbgTimer = 0;
            webLogf("GPS fix:%s sats:%s HDOP:%s alt:%sm", fixQuality, numSats, HDOP, altitude);
        }
    }
}

void VTG_Handler()
{
    // vtg heading
    parser.getArg(0, vtgHeading);
    headingVTG = atof(vtgHeading);

    // vtg Speed knots
    parser.getArg(4, speedKnots);

    if (hasFuncMode(CAN_MODE_J1939)) j1939UpdateFromVTG();
    VTGReadyTime = 0;
    { uint32_t _n = millis(); vtgIntervalMs = _n - lastVtgMs; lastVtgMs = _n; }

    updateGpsMotion();   // headingRate + wheelAngleGPS from course-over-ground (all modes)
}

// Compute vehicle yaw rate and GPS-derived wheel angle from VTG course over ground.
// Works in every heading mode (IMU / HPR / RELPOS) since VTG is always present.
// Used by Keya & IMU-WAS auto-zero and shown in Live tab Gr3 WAS.
void updateGpsMotion()
{
    static float         emaHdg = 0;   // EMA-filtered heading (deg)
    static elapsedMillis dtT    = 0;
    static bool          init   = false;

    // Yaw rate follows the configured HEADING SOURCE (always a heading in degrees —
    // unit-safe). True heading sources (IMU fused, dual-antenna HPR/RELPOS) are valid
    // even stationary; VTG course is only a fallback and needs motion.
    float hdg; bool usingVtg = false;
    if (moduleConfig.headingSource == HDG_SRC_RELPOS) {
        hdg = (float)heading;                                   // RELPOS dual-antenna
    } else if (moduleConfig.headingSource == HDG_SRC_HPR && umHeading[0] != 0) {
        hdg = atof(umHeading);                                  // HPR dual-antenna
    } else if (moduleConfig.headingSource == HDG_SRC_IMU) {
        hdg = useTMxx_IMU ? (float)(TM171_IMU.getYaw() / 100.0) // TM171 (deg)
                          : (yaw / 10.0f);                      // BNO085 (deg x10 -> deg)
    } else {
        hdg = (float)headingVTG; usingVtg = true;               // course-over-ground fallback
    }

    float dt = dtT / 1000.0f;
    dtT = 0;
    if (!init) { emaHdg = hdg; init = true; return; }
    if (dt < 0.02f) dt = 0.02f;        // floor dt (a tiny dt hugely amplifies noise)

    // ── Filter the HEADING first, THEN differentiate (cleaner than filtering the
    // rate: differentiating a smoothed signal avoids amplifying glitches). The EMA
    // step itself is the change of the filtered heading this sample → divide by dt.
    float dRaw = hdg - emaHdg;
    if (dRaw > 180)  dRaw -= 360;
    if (dRaw < -180) dRaw += 360;
    float a = moduleConfig.yawRateFilter;            // heading EMA factor (0 = off)
    float step;
    if (a > 0.0f && a < 1.0f) { step = a * dRaw; emaHdg += step; }
    else                      { step = dRaw;     emaHdg  = hdg; }   // filter off = raw
    if (emaHdg < 0.0f)    emaHdg += 360.0f;
    if (emaHdg >= 360.0f) emaHdg -= 360.0f;

    // VTG fallback needs movement; a real heading source is valid at any speed.
    if (!usingVtg || gpsSpeed > 1.0f) {
        float rate = step / dt;                      // deg/s from the smoothed heading
        if (rate >  90.0f) rate =  90.0f;            // clamp absurd spikes
        if (rate < -90.0f) rate = -90.0f;
        headingRate = rate;
    } else {
        headingRate = 0;
    }

    // Bicycle-model wheel angle from yaw rate + speed (needs speed to be meaningful).
    if (gpsSpeed > 1.0f) {
        double ms = gpsSpeed * 0.27778;
        wheelAngleGPS = atan(headingRate / RAD_TO_DEG * moduleConfig.wheelBase / ms) * RAD_TO_DEG;
        if (!(wheelAngleGPS < 50 && wheelAngleGPS > -50)) wheelAngleGPS = steerAngleActual;
    } else {
        wheelAngleGPS = 0;
    }
}

void HPR_Handler()
{
    HPRReadyTime = 0;
    { uint32_t _n = millis(); hprIntervalMs = _n - lastHprMs; lastHprMs = _n; }
    parser.getArg(1, umHeading);
    parser.getArg(2, umRoll);     rollDual      = atof(umRoll);
    parser.getArg(4, solQuality); solQualityHPR = atoi(solQuality);

    // ── RTK quality guard for heading (UM982 second antenna) ─────────────────
    // Condition: main antenna has RTK (4) but secondary antenna doesn't
    // If main is not RTK either, AgIO handles it normally — no intervention needed
    if (moduleConfig.headingSource == HDG_SRC_HPR) {
        bool mainIsRTK      = (mainFixQuality == 4);
        bool secondaryIsRTK = (solQualityHPR  == 4);

        if (secondaryIsRTK || !mainIsRTK) {
            // Both OK, or main also degraded (AgIO handles that) — update heading normally
            heading = atof(umHeading);
            if (hprRtkLost) {
                hprRtkLost      = false;
                hprCutoffActive = false;
                hprLostTimer    = 0;
                sendDisplayMessage("Heading antenna: RTK fix restored", 5, 1);
                webLog("HPR RTK restored");
            }
        } else {
            // Main=RTK but secondary lost RTK → dangerous: freeze heading
            if (!hprRtkLost) {
                hprRtkLost   = true;
                hprLostTimer = 0;
            }
            // heading intentionally NOT updated — keeps last good value
        }
    } else {
        heading = atof(umHeading);  // no quality guard for other sources
    }

    bool needsHPR = (moduleConfig.headingSource == HDG_SRC_HPR
                  || moduleConfig.rollSource    == ROLL_SRC_HPR);
    if (!needsHPR) return;  // not using HPR source, just store data

    useDual = true;
    if (dualReadyGGA) {
        // GGA already arrived — process immediately
        imuHandler();
        BuildNmea();
        dualReadyGGA = dualReadyHPR = false;
        digitalWrite(GPSGREEN_LED, HIGH);
        digitalWrite(GPSRED_LED,   HIGH);
    } else {
        // GGA hasn't arrived yet — mark HPR ready, GGA_Handler will trigger
        dualReadyHPR = true;
    }
}

void imuHandler()
{
    // Write heading string only when source is not IMU (IMU sets imuHeading in loop())
    if (moduleConfig.headingSource != HDG_SRC_IMU)
        dtostrf(heading, 4, 2, imuHeading);

    // Write roll string only when source is not IMU (IMU sets imuRoll in loop())
    if (moduleConfig.rollSource != ROLL_SRC_IMU)
        dtostrf(rollDual, 4, 2, imuRoll);

    // NMEA yaw rate field: dual → use GPS heading rate; IMU-RVC → gyro angVel
    if (moduleConfig.headingSource != HDG_SRC_IMU) {
        int16_t yawRatex10 = (int16_t)(headingRate * 10);
        itoa(yawRatex10, imuYawRate, 10);
    } else if (useBNO08xRVC) {
        itoa(bnoData.angVel, imuYawRate, 10);
    }
    // headingRate + wheelAngleGPS are computed in updateGpsMotion() (VTG_Handler)
}

void readTM171()
{
    TM171_IMU.readAngles();

    // Populate NMEA strings (respect X/Y axis swap, same as BNO path)
    if (steerConfig.IsUseY_Axis) {
        strcpy(imuRoll,  TM171_IMU.getPitchStr());
        strcpy(imuPitch, TM171_IMU.getRollStr());
    } else {
        strcpy(imuPitch, TM171_IMU.getPitchStr());
        strcpy(imuRoll,  TM171_IMU.getRollStr());
    }
    strcpy(imuHeading, TM171_IMU.getYawStr());

    // Yaw rate from successive yaw readings (TM171 sends no gyro/angular velocity)
    static double        tmPrevYaw = 0;
    static elapsedMillis tmYrTimer = 0;
    static bool          tmYrInit  = false;
    float  tdt = tmYrTimer / 1000.0f; tmYrTimer = 0;
    double curY = TM171_IMU.getYaw() / 100.0;   // getYaw() is deg×100
    double tyd = curY - tmPrevYaw;
    tmPrevYaw = curY;
    if (tyd > 180)  tyd -= 360;
    if (tyd < -180) tyd += 360;
    int16_t tyr = (tmYrInit && tdt > 0.005f) ? (int16_t)((tyd / tdt) * 10.0) : 0;
    tmYrInit = true;
    itoa(tyr, imuYawRate, 10);
}

void readBNO_RVC()
{
    if (rvc.read(&bnoData)) {
        useBNO08xRVC = true;
        yaw   = bnoData.yawX10;
        roll  = bnoData.rollX10;
        pitch = bnoData.pitchX10;
        itoa(yaw,            imuHeading, 10);
        itoa(pitch,          imuPitch,   10);
        itoa(roll,           imuRoll,    10);
        itoa(bnoData.angVel, imuYawRate, 10);
    }
}

void readBNO()
{
    if (bno08x.dataAvailable() == true)
    {
        float dqx, dqy, dqz, dqw, dacr;
        uint8_t dac;

        //get quaternion
        bno08x.getQuat(dqx, dqy, dqz, dqw, dacr, dac);
        float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
        dqw = dqw / norm;
        dqx = dqx / norm;
        dqy = dqy / norm;
        dqz = dqz / norm;

        float ysqr = dqy * dqy;

        // yaw (z-axis rotation)
        float t3 = +2.0 * (dqw * dqz + dqx * dqy);
        float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
        yaw = atan2(t3, t4);

        // Convert yaw to degrees x10
        yaw = (int16_t)((yaw * -RAD_TO_DEG_X_10));
        if (yaw < 0) yaw += 3600;

        // pitch (y-axis rotation)
        float t2 = +2.0 * (dqw * dqy - dqz * dqx);
        t2 = t2 > 1.0 ? 1.0 : t2;
        t2 = t2 < -1.0 ? -1.0 : t2;

        // roll (x-axis rotation)
        float t0 = +2.0 * (dqw * dqx + dqy * dqz);
        float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);

        if (steerConfig.IsUseY_Axis)
        {
            roll = asin(t2) * RAD_TO_DEG_X_10;
            pitch = atan2(t0, t1) * RAD_TO_DEG_X_10;
        }
        else
        {
            pitch = asin(t2) * RAD_TO_DEG_X_10;
            roll = atan2(t0, t1) * RAD_TO_DEG_X_10;
        }

        itoa(yaw, imuHeading, 10);
        itoa(pitch, imuPitch, 10);
        itoa(roll, imuRoll, 10);

        // Yaw rate from successive yaw readings (I2C BNO has no gyro report)
        static double        prevYawDeg = 0;
        static elapsedMillis yrTimer    = 0;
        static bool          yrInit     = false;
        float  ydt = yrTimer / 1000.0f; yrTimer = 0;
        double curYawDeg = yaw / 10.0;            // yaw is deg×10 (0..3600)
        double yd = curYawDeg - prevYawDeg;
        prevYawDeg = curYawDeg;
        if (yd > 180)  yd -= 360;
        if (yd < -180) yd += 360;
        int16_t yrx10 = (yrInit && ydt > 0.005f) ? (int16_t)((yd / ydt) * 10.0) : 0;
        yrInit = true;
        itoa(yrx10, imuYawRate, 10);
    }
}

void BuildNmea(void)
{
    strcpy(nmea, "");

    if (moduleConfig.nmeaType == NMEA_TYPE_PAOGI) strcat(nmea, "$PAOGI,");
    else                                           strcat(nmea, "$PANDA,");

    strcat(nmea, fixTime);
    strcat(nmea, ",");

    strcat(nmea, latitude);
    strcat(nmea, ",");

    strcat(nmea, latNS);
    strcat(nmea, ",");

    strcat(nmea, longitude);
    strcat(nmea, ",");

    strcat(nmea, lonEW);
    strcat(nmea, ",");

    // 6
    strcat(nmea, fixQuality);
    strcat(nmea, ",");

    strcat(nmea, numSats);
    strcat(nmea, ",");

    strcat(nmea, HDOP);
    strcat(nmea, ",");

    strcat(nmea, altitude);
    strcat(nmea, ",");

    //10
    strcat(nmea, ageDGPS);
    strcat(nmea, ",");

    //11
    strcat(nmea, speedKnots);
    strcat(nmea, ",");

    //12
    strcat(nmea, imuHeading);
    strcat(nmea, ",");

    //13
    strcat(nmea, imuRoll);
    strcat(nmea, ",");

    //14
    strcat(nmea, imuPitch);
    strcat(nmea, ",");

    //15
    strcat(nmea, imuYawRate);

    strcat(nmea, "*");

    CalculateChecksum();

    strcat(nmea, "\r\n");

    //SerialAOG.write(nmea);  //Always send USB GPS data

    int len = strlen(nmea);
    Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
    Eth_udpPAOGI.write(nmea, len);
    Eth_udpPAOGI.endPacket();
}

void CalculateChecksum(void)
{
  int16_t sum = 0;
  int16_t inx = 0;
  char tmp;

  // The checksum calc starts after '$' and ends before '*'
  for (inx = 1; inx < 200; inx++)
  {
    tmp = nmea[inx];

    // * Indicates end of data and start of checksum
    if (tmp == '*')
    {
      break;
    }

    sum ^= tmp;    // Build checksum
  }

  byte chk = (sum >> 4);
  char hex[2] = { asciiHex[chk], 0 };
  strcat(nmea, hex);

  chk = (sum % 16);
  char hex2[2] = { asciiHex[chk], 0 };
  strcat(nmea, hex2);
}

/*
  $PANDA
  (1) Time of fix

  position
  (2,3) 4807.038,N Latitude 48 deg 07.038' N
  (4,5) 01131.000,E Longitude 11 deg 31.000' E

  (6) 1 Fix quality:
    0 = invalid
    1 = GPS fix(SPS)
    2 = DGPS fix
    3 = PPS fix
    4 = Real Time Kinematic
    5 = Float RTK
    6 = estimated(dead reckoning)(2.3 feature)
    7 = Manual input mode
    8 = Simulation mode
  (7) Number of satellites being tracked
  (8) 0.9 Horizontal dilution of position
  (9) 545.4 Altitude (ALWAYS in Meters, above mean sea level)
  (10) 1.2 time in seconds since last DGPS update
  (11) Speed in knots

  FROM IMU:
  (12) Heading in degrees
  (13) Roll angle in degrees(positive roll = right leaning - right down, left up)

  (14) Pitch angle in degrees(Positive pitch = nose up)
  (15) Yaw Rate in Degrees / second

  CHKSUM
*/

/*
  $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M ,  ,*47
   0     1      2      3    4      5 6  7  8   9    10 11  12 13  14
        Time      Lat       Lon     FixSatsOP Alt
  Where:
     GGA          Global Positioning System Fix Data
     123519       Fix taken at 12:35:19 UTC
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     1            Fix quality: 0 = invalid
                               1 = GPS fix (SPS)
                               2 = DGPS fix
                               3 = PPS fix
                               4 = Real Time Kinematic
                               5 = Float RTK
                               6 = estimated (dead reckoning) (2.3 feature)
                               7 = Manual input mode
                               8 = Simulation mode
     08           Number of satellites being tracked
     0.9          Horizontal dilution of position
     545.4,M      Altitude, Meters, above mean sea level
     46.9,M       Height of geoid (mean sea level) above WGS84
                      ellipsoid
     (empty field) time in seconds since last DGPS update
     (empty field) DGPS station ID number
      47          the checksum data, always begins with


  $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  0      1    2   3      4    5      6   7     8     9     10   11
        Time      Lat        Lon       knots  Ang   Date  MagV

  Where:
     RMC          Recommended Minimum sentence C
     123519       Fix taken at 12:35:19 UTC
     A            Status A=active or V=Void.
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     022.4        Speed over the ground in knots
     084.4        Track angle in degrees True
     230394       Date - 23rd of March 1994
     003.1,W      Magnetic Variation
      6A          The checksum data, always begins with

  $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48

    VTG          Track made good and ground speed
    054.7,T      True track made good (degrees)
    034.4,M      Magnetic track made good
    005.5,N      Ground speed, knots
    010.2,K      Ground speed, Kilometers per hour
     48          Checksum
*/
