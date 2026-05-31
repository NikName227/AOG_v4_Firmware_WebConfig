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
    if (hasFuncMode(CAN_MODE_J1939)) j1939UpdateFromGGA();

    // ── Source-based routing ──────────────────────────────────────────────────
    bool needsHPR    = (moduleConfig.headingSource == HDG_SRC_HPR
                     || moduleConfig.rollSource    == ROLL_SRC_HPR);
    bool needsRELPOS = (moduleConfig.headingSource == HDG_SRC_RELPOS);

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
}

void HPR_Handler()
{
    HPRReadyTime = 0;
    parser.getArg(1, umHeading);  heading       = atof(umHeading);
    parser.getArg(2, umRoll);     rollDual      = atof(umRoll);
    parser.getArg(4, solQuality); solQualityHPR = atoi(solQuality);

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

    // Always compute heading rate, working direction, wheel angle from heading
    if (abs((int)(headingVTG - heading) % 360) > 120 && gpsSpeed > 0.5)
        workingDir = -1;
    else
        workingDir = 1;

    static double headingOld = heading;
    headingRate = (heading - headingOld) * GPS_Hz;
    headingOld = heading;
    if (headingRate > 360)  headingRate -= 360;
    if (headingRate < -360) headingRate += 360;

    // Write yaw rate only when source is not IMU (IMU already sets imuYawRate)
    if (moduleConfig.headingSource != HDG_SRC_IMU) {
        int16_t yawRatex10 = (int16_t)(headingRate * 10);
        itoa(yawRatex10, imuYawRate, 10);
    } else if (useBNO08xRVC) {
        // BNO RVC provides angular velocity
        int16_t yawRatex10 = bnoData.angVel;
        itoa(yawRatex10, imuYawRate, 10);
    }

    double ms = gpsSpeed * 0.27778;
    if (gpsSpeed > 1) {
        wheelAngleGPS = atan(headingRate / RAD_TO_DEG * wheelBase / ms) * RAD_TO_DEG * workingDir;
        if (!(wheelAngleGPS < 50 && wheelAngleGPS > -50)) wheelAngleGPS = steerAngleActual;
    }
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
        itoa(0, imuYawRate, 10);
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
