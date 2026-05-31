/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/

////////////////// User Settings /////////////////////////

//How many degrees before decreasing Max PWM
#define LOW_HIGH_DEGREES 1.5

/*  PWM Frequency ->
     490hz (default) = 0
     122hz = 1
     3921hz = 2
*/
#define PWM_Frequency 1

//WAS Calabration
float inputWAS[] =  { -50.00, -45.0, -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0 };  //Input WAS do not adjust
float outputWAS[] = { -50.00, -45.0, -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0 };

/////////////////////////////////////////////

// if not in eeprom, overwrite
#define EEP_Ident 2400

//   ***********  Motor drive connections  **************888
//Connect ground only for cytron, Connect Ground and +5v for IBT2

//Dir1 for Cytron Dir, Both L and R enable for IBT2
#define DIR1_RL_ENABLE  4

//PWM1 for Cytron PWM, Left PWM for IBT2
#define PWM1_LPWM  2

//Not Connected for Cytron, Right PWM for IBT2
#define PWM2_RPWM  3

//--------------------------- Switch Input Pins ------------------------
#define STEERSW_PIN 32
#define WORKSW_PIN 34
#define REMOTE_PIN 37

//Define sensor pin for current or pressure sensor
#define CURRENT_SENSOR_PIN A17
#define PRESSURE_SENSOR_PIN A10

#define CONST_180_DIVIDED_BY_PI 57.2957795130823

#include <Wire.h>
#include <EEPROM.h>
#include "zADS1115.h"
ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);     // Use this for the 16-bit version ADS1115

#include <IPAddress.h>

// ethernet
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

//uint8_t Ethernet::buffer[200]; // udp send and receive buffer
uint8_t autoSteerUdpData[serial_buffer_size];  // Buffer For Receiving UDP Data

//loop time variables in microseconds
const uint16_t LOOP_TIME = 25;  //40Hz
uint32_t autsteerLastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

const uint16_t WATCHDOG_THRESHOLD = 100;
const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;

//Heart beat hello AgIO
uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
uint8_t helloFromAutoSteer[] = { 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71 };
int16_t helloSteerPosition = 0;

//fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
uint8_t PGN_253[] = {0x80,0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
int8_t PGN_253_Size = sizeof(PGN_253) - 1;

//fromAutoSteerData FD 250 - sensor values etc
uint8_t PGN_250[] = { 0x80,0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
int8_t PGN_250_Size = sizeof(PGN_250) - 1;
uint8_t aog2Count = 0;
float sensorReading;
float sensorSample;

elapsedMillis gpsSpeedUpdateTimer = 0;

//EEPROM
int16_t EEread = 0;

//Relays
bool isRelayActiveHigh = true;
uint8_t relay = 0, relayHi = 0, uTurn = 0;
uint8_t tram = 0;

//Switches
uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

//On Off
uint8_t guidanceStatus = 0;
uint8_t prevGuidanceStatus = 0;
bool guidanceStatusChanged = false;
elapsedMillis motorHandDetectionTime;

//speed sent as *10
float gpsSpeed = 0;

//steering variables
elapsedMillis steerSensorReadTime;
float steerAngleActual = 0;
float steerAngleActual_previous = 0;
float steerAngleSpeedActual = 0;
float steerAngleSetPoint = 0; //the desired angle from AgOpen
int16_t steeringPosition = 0; //from steering sensor
float steerAngleError = 0; //setpoint - actual

//pwm variables
int16_t pwmDrive = 0, pwmDisplay = 0;
float pValue = 0;
float pValue_previous = 0;
float errorAbs = 0;
float highLowPerDeg = 0;

//Steer switch button  ***********************************************************************************************************
uint8_t currentState = 1, reading, previous = 0;
uint8_t pulseCount = 0; // Steering Wheel Encoder
bool encEnable = false; //debounce flag
uint8_t thisEnc = 0, lastEnc = 0;

//Variables for settings
struct Storage {
  uint8_t Kp = 40;              // proportional gain
  uint8_t lowPWM = 10;          // band of no action
  int16_t wasOffset = 0;
  uint8_t minPWM = 9;
  uint8_t highPWM = 60;         // max PWM value
  float steerSensorCounts = 30;
  float AckermanFix = 1;        // sent as percent
};  Storage steerSettings;      // 11 bytes

//Variables for settings - 0 is false
struct Setup {
  uint8_t InvertWAS = 0;
  uint8_t IsRelayActiveHigh = 0;    // if zero, active low (default)
  uint8_t MotorDriveDirection = 0;
  uint8_t SingleInputWAS = 1;
  uint8_t CytronDriver = 1;
  uint8_t SteerSwitch = 0;          // 1 if switch selected
  uint8_t SteerButton = 0;          // 1 if button selected
  uint8_t ShaftEncoder = 0;
  uint8_t PressureSensor = 0;
  uint8_t CurrentSensor = 0;
  uint8_t PulseCountMax = 5;
  uint8_t IsDanfoss = 0;
  uint8_t IsUseY_Axis = 0;     //Set to 0 to use X Axis, 1 to use Y avis
}; Setup steerConfig;               // 9 bytes

void steerConfigInit()
{
  if (steerConfig.CytronDriver) 
  {
    pinMode(PWM2_RPWM, OUTPUT);
  }
}

void steerSettingsInit()
{
  // for PWM High to Low interpolator
  highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
}

void autosteerSetup()
{
  //PWM rate settings. Set them both the same!!!!
  /*  PWM Frequency ->
       490hz (default) = 0
       122hz = 1
       3921hz = 2
  */
  if (PWM_Frequency == 0)
  {
    analogWriteFrequency(PWM1_LPWM, 490);
    analogWriteFrequency(PWM2_RPWM, 490);
  }
  else if (PWM_Frequency == 1)
  {
    analogWriteFrequency(PWM1_LPWM, 122);
    analogWriteFrequency(PWM2_RPWM, 122);
  }
  else if (PWM_Frequency == 2)
  {
    analogWriteFrequency(PWM1_LPWM, 3921);
    analogWriteFrequency(PWM2_RPWM, 3921);
  }

  //keep pulled high and drag low to activate, noise free safe
  pinMode(WORKSW_PIN, INPUT_PULLUP);
  pinMode(STEERSW_PIN, INPUT_PULLUP);
  pinMode(REMOTE_PIN, INPUT_PULLUP);
  pinMode(DIR1_RL_ENABLE, OUTPUT);

  // Disable digital inputs for analog input pins
  pinMode(CURRENT_SENSOR_PIN, INPUT_DISABLE);
  pinMode(PRESSURE_SENSOR_PIN, INPUT_DISABLE);

  //set up communication
  Wire1.end();
  Wire1.begin();
    
  // Check ADC 
  if(adc.testConnection())
  {
    Serial.println("ADC Connecton OK");
  }
  else
  {
    Serial.println("ADC Connecton FAILED!");
    Autosteer_running = false;
  }

  //50Khz I2C
  //TWBR = 144;   //Is this needed?

  EEPROM.get(0, EEread);              // read identifier

  if (EEread != EEP_Ident)            // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(10, steerSettings);
    EEPROM.put(40, steerConfig);
    EEPROM.put(60, networkAddress);    
  }
  else
  {
    EEPROM.get(10, steerSettings);     // read the Settings
    EEPROM.get(40, steerConfig);
    EEPROM.get(60, networkAddress); 
  }

  steerSettingsInit();
  steerConfigInit();

  if (Autosteer_running) 
  {
    Serial.println("Autosteer running, waiting for AgOpenGPS");
    // Autosteer Led goes Red if ADS1115 is found
    digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
    digitalWrite(AUTOSTEER_STANDBY_LED, 1);
  }
  else
  {
    Autosteer_running = false;  //Turn off auto steer if no ethernet (Maybe running T4.0)
//    if(!Ethernet_running)Serial.println("Ethernet not available");
    Serial.println("Autosteer disabled, GPS only mode");   
    return;
  }

  adc.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS); //128 samples per second
  adc.setGain(ADS1115_REG_CONFIG_PGA_6_144V);

}// End of Setup

void autosteerLoop()
{

  ReceiveUdp();
  KeyaBus_Receive();
  if (hasFuncMode(CAN_MODE_VBUS)) {
      VBus_Receive();
      ISO_Receive();
      KBus_Receive();
  }

  // Loop triggers every 100 msec and sends back gyro heading, and roll, steer angle etc
  currentTime = systick_millis_count;

  if (currentTime - autsteerLastTime >= LOOP_TIME)
  {
    autsteerLastTime = currentTime;

    //reset debounce
    encEnable = true;

    //If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
    if (watchdogTimer++ > 250)
    {
        watchdogTimer = WATCHDOG_FORCE_VALUE;
        steerSwitch = 1; // reset values like it turned off
        currentState = 1;
        gpsSpeed = 0;
    }

    //read all the switches
    workSwitch = digitalRead(WORKSW_PIN);       // read work switch

    //Engage steering via 1 PCB Button or 2 Tablet

    // 1 PCB Button pressed?
    reading = digitalRead(STEERSW_PIN);

    if (steerConfig.SteerSwitch == 1)
    {
        // Switch is off so reset ready for next switch on
        if (reading == HIGH)
        {
            currentState = 1;
            steerSwitch = 1;
            previous = reading;
        }
    }

    // 2 Has tablet button been pressed?
    if (guidanceStatusChanged)
    {
        if (guidanceStatus == 1)    //Must have changed Off >> On
        {
            currentState = 0;
            steerSwitch = 0;
        }
    }

    // If AOG has stopped steering, wait then turn off steerswitch ready for next engage.
    static int switchCounter = 0;

    if (steerSwitch == 0 && guidanceStatus == 0)
    {
        if (switchCounter++ > 30)
        {
            currentState = 1;
            steerSwitch = 1;
        }
    }
    else
    {
        switchCounter = 0;
    }

    // Arduino software button code
    if (reading == LOW && previous == HIGH)
    {
        if (currentState == 1)
        {
            currentState = 0;
            steerSwitch = 0;
        }
        else
        {
            currentState = 1;
            steerSwitch = 1;
        }
    }
    previous = reading;

    // CAN engage from tractor bus (toggle steerSwitch like tablet button)
    if (engageCAN) {
        engageCAN = false;
        if (currentState == 1) {
            currentState = 0;
            steerSwitch = 0;
        } else {
            currentState = 1;
            steerSwitch = 1;
        }
    }

    // Encoder sensor?
    if (steerConfig.ShaftEncoder && pulseCount >= steerConfig.PulseCountMax)
    {
      steerSwitch = 1; 
      currentState = 1;
      previous = 0;
    }

    // Pressure sensor?
    if (steerConfig.PressureSensor)
    {
      sensorSample = (float)analogRead(PRESSURE_SENSOR_PIN);
      sensorSample *= 0.25;
      sensorReading = sensorReading * 0.6 + sensorSample * 0.4;
      if (sensorReading >= steerConfig.PulseCountMax)
      {
          steerSwitch = 1; 
          currentState = 1;
          previous = 0;
      }
    }

    // Current sensor?
    if (steerConfig.CurrentSensor)
    {
        if (keyaDetected)
        {
            if (sensorReading >= steerConfig.PulseCountMax)
            {
                steerSwitch = 1;
                currentState = 1;
                previous = 0;
            }
        }
        else
        {

            // NV speed-direction disengage (configurable via web)
            if (moduleConfig.motorDisEnable
                && abs(steerAngleError) > moduleConfig.motorAngleErrorMin
                && steerSwitch == 0)
            {
                if ((steerAngleSpeedActual * steerAngleError) < 0) {
                    if (motorHandDetectionTime > moduleConfig.speedDiffTimeout) {
                        Serial.println("Kill Autosteer PWM from speed direction");
                        webLog("Motor: speed direction cutoff");
                        steerSwitch = 1;
                        currentState = 1;
                        previous = 0;
                    }
                } else {
                    motorHandDetectionTime = 0;
                }
            }

            if (DBG_MOTOR_DIFF) {
                static uint32_t prevMotorDbg = 0;
                if (millis() - prevMotorDbg >= 500) {
                    prevMotorDbg = millis();
                    Serial.print("ActSped: ");   Serial.print(steerAngleSpeedActual);
                    Serial.print(" AngleErr: "); Serial.print(steerAngleError);
                    Serial.print(" param: ");    Serial.println(moduleConfig.speedDiffTimeout);
                    webLogf("Motor ActSped:%.2f AngleErr:%.2f timeout:%u ms",
                        steerAngleSpeedActual, steerAngleError,
                        moduleConfig.speedDiffTimeout);
                }
            }
        
            /*sensorSample = (float)analogRead(CURRENT_SENSOR_PIN);
            sensorSample = (abs(775 - sensorSample)) * 0.5;
            sensorReading = sensorReading * 0.7 + sensorSample * 0.3;
            sensorReading = min(sensorReading, 255);

            if (sensorReading >= steerConfig.PulseCountMax)
            {
                steerSwitch = 1;
                currentState = 1;
                previous = 0;
            }*/
        }
    }

    remoteSwitch = digitalRead(REMOTE_PIN);
    switchByte = 0;
    switchByte |= (remoteSwitch << 2);  //put remote in bit 2
    switchByte |= (steerSwitch << 1);   //put steerswitch status in bit 1 position
    switchByte |= workSwitch;

    // Initial auto-zero for Keya encoder — runs before autosteer is allowed
    if (moduleConfig.wasSource == WAS_SOURCE_KEYA
        && keyaDetected && keyaEncInitDone && !keyaInitialZeroDone
        && moduleConfig.keyaAzEnable && gpsSpeed >= moduleConfig.keyaAzSpeedMin)
    {
        static elapsedMillis keyaInitTimer = 0;
        if ((float)abs(headingRate) <= moduleConfig.keyaAzYawMax) {
            if (keyaInitTimer > (float)moduleConfig.keyaAzTimeSlowMs) {
                // Fast direct offset to GPS wheel angle (Flodu model)
                int32_t deltaTicks = keyaEncoderRaw - moduleConfig.keyaZeroTicks;
                float rawAngle = (moduleConfig.keyaEncInvert ? -(float)deltaTicks : (float)deltaTicks)
                                 / moduleConfig.keyaTicksPerDeg;
                // Store initial offset as keyaZeroTicks shift
                moduleConfig.keyaZeroTicks = keyaEncoderRaw - (int32_t)(wheelAngleGPS * moduleConfig.keyaTicksPerDeg);
                keyaInitialZeroDone = true;
                webLog("Keya WAS: initial zero done — autosteer unlocked");
                Serial.println("Keya WAS: initial zero done");
            }
        } else {
            keyaInitTimer = 0;
        }
    }

    switch (moduleConfig.wasSource)
    {
    case WAS_SOURCE_IMU_CAN:
    {
        if (!imuWasReceived || imuWasTimeout > 500) break;

        static float imuWasZero      = 0.0f;
        static float imuWasGpsOffset = 0.0f;

        if (imuWasZeroRequest) {
            imuWasZeroRequest = false;
            imuWasZero        = imuWasRawYaw;
            imuWasGpsOffset   = 0.0f;
            Serial.println("IMU WAS: zero set");
            webLog("IMU WAS: zero set");
        }

        float rawAngle = imuWasRawYaw - imuWasZero;
        if (moduleConfig.imuWasInvert) rawAngle = -rawAngle;
        rawAngle *= moduleConfig.imuWasCpdScale;

        if (moduleConfig.imuWasAzEnable
            && gpsSpeed >= moduleConfig.imuWasSpeedMin
            && gpsSpeed > 1.0f)
        {
            if ((float)abs(headingRate) <= moduleConfig.imuWasYawMax) {
                float wasDiff = (rawAngle + imuWasGpsOffset) - wheelAngleGPS;
                imuWasGpsOffset -= wasDiff * moduleConfig.imuWasAzBeta;
            }
        }

        steerAngleActual = rawAngle + imuWasGpsOffset;
        steerAngleSpeedActual = steerAngleSpeedActual * 0.6f
                              + (steerAngleActual - steerAngleActual_previous) * 0.4f
                              / (steerSensorReadTime / 1000.0f);
        steerSensorReadTime = 0;
        steerAngleActual_previous = steerAngleActual;
        break;
    }
    case WAS_SOURCE_CAN_VALVE:
    {
        float rawAngle;
        if (moduleConfig.steerBrand == 3 || moduleConfig.steerBrand == 5) {
            rawAngle = (float)FendtEstCurve / (steerSettings.steerSensorCounts * 10.0f);
        } else {
            int16_t rawPos = (int16_t)(estCurve - 32128) + steerSettings.wasOffset;
            rawAngle = (float)rawPos / steerSettings.steerSensorCounts;
        }
        if (steerConfig.InvertWAS) rawAngle = -rawAngle;
        steerAngleActual = rawAngle;
        steerAngleSpeedActual = steerAngleSpeedActual * 0.6f
                              + (steerAngleActual - steerAngleActual_previous) * 0.4f
                              / (steerSensorReadTime / 1000.0f);
        steerSensorReadTime = 0;
        steerAngleActual_previous = steerAngleActual;
        break;
    }
    case WAS_SOURCE_KEYA:
    {
        if (!keyaDetected) break;

        // Block autosteer until initial auto-zero is done (Flodu model)
        if (!keyaInitialZeroDone) {
            watchdogTimer = WATCHDOG_FORCE_VALUE;
            steerAngleActual = 0;
            break;
        }

        static float keyaGpsOffset = 0.0f;
        static elapsedMillis keyaStraightTimer = 0;

        // Cumulative int32 encoder — invert then subtract zero
        int32_t deltaTicks = keyaEncoderRaw - moduleConfig.keyaZeroTicks;
        float rawAngle = (moduleConfig.keyaEncInvert ? -(float)deltaTicks : (float)deltaTicks)
                         / moduleConfig.keyaTicksPerDeg;

        if (moduleConfig.keyaAzEnable && gpsSpeed >= moduleConfig.keyaAzSpeedMin)
        {
            bool isStable = ((float)abs(headingRate) <= moduleConfig.keyaAzYawMax);

            // Linear time interpolation (Flodu model)
            float azTimeMs;
            if (gpsSpeed <= (float)moduleConfig.keyaAzSpeedSlow)
                azTimeMs = (float)moduleConfig.keyaAzTimeSlowMs;
            else if (gpsSpeed >= moduleConfig.keyaAzSpeedFast)
                azTimeMs = (float)moduleConfig.keyaAzTimeFastMs;
            else {
                float t = (gpsSpeed - (float)moduleConfig.keyaAzSpeedSlow)
                        / (moduleConfig.keyaAzSpeedFast - (float)moduleConfig.keyaAzSpeedSlow);
                azTimeMs = (float)moduleConfig.keyaAzTimeSlowMs
                         + t * ((float)moduleConfig.keyaAzTimeFastMs - (float)moduleConfig.keyaAzTimeSlowMs);
            }

            if (isStable) {
                if (keyaStraightTimer > azTimeMs) {
                    // Guidance-aware: slower correction when autosteer active (Flodu model)
                    float beta = (watchdogTimer < WATCHDOG_THRESHOLD)
                                 ? moduleConfig.keyaAzBeta * 0.2f
                                 : moduleConfig.keyaAzBeta;
                    float wasDiff = (rawAngle + keyaGpsOffset) - wheelAngleGPS;
                    keyaGpsOffset -= wasDiff * beta;
                }
            } else {
                keyaStraightTimer = 0;
            }
        }

        float correctedAngle = rawAngle + keyaGpsOffset;
        static float keyaEmaOut = 0.0f;
        if (moduleConfig.keyaEmaAlpha > 0.0f) {
            keyaEmaOut = moduleConfig.keyaEmaAlpha * correctedAngle + (1.0f - moduleConfig.keyaEmaAlpha) * keyaEmaOut;
            steerAngleActual = keyaEmaOut;
        } else {
            steerAngleActual = correctedAngle;
            keyaEmaOut = correctedAngle;
        }
        steerAngleSpeedActual = steerAngleSpeedActual * 0.6f
                              + (steerAngleActual - steerAngleActual_previous) * 0.4f
                              / (steerSensorReadTime / 1000.0f);
        steerSensorReadTime = 0;
        steerAngleActual_previous = steerAngleActual;
        break;
    }
    default: // WAS_SOURCE_ADS1115
    {
        if (steerConfig.SingleInputWAS)
        {
            adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);
            steeringPosition = adc.getConversion();
            adc.triggerConversion();
            steeringPosition = (steeringPosition >> 1);
            helloSteerPosition = steeringPosition - 6800;
        }
        else
        {
            adc.setMux(ADS1115_REG_CONFIG_MUX_DIFF_0_1);
            steeringPosition = adc.getConversion();
            adc.triggerConversion();
            steeringPosition = (steeringPosition >> 1);
            helloSteerPosition = steeringPosition - 6800;
        }

        if (steerConfig.InvertWAS)
        {
            steeringPosition = (steeringPosition - 6805 - steerSettings.wasOffset);
            steerAngleActual = (float)(steeringPosition) / -steerSettings.steerSensorCounts;
        }
        else
        {
            steeringPosition = (steeringPosition - 6805 + steerSettings.wasOffset);
            steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
        }
        steerAngleSpeedActual = steerAngleSpeedActual * 0.6 + (steerAngleActual-steerAngleActual_previous)*0.4 / (steerSensorReadTime/1000);
        steerSensorReadTime = 0;
        steerAngleActual_previous = steerAngleActual;
        break;
    }
    } // end switch wasSource

    //Ackerman fix
    if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);

    //WAS fault or over 25km, cut steering
    if ((steerAngleActual < inputWAS[0]) || (steerAngleActual > inputWAS[20]) || gpsSpeed > 25)
    {
        steerSwitch = 1; // reset values like it turned off
        currentState = 1;
        previous = 0;
        watchdogTimer = WATCHDOG_FORCE_VALUE;
    }

    //Map WAS
    float mappedWAS = multiMap<float>(steerAngleActual, inputWAS, outputWAS, 21);
    steerAngleActual = mappedWAS;

    if (watchdogTimer < WATCHDOG_THRESHOLD)
    {
      //Enable H Bridge for IBT2, hyd aux, etc for cytron
      if (steerConfig.CytronDriver)
      {
        if (steerConfig.IsRelayActiveHigh)
        {
          digitalWrite(PWM2_RPWM, 0);
        }
        else
        {
          digitalWrite(PWM2_RPWM, 1);
        }
      }
      else digitalWrite(DIR1_RL_ENABLE, 1);

      steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error
      //if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;

      //Don't turn wheels if speed less than 0.3km/hr
      if (gpsSpeed < 0.2) steerAngleError = 0;

      keyaIntendToSteer = true;
      calcSteeringPID();
      if (hasFuncMode(CAN_MODE_VBUS)) {
          canSteerIntend = true;
          float curveCmd = steerAngleSetPoint * steerSettings.steerSensorCounts;
          if (steerConfig.InvertWAS) curveCmd = -curveCmd;
          if (moduleConfig.steerBrand == 3 || moduleConfig.steerBrand == 5) {
              FendtSetCurve = (int16_t)constrain(curveCmd * 10.0f, -32000.0f, 32000.0f);
              setCurve = (uint16_t)(32128 + FendtSetCurve);
          } else {
              setCurve = (uint16_t)constrain(32128L + (int32_t)curveCmd, 0L, 65535L);
          }
          VBus_Send();
      } else {
          motorDrive();
      }
      // Autosteer Led goes GREEN if autosteering

      digitalWrite(AUTOSTEER_ACTIVE_LED, 1);
      digitalWrite(AUTOSTEER_STANDBY_LED, 0);
    }
    else
    {
      //we've lost the comm to AgOpenGPS, or just stop request
      //Disable H Bridge for IBT2, hyd aux, etc for cytron
      if (steerConfig.CytronDriver)
      {
        if (steerConfig.IsRelayActiveHigh)
        {
          digitalWrite(PWM2_RPWM, 1);
        }
        else
        {
          digitalWrite(PWM2_RPWM, 0);
        }
      }
      else digitalWrite(DIR1_RL_ENABLE, 0); //IBT2

      keyaIntendToSteer = false;
      canSteerIntend = false;
      setCurve = 32128;
      pwmDrive = 0;
      if (hasFuncMode(CAN_MODE_VBUS)) {
          VBus_Send();
      } else {
          motorDrive();
      }
      pulseCount = 0;
      // Autosteer Led goes back to RED when autosteering is stopped
      digitalWrite (AUTOSTEER_STANDBY_LED, 1);
      digitalWrite (AUTOSTEER_ACTIVE_LED, 0);
    }
  } //end of timed loop

  //This runs continuously, outside of the timed loop, keeps checking for new udpData, turn sense
  //delay(1);

  // Speed pulse
  if (gpsSpeedUpdateTimer < 1000)
  {
      if (speedPulseUpdateTimer > 200) // 100 (10hz) seems to cause tone lock ups occasionally
      {
          speedPulseUpdateTimer = 0;

          //130 pp meter, 3.6 kmh = 1 m/sec = 130hz or gpsSpeed * 130/3.6 or gpsSpeed * 36.1111
          //gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
          float speedPulse = gpsSpeed * 36.1111;

          //Serial.print(gpsSpeed); Serial.print(" -> "); Serial.println(speedPulse);

          if (gpsSpeed > 0.11) { // 0.10 wasn't high enough
              tone(velocityPWM_Pin, uint16_t(speedPulse));
          }
          else {
              noTone(velocityPWM_Pin);
          }
      }
  }
  else  // if gpsSpeedUpdateTimer hasn't update for 1000 ms, turn off speed pulse
  {
      noTone(velocityPWM_Pin);
  }

  if (encEnable)
  {
    thisEnc = digitalRead(REMOTE_PIN);
    if (thisEnc != lastEnc)
    {
      lastEnc = thisEnc;
      if ( lastEnc) EncoderFunc();
    }
  }

} // end of main loop


// UDP Receive
void ReceiveUdp()
{
    uint16_t len = Eth_udpAutoSteer.parsePacket();

    // if (len > 0)
    // {
    //  Serial.print("ReceiveUdp: ");
    //  Serial.println(len);
    // }

    // Check for len > 4, because we check byte 0, 1, 3 and 3
    if (len > 4)
    {
        Eth_udpAutoSteer.read(autoSteerUdpData, UDP_TX_PACKET_MAX_SIZE);

        if (autoSteerUdpData[0] == 0x80 && autoSteerUdpData[1] == 0x81 && autoSteerUdpData[2] == 0x7F) //Data
        {
            if (autoSteerUdpData[3] == 0xFE && Autosteer_running)  //254
            {
                gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
                gpsSpeedUpdateTimer = 0;

                prevGuidanceStatus = guidanceStatus;

                guidanceStatus = autoSteerUdpData[7];
                guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

                //Bit 8,9    set point steer angle * 100 is sent
                steerAngleSetPoint = ((float)(autoSteerUdpData[8] | ((int8_t)autoSteerUdpData[9]) << 8)) * 0.01; //high low bytes

                //Serial.print("steerAngleSetPoint: ");
                //Serial.println(steerAngleSetPoint);

                //Serial.println(gpsSpeed);

                if ((bitRead(guidanceStatus, 0) == 0) /* || (gpsSpeed < 0.1)*/ || (steerSwitch == 1))
                {
                    watchdogTimer = WATCHDOG_FORCE_VALUE; //turn off steering motor
                }
                else          //valid conditions to turn on autosteer
                {
                    watchdogTimer = 0;  //reset watchdog
                }

                //Bit 10 Tram
                tram = autoSteerUdpData[10];

                //Bit 11
                relay = autoSteerUdpData[11];

                //Bit 12
                relayHi = autoSteerUdpData[12];

                //----------------------------------------------------------------------------
                //Serial Send to agopenGPS

                int16_t sa = (int16_t)(steerAngleActual * 100);

                PGN_253[5] = (uint8_t)sa;
                PGN_253[6] = sa >> 8;

                // heading
                PGN_253[7] = (uint8_t)9999;
                PGN_253[8] = 9999 >> 8;

                // roll
                PGN_253[9] = (uint8_t)8888;
                PGN_253[10] = 8888 >> 8;

                PGN_253[11] = switchByte;
                PGN_253[12] = (uint8_t)pwmDisplay;

                //checksum
                int16_t CK_A = 0;
                for (uint8_t i = 2; i < PGN_253_Size; i++)
                    CK_A = (CK_A + PGN_253[i]);

                PGN_253[PGN_253_Size] = CK_A;

                //off to AOG
                SendUdp(PGN_253, sizeof(PGN_253), Eth_ipDestination, portDestination);

                //Steer Data 2 -------------------------------------------------
                if (steerConfig.PressureSensor || steerConfig.CurrentSensor)
                {
                    if (aog2Count++ > 2)
                    {
                        //Send fromAutosteer2
                        PGN_250[5] = (byte)sensorReading;

                        //add the checksum for AOG2
                        CK_A = 0;

                        for (uint8_t i = 2; i < PGN_250_Size; i++)
                        {
                            CK_A = (CK_A + PGN_250[i]);
                        }

                        PGN_250[PGN_250_Size] = CK_A;

                        //off to AOG
                        SendUdp(PGN_250, sizeof(PGN_250), Eth_ipDestination, portDestination);
                        aog2Count = 0;
                    }
                }

                //Serial.println(steerAngleActual);
                //--------------------------------------------------------------------------
            }

            //steer settings
            else if (autoSteerUdpData[3] == 0xFC && Autosteer_running)  //252
            {
                //PID values
                steerSettings.Kp = ((float)autoSteerUdpData[5]);   // read Kp from AgOpenGPS

                steerSettings.highPWM = autoSteerUdpData[6]; // read high pwm

                steerSettings.lowPWM = (float)autoSteerUdpData[7];   // read lowPWM from AgOpenGPS

                steerSettings.minPWM = autoSteerUdpData[8]; //read the minimum amount of PWM for instant on

                float temp = (float)steerSettings.minPWM * 1.2;
                steerSettings.lowPWM = (byte)temp;

                steerSettings.steerSensorCounts = autoSteerUdpData[9]; //sent as setting displayed in AOG

                steerSettings.wasOffset = (autoSteerUdpData[10]);  //read was zero offset Lo

                steerSettings.wasOffset |= (autoSteerUdpData[11] << 8);  //read was zero offset Hi

                steerSettings.AckermanFix = (float)autoSteerUdpData[12] * 0.01;

                //crc
                //autoSteerUdpData[13];

                //store in EEPROM
                EEPROM.put(10, steerSettings);

                // Re-Init steer settings
                steerSettingsInit();
            }

            else if (autoSteerUdpData[3] == 0xFB)  //251 FB - SteerConfig
            {
                uint8_t sett = autoSteerUdpData[5]; //setting0

                if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
                if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1; else steerConfig.IsRelayActiveHigh = 0;
                if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
                if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
                if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
                if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
                if (bitRead(sett, 6)) steerConfig.SteerButton = 1; else steerConfig.SteerButton = 0;
                if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

                steerConfig.PulseCountMax = autoSteerUdpData[6];

                //was speed
                //autoSteerUdpData[7];

                sett = autoSteerUdpData[8]; //setting1 - Danfoss valve etc

                if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1; else steerConfig.IsDanfoss = 0;
                if (bitRead(sett, 1)) steerConfig.PressureSensor = 1; else steerConfig.PressureSensor = 0;
                if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1; else steerConfig.CurrentSensor = 0;
                if (bitRead(sett, 3)) steerConfig.IsUseY_Axis = 1; else steerConfig.IsUseY_Axis = 0;

                //crc
                //autoSteerUdpData[13];

                EEPROM.put(40, steerConfig);

                // Re-Init
                steerConfigInit();

            }//end FB
            else if (autoSteerUdpData[3] == 200) // Hello from AgIO
            {
                if(Autosteer_running)
                {
                int16_t sa = (int16_t)(steerAngleActual * 100);

                helloFromAutoSteer[5] = (uint8_t)sa;
                helloFromAutoSteer[6] = sa >> 8;

                helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
                helloFromAutoSteer[8] = helloSteerPosition >> 8;
                helloFromAutoSteer[9] = switchByte;

                SendUdp(helloFromAutoSteer, sizeof(helloFromAutoSteer), Eth_ipDestination, portDestination);
                }
                if(useBNO08xRVC || useBNO08xI2C || useTMxx_IMU)
                {
                 SendUdp(helloFromIMU, sizeof(helloFromIMU), Eth_ipDestination, portDestination);
                }
            }

            else if (autoSteerUdpData[3] == 201)
            {
             //make really sure this is the subnet pgn
             if (autoSteerUdpData[4] == 5 && autoSteerUdpData[5] == 201 && autoSteerUdpData[6] == 201)
             {
              networkAddress.ipOne = autoSteerUdpData[7];
              networkAddress.ipTwo = autoSteerUdpData[8];
              networkAddress.ipThree = autoSteerUdpData[9];
        
              //save in EEPROM and restart
              EEPROM.put(60, networkAddress);
              SCB_AIRCR = 0x05FA0004; //Teensy Reset
              }
            }//end 201

            //whoami
            else if (autoSteerUdpData[3] == 202)
            {
                //make really sure this is the reply pgn
                if (autoSteerUdpData[4] == 3 && autoSteerUdpData[5] == 202 && autoSteerUdpData[6] == 202)
                {
                    IPAddress rem_ip = Eth_udpAutoSteer.remoteIP();

                    //hello from AgIO
                    uint8_t scanReply[] = { 128, 129, Eth_myip[3], 203, 7,
                        Eth_myip[0], Eth_myip[1], Eth_myip[2], Eth_myip[3], 
                        rem_ip[0],rem_ip[1],rem_ip[2], 23 };

                    //checksum
                    int16_t CK_A = 0;
                    for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
                    {
                        CK_A = (CK_A + scanReply[i]);
                    }
                    scanReply[sizeof(scanReply)-1] = CK_A;

                    static uint8_t ipDest[] = { 255,255,255,255 };
                    uint16_t portDest = 9999; //AOG port that listens

                    //off to AOG
                    SendUdp(scanReply, sizeof(scanReply), ipDest, portDest);
                }
            }
        } //end if 80 81 7F
    }
}

#ifdef ARDUINO_TEENSY41
void SendUdp(uint8_t *data, uint8_t datalen, IPAddress dip, uint16_t dport)
{
  Eth_udpAutoSteer.beginPacket(dip, dport);
  Eth_udpAutoSteer.write(data, datalen);
  Eth_udpAutoSteer.endPacket();
}
#endif

//ISR Steering Wheel Encoder
void EncoderFunc()
{
  if (encEnable)
  {
    pulseCount++;
    encEnable = false;
  }
}

//Rob Tillaart, https://github.com/RobTillaart/MultiMap
template<typename T>
T multiMap(T value, T* _in, T* _out, uint8_t size)
{
    // take care the value is within range
    // value = constrain(value, _in[0], _in[size-1]);
    if (value <= _in[0]) return _out[0];
    if (value >= _in[size - 1]) return _out[size - 1];

    // search right interval
    uint8_t pos = 1;  // _in[0] already tested
    while (value > _in[pos]) pos++;

    // this will handle all exact "points" in the _in array
    if (value == _in[pos]) return _out[pos];

    // interpolate in the right segment for the rest
    return (value - _in[pos - 1]) * (_out[pos] - _out[pos - 1]) / (_in[pos] - _in[pos - 1]) + _out[pos - 1];
}
