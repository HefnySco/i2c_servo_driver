#ifndef I2C_SERVODRIVER_ARDUINO
#define I2C_SERVODRIVER_ARDUINO

#define  VERSION  230

#include "types.h"

// default POSHOLD control gains
#define POSHOLD_P              .11
#define POSHOLD_I              0.0
#define POSHOLD_IMAX           20        // degrees

#define POSHOLD_RATE_P         2.0
#define POSHOLD_RATE_I         0.08      // Wind control
#define POSHOLD_RATE_D         0.045     // try 2 or 3 for POSHOLD_RATE 1
#define POSHOLD_RATE_IMAX      20        // degrees

// default Navigation PID gains
#define NAV_P                  1.4
#define NAV_I                  0.20      // Wind control
#define NAV_D                  0.08      //
#define NAV_IMAX               20        // degrees

#define MINCHECK 1100
#define MAXCHECK 1900

extern volatile unsigned long timer0_overflow_count;

extern const char pidnames[];
extern const char boxnames[];
extern const uint8_t boxids[];

extern uint32_t currentTime;
extern uint32_t previousTime;
extern uint32_t cycleTime;
extern uint16_t calibratingA;
extern uint16_t calibratingB;
extern uint16_t calibratingG;
extern int16_t  magHold,headFreeModeHold;
extern uint8_t  vbatMin;
extern uint8_t  rcOptions[CHECKBOXITEMS];
extern int32_t  AltHold;
extern int16_t  sonarAlt;
extern int16_t  BaroPID;
extern int16_t  errorAltitudeI;

extern int16_t  i2c_errors_count;
extern uint8_t alarmArray[16];
extern global_conf_t global_conf;

extern imu_t imu;
extern analog_t analog;
extern alt_t alt;
extern att_t att;
#ifdef LOG_PERMANENT
extern plog_t plog;
#endif

extern int16_t debug[4];

extern conf_t conf;

extern int16_t  annex650_overrun_count;
extern flags_struct_t f;
extern uint16_t intPowerTrigger1;

extern int16_t gyroZero[3];
extern int16_t angle[2];


#if BARO
  extern int32_t baroPressure;
  extern int16_t baroTemperature; // temp in 0.01 deg
  extern int32_t baroPressureSum;
#endif

extern int16_t axisPID[3];
extern int16_t motor[8];
extern int16_t servo[8];

extern int16_t failsafeEvents;
extern volatile int16_t failsafeCnt;

extern int16_t rcData[RC_CHANS];
extern int16_t rcSerial[8];
extern volatile uint16_t Servo_Buffer[18];
extern uint8_t rcSerialCount;
extern int16_t lookupPitchRollRC[5];
extern int16_t lookupThrottleRC[11];
extern volatile uint8_t i2c_slave_received;
#if defined(POWERMETER) || ( defined(LOG_VALUES) && (LOG_VALUES >= 3) )
  #define PMOTOR_SUM 8                     // index into pMeter[] for sum
  extern uint32_t pMeter[PMOTOR_SUM + 1];  // we use [0:7] for eight motors,one extra for sum
  extern uint8_t pMeterV;                  // dummy to satisfy the paramStruct logic in ConfigurationLoop()
  extern uint32_t pAlarm;                  // we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
  extern uint16_t powerValue;              // last known current
#endif

#if defined(LCD_TELEMETRY)
  extern uint8_t telemetry;
  extern uint8_t telemetry_auto;
#endif
#ifdef LCD_TELEMETRY_STEP
  extern char telemetryStepSequence[];
  extern uint8_t telemetryStepIndex;
#endif

#if defined(LOG_VALUES) || defined(LCD_TELEMETRY)
  extern uint16_t cycleTimeMax;       // highest ever cycle timen
  extern uint16_t cycleTimeMin;       // lowest ever cycle timen
  extern int32_t  BAROaltMax;         // maximum value
  extern uint16_t GPS_speedMax;       // maximum speed from gps
  extern uint16_t powerValueMaxMAH;
#endif
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING) || defined(LOG_PERMANENT)
  extern uint32_t armedTime;
#endif


  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Serial GPS only variables
  //navigation mode
  #define NAV_MODE_NONE          0
  #define NAV_MODE_POSHOLD       1
  #define NAV_MODE_WP            2
  extern uint8_t nav_mode; // Navigation mode
  extern int16_t  nav[2];
  extern int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

  // default POSHOLD control gains
  #define POSHOLD_P              .11
  #define POSHOLD_I              0.0
  #define POSHOLD_IMAX           20        // degrees

  #define POSHOLD_RATE_P         2.0
  #define POSHOLD_RATE_I         0.08      // Wind control
  #define POSHOLD_RATE_D         0.045     // try 2 or 3 for POSHOLD_RATE 1
  #define POSHOLD_RATE_IMAX      20        // degrees

  // default Navigation PID gains
  #define NAV_P                  1.4
  #define NAV_I                  0.20      // Wind control
  #define NAV_D                  0.08      //
  #define NAV_IMAX               20        // degrees

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Serial GPS only variables
  //navigation mode
  #define NAV_MODE_NONE          0
  #define NAV_MODE_POSHOLD       1
  #define NAV_MODE_WP            2

  extern volatile uint8_t  spekFrameFlags;
  extern volatile uint32_t spekTimeLast;
  extern uint8_t  spekFrameDone;

  #if defined(OPENLRSv2MULTI)
    extern uint8_t pot_P,pot_I; // OpenLRS onboard potentiometers for P and I trim or other usages
  #endif

  // **********************
  //Automatic ACC Offset Calibration
  // **********************
  #if defined(INFLIGHT_ACC_CALIBRATION)
    extern uint16_t InflightcalibratingA;
    extern int16_t AccInflightCalibrationArmed;
    extern uint16_t AccInflightCalibrationMeasurementDone;
    extern uint16_t AccInflightCalibrationSavetoEEProm;
    extern uint16_t AccInflightCalibrationActive;
  #endif

#if defined(ARMEDTIMEWARNING)
  extern uint32_t  ArmedTimeWarningMicroSeconds;
#endif

#if defined(THROTTLE_ANGLE_CORRECTION)
  extern int16_t throttleAngleCorrection;
  extern int8_t  cosZ;
#endif

void annexCode();

#endif /* MULTIWII_H_ */
