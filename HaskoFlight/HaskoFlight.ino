//GCC optimization
#pragma GCC optimize("-Ofast")

#if F_CPU != 16000000L
#error wrong CPU frequency
#endif

#include <avr/pgmspace.h>
#include "MPU6050.h"
#include "PIDcontroller.h"
#include "CPPM.h"
#include "OneShot125.h"

#define FC_DEBUG
//#define CALIB_MOTORS

class Benchmark {
public:
  Benchmark() {
    m_startMicros = micros();
  }
  ~Benchmark() {
#ifdef FC_DEBUG
    Serial.println(micros() - m_startMicros);
#endif
  }
private:
  uint32_t m_startMicros;
};

#define ANGLE_MAX 45.0       //in degrees
#define RATE_MAX 350.0       //in deg/s
#define PPM_CHANNEL_COUNT 8  //how many RC channels do you need?
const uint8_t PPM_Pin = A0;  //PC0, PCINT8
const uint16_t motorSpinPWM = 1080;
const uint16_t motorMaxPWM = 2000;
const int16_t rcMidPWM = 1500;
const float levelInputScale = DEG_TO_RAD * ANGLE_MAX / 500.0;
const float rateInputScale = DEG_TO_RAD * RATE_MAX / 500.0;
const float gyroScale = DEG_TO_RAD * 2000.0 / 32767.0;  // max rate / intmax
const float quatScale = 1.0 / 1073741824.0;             // 2^30
const float iMax = 20.0;

enum flightMode { DISARMED,
                  RATE_MODE,
                  ANGLE_MODE };

//{ P,  I,  D}
const pidStruct KpidRate[] PROGMEM = {
  { 120, 0, 4 },   //roll
  { 90, 0, 4 },    //pitch
  { 250, 170, 4 }  //yaw
};

const pidStruct KpidAngle[] PROGMEM = {
  { 110, 300, 26 },
  { 150, 400, 32 }
};

//#define KNOB_TUNING_AXIS YAW
#define KNOB_TUNING_PART INTEGRAL
//#define KNOB_TUNING_SCALE / 30 - 33  //0-33
//#define KNOB_TUNING_SCALE / 10 - 80  //20-120
//#define KNOB_TUNING_SCALE / 2 - 450  //50-550
#define KNOB_TUNING_SCALE -1000  //0-1000

PIDcontroller pidConroller(iMax, KpidRate);

//forward declarations
void motorMixer(const int16_t throttle, const float (&PIDval)[3], uint16_t (&motorPWM)[4]);

void setup() {
#ifdef FC_DEBUG
  Serial.begin(115200);
  Serial.println("FC begin");
#endif

  //init ppm
  attachCPPM(PPM_Pin, PPM_CHANNEL_COUNT);

  //continue only after RC ok and disarmed
  while (!CPPMget(4) || CPPMget(4) > 1250)
    ;

  //init IMU
  mpu.init();  // load dmp and setup for normal use

  //init motor pwm
  attachOneShot125();

#ifdef FC_DEBUG
  Serial.println("FC init done");
#endif
}

void loop() {

  //when new IMU data available calculate PID and output, also serves as motor pwm timing
  if (mpu.newData()) {

    //Benchmark bench;

    //yaw always in rate mode, so we can do this regardless of flight mode
    //input scaling
    float setPoint[3];
    setPoint[YAW] = (float)(CPPMget(3) - rcMidPWM) * rateInputScale;  //desired yaw rate in rad/s

    //measurement
    float processVariable[3];
    processVariable[YAW] = (float)mpu.getGyro(2) * gyroScale;

    //output variables to pass between functions
    float pidValues[3];
    uint16_t motorPWM[4] = { motorMinPWM, motorMinPWM, motorMinPWM, motorMinPWM };

    //track changes of flight mode
    static flightMode lastFlightMode;
    flightMode currentFlightMode;
    if (CPPMget(4) > 1750)
      currentFlightMode = ANGLE_MODE;
    else if (CPPMget(4) > 1250)
      currentFlightMode = RATE_MODE;
    else
      currentFlightMode = DISARMED;

    if (currentFlightMode != lastFlightMode) {
      lastFlightMode = currentFlightMode;
      //change Kpid
      if (currentFlightMode == ANGLE_MODE)
        pidConroller.setKPID(KpidAngle);
      else if (currentFlightMode == RATE_MODE)
        pidConroller.setKPID(KpidRate);
      //reset PID i and d term
      pidConroller.reset(true);
    }

    //to prevent I term windup on ground
    if (CPPMget(2) < 1020) {
      pidConroller.reset();
    }

#ifdef KNOB_TUNING_AXIS
    float tuneValue = max((float)CPPMget(5) KNOB_TUNING_SCALE, 0.0);
    pidConroller.setKPID(KNOB_TUNING_AXIS, KNOB_TUNING_PART, tuneValue);
#ifdef FC_DEBUG
    Serial.println(tuneValue);
#endif
#endif  //KNOB_TUNING_AXIS

    switch (currentFlightMode) {
      case RATE_MODE:
        {

          setPoint[ROLL] = (float)(CPPMget(0) - rcMidPWM) * rateInputScale;   //desired roll rate
          setPoint[PITCH] = (float)(CPPMget(1) - rcMidPWM) * rateInputScale;  //desired pitch rate

          //Scale for simple rate controller
          processVariable[ROLL] = (float)mpu.getGyro(1) * gyroScale;
          processVariable[PITCH] = (float)mpu.getGyro(0) * -gyroScale;

          pidConroller.update(setPoint, processVariable, pidValues);

          motorMixer(CPPMget(2), pidValues, motorPWM);
        }
        break;

      case ANGLE_MODE:
        {

          setPoint[ROLL] = (float)(CPPMget(0) - rcMidPWM) * levelInputScale;   //desired roll angle
          setPoint[PITCH] = (float)(CPPMget(1) - rcMidPWM) * levelInputScale;  //desired pitch angle

          //convert quaternions from int32 to float and normalize
          float q[4];
          for (uint8_t i = 0; i < 4; i++)
            q[i] = (float)mpu.getQuat(i) * quatScale;

          // Compute Tait-Bryan angles for angle mode
          // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
          processVariable[ROLL] = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
          processVariable[PITCH] = -atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
          //processVariable[YAW] = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));

          float deriv[] = { (float)mpu.getGyro(1) * gyroScale,
                            (float)mpu.getGyro(0) * -gyroScale };

          pidConroller.update(setPoint, processVariable, pidValues, deriv);

          motorMixer(CPPMget(2), pidValues, motorPWM);
        }
        break;

      default:
        {
#ifdef CALIB_MOTORS
          if (CPPMget(2) > rcMidPWM) {
            for (uint16_t &i : motorPWM)
              i = motorMaxPWM;
          }
#endif  //CALIB_MOTORS

          //otherwise motorPWM already at minimum
        }
        break;
    }

    writeOneShot125(motorPWM);

#ifdef FC_DEBUG
    Serial.print(String(micros()) + '\t');
    Serial.print(String(setPoint[ROLL] * RAD_TO_DEG) + '\t');
    Serial.print(String(processVariable[ROLL] * RAD_TO_DEG));
    //Serial.print(String(processVariable[PITCH] * RAD_TO_DEG) + '\t');
    //Serial.print(String(processVariable[YAW] * RAD_TO_DEG));
    Serial.println();
#endif  //FC_DEBUG
  }
}

void motorMixer(const int16_t throttle, const float (&PIDval)[3], uint16_t (&motorPWM)[4]) {
  //mixer for quadcopter X layout
  motorPWM[0] = constrain((uint16_t)(throttle - PIDval[ROLL] + PIDval[PITCH] + PIDval[YAW]), motorSpinPWM, motorMaxPWM);  //rear right
  motorPWM[1] = constrain((uint16_t)(throttle - PIDval[ROLL] - PIDval[PITCH] - PIDval[YAW]), motorSpinPWM, motorMaxPWM);  //front right
  motorPWM[2] = constrain((uint16_t)(throttle + PIDval[ROLL] - PIDval[PITCH] + PIDval[YAW]), motorSpinPWM, motorMaxPWM);  //front left
  motorPWM[3] = constrain((uint16_t)(throttle + PIDval[ROLL] + PIDval[PITCH] - PIDval[YAW]), motorSpinPWM, motorMaxPWM);  //rear left
}

/*
TODO:
-- IMU euler angles
-- ppm input
-- 4 pwm outputs
-- pid controller x3
-- mixer
-- failsafe - set on receiver
-- mode change ch5
- IMU calibration (gyro done automatically after 8s of no movement) - using? mpu_run_self_test(long *gyro, long *accel)
-- Increase DMP to 200Hz
-- Add rate controller - needs Kpid switching
-- Figure out yaw
-- Tune
-- Motor PWM timing
- remount IMU to correct orientation, or do some internal sensor rotation - only possible in dmp
-- scale either to degrees or radians overall
-- no global variables where possible
-- change Dterm on process variable instead of error (fixes sudden D changes)
-- optimization: manually set dT in PID controller
-- test gyro value against processVariable change. Need to scale?
-- improve parsing dmp fifo data with endian conversion and/or struct
-- get rid of I2C helper, integrate into MPU6050
-- get rid of inv_dmp_uncompress, integrate into MPU6050
- reconfigure IMU on the fly for faster rate mode
- convert PID calculation to integer math
- GPS integration, calculate heading, DMP good enough???
-- improve pidStuct, extract yaw from level
-- integral safety on ground, at low throttle
-- put PID constants into EEPROM, or progmem
- experiment with GCC optimization - not faster in pidcontroller
- bug: roll/pitch coupling on yaw command in angle mode - probably esc problem? - serial debug?
-- OTA firmware update
- save automatic gyro calibration - mpu_write_mem(D_EXT_GYRO_BIAS_X, 12, regs)
-- separate out ppm and pwm functions to own class
- write own UART library with a circular buffer for logging (if need more resources)
*/
