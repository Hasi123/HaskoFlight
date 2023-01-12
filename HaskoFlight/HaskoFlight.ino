//GCC optimization
#pragma GCC optimize("-Ofast")

#if F_CPU != 16000000L
#error wrong CPU frequency
#endif

#include <util/atomic.h>
#include <avr/pgmspace.h>
#include "MPU6050.h"
#include "PIDcontroller.h"

#define FC_DEBUG
//#define PWM_BLOCKING
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
// motor pins:
// pin 9, OC1A, PB1, rear right
// pin 10, OC1B, PB2, front right
// pin 11, OC2A, PB3, front left
// pin 3, OC2B, PD3, rear left
const uint16_t motorMinPWM = 1000;
const uint16_t motorSpinPWM = 1080;
const uint16_t motorMaxPWM = 2000;
const int16_t rcMidPWM = 1500;
const float levelInputScale = DEG_TO_RAD * ANGLE_MAX / 500.0;
const float rateInputScale = DEG_TO_RAD * RATE_MAX / 500.0;
const float gyroScale = DEG_TO_RAD * 2000.0 / 32767.0;  // max rate / intmax
const float quatScale = 1.0 / 1073741824.0;             // 2^30
const float iMax = 20.0;

volatile int16_t _ppm[PPM_CHANNEL_COUNT];  //roll, pitch, throttle, yaw, flight mode, tuning

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
void startMotorPwm(uint16_t (&motorPWM)[4]);

ISR(PCINT1_vect) {
  static uint16_t pulse;
  static uint16_t counter;
  static uint8_t channel;
  static uint32_t last_micros;

  uint32_t current_micros = micros();
  counter = current_micros - last_micros;
  last_micros = current_micros;

  if (counter < 520) {  //must be a pulse if less than 500us
    pulse = counter;
  } else if (counter > 1910) {  //sync pulses over 1910us
    channel = 0;
  } else {  //servo values
    if (channel < PPM_CHANNEL_COUNT) {
      _ppm[channel] = counter + pulse;
    }
    channel++;
  }
}

ISR(TIMER2_OVF_vect) {
  TCCR2A = _BV(COM2A1) | _BV(COM2B1);

  //if ISR is delayed, trigger immediate pin change
  if (TCNT2 >= OCR2A)
    TCCR2B |= _BV(FOC2A);
  if (TCNT2 >= OCR2B)
    TCCR2B |= _BV(FOC2B);

  //disable overflow interrupt to not run this vector unnecessarily
  TIMSK2 = 0;
}

void setup() {
#ifdef FC_DEBUG
  Serial.begin(115200);
  Serial.println("FC begin");
#endif

  //init ppm
  pinMode(PPM_Pin, INPUT);
  PCMSK1 = _BV(PCINT8);  //listen for pin A0
  PCICR |= _BV(PCIE1);   //enable interrupt

  //continue only after RC ok and disarmed
  while (!_ppm[4] || _ppm[4] > 1250)
    ;

  //init IMU
  mpu.init();  // load dmp and setup for normal use

  //init motor output
  DDRB |= _BV(PINB1) | _BV(PINB2) | _BV(PINB3);
  DDRD |= _BV(PIND3);

  //configure timers and pwm
  TCCR1A = 0;
  TCCR1B = 0;
#ifdef PWM_BLOCKING
  //init timer1 for timing
  TCCR1B = _BV(CS11);  //set timer1 to increment every 0,5 us
#else
  TCCR2A = 0;
  TCCR2B = 0;

  //timer 1: compare output mode to Clear, normal mode, no prescaler
  TCCR1A = _BV(COM1A1) | _BV(COM1B1);
  TCCR1B = _BV(CS10);

  //timer 2: compare output mode to Clear, normal mode, 8th prescaler
  TCCR2A = _BV(COM2A1) | _BV(COM2B1);
  TCCR2B = _BV(CS21);
#endif  //PWM_BLOCKING

#ifdef FC_DEBUG
  Serial.println("FC init done");
#endif
}

void loop() {

  //when new IMU data available calculate PID and output, also serves as motor pwm timing
  if (mpu.newData()) {

    //Benchmark bench;

    int16_t ppm[PPM_CHANNEL_COUNT];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      memcpy(ppm, (const void *)_ppm, sizeof(ppm));
    }

    //yaw always in rate mode, so we can do this regardless of flight mode
    //input scaling
    float setPoint[3];
    setPoint[YAW] = (float)(ppm[3] - rcMidPWM) * rateInputScale;  //desired yaw rate in rad/s

    //measurement
    float processVariable[3];
    processVariable[YAW] = (float)mpu.getGyro(2) * gyroScale;

    //output variables to pass between functions
    float pidValues[3];
    uint16_t motorPWM[4] = { motorMinPWM, motorMinPWM, motorMinPWM, motorMinPWM };

    //track changes of flight mode
    static flightMode lastFlightMode;
    flightMode currentFlightMode;
    if (ppm[4] > 1750)
      currentFlightMode = ANGLE_MODE;
    else if (ppm[4] > 1250)
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
    if (ppm[2] < 1020) {
      pidConroller.reset();
    }

#ifdef KNOB_TUNING_AXIS
    float tuneValue = max((float)ppm[5] KNOB_TUNING_SCALE, 0.0);
    pidConroller.setKPID(KNOB_TUNING_AXIS, KNOB_TUNING_PART, tuneValue);
#ifdef FC_DEBUG
    Serial.println(tuneValue);
#endif
#endif  //KNOB_TUNING_AXIS

    switch (currentFlightMode) {
      case RATE_MODE:
        {

          setPoint[ROLL] = (float)(ppm[0] - rcMidPWM) * rateInputScale;   //desired roll rate
          setPoint[PITCH] = (float)(ppm[1] - rcMidPWM) * rateInputScale;  //desired pitch rate

          //Scale for simple rate controller
          processVariable[ROLL] = (float)mpu.getGyro(1) * gyroScale;
          processVariable[PITCH] = (float)mpu.getGyro(0) * -gyroScale;

          pidConroller.update(setPoint, processVariable, pidValues);

          motorMixer(ppm[2], pidValues, motorPWM);
        }
        break;

      case ANGLE_MODE:
        {

          setPoint[ROLL] = (float)(ppm[0] - rcMidPWM) * levelInputScale;   //desired roll angle
          setPoint[PITCH] = (float)(ppm[1] - rcMidPWM) * levelInputScale;  //desired pitch angle

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

          motorMixer(ppm[2], pidValues, motorPWM);
        }
        break;

      default:
        {
#ifdef CALIB_MOTORS
          if (ppm[2] > rcMidPWM) {
            for (uint16_t &i : motorPWM)
              i = motorMaxPWM;
          }
#endif  //CALIB_MOTORS

          //otherwise motorPWM already at minimum
        }
        break;
    }

    startMotorPwm(motorPWM);

#ifdef FC_DEBUG
    Serial.print(String(processVariable[ROLL] * RAD_TO_DEG) + '\t');
    Serial.print(String(processVariable[PITCH] * RAD_TO_DEG) + '\t');
    Serial.println(String(processVariable[YAW] * RAD_TO_DEG));
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

//expects pwm values between 1000 and 2000, make sure they are in bounds
void startMotorPwm(uint16_t (&motorPWM)[4]) {
#ifdef PWM_BLOCKING
  TCNT1 = 0;
  PORTB |= _BV(1) | _BV(2) | _BV(3);
  PORTD |= _BV(3);
  for (uint16_t &i : motorPWM)
    i <<= 1;
  uint8_t motready = 0;
  while (motready < 0x0F) {
    for (uint8_t i = 0; i < sizeof(motpin); i++) {
      if (motorPWM[i] < TCNT1 && !bitRead(motready, i)) {
        //digitalWrite(motpin[i], LOW);
        bitClear((motpin[i] < 8) ? PORTD : PORTB, motpin[i] % 8);
        bitSet(motready, i);
      }
    }
  }
#else
  OCR1A = motorPWM[0] << 1;
  OCR1B = motorPWM[1] << 1;
  OCR2A = (motorPWM[2] - motorMinPWM) >> 2;
  OCR2B = (motorPWM[3] - motorMinPWM) >> 2;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    //timer 1
    TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0);  //set compare output mode to Set
    TCNT1 = 0;                                                       //reset timer
    TCCR1C = _BV(FOC1A) | _BV(FOC1B);                                //force output compare
    TCCR1A = _BV(COM1A1) | _BV(COM1B1);                              //set compare output mode to Clear

    //timer 2
    TCCR2A = _BV(COM2A1) | _BV(COM2A0) | _BV(COM2B1) | _BV(COM2B0);
    TCNT2 = 6;  //for initial pulse, will overflow in 250 cycles
    TCCR2B = _BV(FOC2A) | _BV(FOC2B) | _BV(CS21);
    //make sure to nail lower pmw, ISR might be delayed
    if (OCR2A <= 6)
      TCCR2A &= ~_BV(COM2A0);
    if (OCR2B <= 6)
      TCCR2A &= ~_BV(COM2B0);
    TIMSK2 = _BV(TOIE2);  //enable overflow interrupt
    TIFR2 = _BV(TOV2);    //clear overflow interrupt flag
  }
#endif  //PWM_BLOCKING
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
- get rid of inv_dmp_uncompress, integrate into MPU6050
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
- separate out ppm and pwm functions to own class
*/
