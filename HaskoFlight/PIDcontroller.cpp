#include "PIDcontroller.h"
#include <Arduino.h>
#include <avr/pgmspace.h>

PIDcontroller::PIDcontroller(const float& in_iMax, const pidStruct (&in_KPID)[3], const float in_dT) {
  m_iMax = in_iMax;
  setKPID(in_KPID);
  m_dT = in_dT;
}

void PIDcontroller::setKPID(const pidStruct (&in_KPID)[3]) {
  memcpy_P(m_Kpid, in_KPID, sizeof(m_Kpid));
}

void PIDcontroller::setKPID(const pidStruct (&in_KPID)[2]) {
  memcpy_P(m_Kpid, in_KPID, sizeof(pidStruct) * 2);
}

void PIDcontroller::setKPID(const flightAxes axis, const pidTerms term, const pidDataType& in_value) {
  pidDataType* newValuePtr = &m_Kpid[axis].P + term;
  *newValuePtr = in_value;
}

void PIDcontroller::update(const float (&setPoint)[3], const float (&processVariable)[3], float (&PIDval)[3], const float* derivative) {
  //passed time
  float dT;
  if (m_dT <= 0.0) {
    uint32_t nowmicros = micros();
    dT = (float)(nowmicros - m_lastmicros) * 1e-6;
    m_lastmicros = nowmicros;
  } else {
    dT = m_dT;
  }

  //PID calculation
  for (uint8_t axis = 0; axis < 3; axis++) {
    float error = setPoint[axis] - processVariable[axis];

    m_iTerm[axis] += error * dT;
    m_iTerm[axis] = constrain(m_iTerm[axis], -m_iMax, m_iMax);

    //D term on process variable instead of error (sign change!)
    float dTerm;
    if (derivative && axis < 2) {  //for angle mode this can directly be the gyro value
      dTerm = derivative[axis];
    } else {
      dTerm = (processVariable[axis] - m_lastD[axis]) / dT;
      m_lastD[axis] = processVariable[axis];
    }

    PIDval[axis] = (m_Kpid[axis].P * error) + (m_Kpid[axis].I * m_iTerm[axis]) - (m_Kpid[axis].D * dTerm);

    //Serial.print(String(m_Kpid[axis].D) + " ");
  }
  //Serial.println();
}

void PIDcontroller::reset(bool resetD) {
  for (uint8_t axis = 0; axis < 3; axis++) {
    m_iTerm[axis] = 0.0;
    if (resetD)
      m_lastD[axis] = 0.0;
  }
}
