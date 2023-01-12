#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

//#pragma GCC optimize("-Ofast")

#include <Arduino.h>

enum flightAxes { ROLL,
                  PITCH,
                  YAW };

enum pidTerms { PROPORTIONAL,
                INTEGRAL,
                DERIVATIVE };

typedef uint16_t pidDataType;

struct pidStruct {
  pidDataType P, I, D;
};

class PIDcontroller {
public:
  PIDcontroller(const float& in_iMax, const pidStruct (&in_KPID)[3], const float in_dT = 0.0);

  void setKPID(const pidStruct (&in_KPID)[3]);
  void setKPID(const pidStruct (&in_KPID)[2]);
  void setKPID(const flightAxes axis, const pidTerms term, const pidDataType& in_value);
  void update(const float (&setPoint)[3], const float (&processVariable)[3], float (&PIDval)[3], const float* derivative = nullptr);
  void reset(bool resetD = false);

private:
  float m_dT;
  float m_iMax;
  pidStruct m_Kpid[3];
  uint32_t m_lastmicros;
  float m_iTerm[3];
  float m_lastD[3];
};

#endif