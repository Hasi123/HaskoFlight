/*
  CPPM.cpp - Library for reading a CPPM (PPM SUM) stream using interrupts
  Created by David Hasko, 15.01.2023
  Released under the GPL-3.0 license
*/

#include "Arduino.h"
#include "CPPM.h"
#include <util/atomic.h>

static uint8_t _cppm_count;
static volatile int16_t *_cppm_val;

bool attachCPPM(const uint8_t pin, const uint8_t count) {
  //only port C (A0-A5, PCINT8-13) has an ISR defined below, other pins would trigger __bad_interrupt
  if (pin < A0 || pin > A5)
    return false;

#if 0
  _cppm_val = (int16_t *)calloc(count, sizeof(int16_t));
#else
  //smaller code?
  _cppm_val = (int16_t *)malloc(count * sizeof(int16_t));
  if (!_cppm_val)
    return false;
  memset((void *)_cppm_val, 0, count * sizeof(int16_t));
#endif
  _cppm_count = count;

//configure timer1
#ifdef CPPM_USE_TIMER1
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << CS11);
#endif

  //configure pin
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *ddrReg = portModeRegister(port);
  volatile uint8_t *pcmskReg = digitalPinToPCMSK(pin);

  *ddrReg &= ~bit;        //INPUT
  *pcmskReg = bit;        //select pin
  PCICR = _BV(port - 2);  //enable pin change interrupt

  return true;
}

void detachCPPM(void) {
  PCICR = 0;
  _cppm_count = 0;
  free((void *)_cppm_val);
  _cppm_val = nullptr;
}

int16_t CPPMget(const uint8_t channel) {
  if (channel >= _cppm_count)
    return -1;
  //16 bit read is two instructions on AVR, the ISR could update the value in between
  int16_t value;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    value = _cppm_val[channel];
  }
  return value;
}

void CPPMintFunc(void) {
  static uint16_t pulse;
  static uint16_t counter;
  static uint8_t channel;

#ifdef CPPM_USE_TIMER1
  counter = TCNT1;
  TCNT1 = 0;
  counter /= (F_CPU / 8000000);
#else
  static uint32_t last_time;
  uint32_t current_micros = micros();
  counter = current_micros - last_time;
  last_time = current_micros;
#endif

  if (counter < 520) {  //must be a pulse if less than 520us
    pulse = counter;
  } else if (counter > 1910) {  //sync pulses over 1910us
    channel = 0;
  } else {  //servo values
    if (channel < _cppm_count) {
      _cppm_val[channel] = counter + pulse;
    }
    channel++;
  }
}
/*
ISR(PCINT0_vect) {
  CPPMintFunc();
}
*/
ISR(PCINT1_vect) {
  CPPMintFunc();
}
/*
ISR(PCINT2_vect) {
  CPPMintFunc();
}
*/