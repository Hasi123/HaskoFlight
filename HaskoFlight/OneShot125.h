/*
  OneShot125.h - Library for generating a OneShot125 signal using timer 1 and timer 2
  Created by David Hasko, 27.01.2023
  Released under the GPL-3.0 license

  The pins are unfortunately fixed:
  pin 9, OC1A, PB1, rear right
  pin 10, OC1B, PB2, front right
  pin 11, OC2A, PB3, front left
  pin 3, OC2B, PD3, rear left
*/

#ifndef _ONESHOT125_h
#define _ONESHOT125_h

#include "Arduino.h"

//#define PWM_BLOCKING
#define motorMinPWM 1000

void attachOneShot125(void);
void writeOneShot125(uint16_t (&motorPWM)[4]);

#endif
