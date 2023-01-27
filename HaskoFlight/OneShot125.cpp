/*
  OneShot125.cpp - Library for generating a OneShot125 signal using timer 1 and timer 2
  Created by David Hasko, 27.01.2023
  Released under the GPL-3.0 license

  The pins are unfortunately fixed:
  pin 9, OC1A, PB1, rear right
  pin 10, OC1B, PB2, front right
  pin 11, OC2A, PB3, front left
  pin 3, OC2B, PD3, rear left
*/

#include "Arduino.h"
#include "OneShot125.h"
#include <util/atomic.h>

//init motor output
void attachOneShot125(void) {
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
}

uint8_t motpin[] = { 9, 10, 11, 3 };

//expects pwm values between 1000 and 2000, make sure they are in bounds
void writeOneShot125(uint16_t (&motorPWM)[4]) {
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
