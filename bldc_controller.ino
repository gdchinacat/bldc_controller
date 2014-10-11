/*
    Copyright (C) 2014 Anthony Hutchinson

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <math.h>
#include "bldc_controller.h"
#include "Motor.h"

extern "C" {
#include "PWM.h"
}

extern Motor motor;

#define enable_timer0_compa();   TIMSK0 |= _BV(OCIE0A);
#define disable_timer0_compa();   TIMSK0 |= _BV(OCIE0A);

void setup() {

  pinMode(diag_pin, OUTPUT);

  Serial.begin(4000000);
  //Serial.setTimeout(5);

  OCR0A = 0;  //hardcoded
  enable_timer0_compa();

}

#define DELIMITER ","

void serial_monitor() {
//      int period = motor.commutation_period;
//      unsigned int last_even = motor.last_even;
//      unsigned int last_odd = motor.last_odd;
//      int phase_shift = motor.phase_shift;
      unsigned int interrupt_count = motor.interrupt_count;
      interrupts();
      
      // Speed Control Monitor -- text is bulky and slow, use binary
      Serial.print(millis()); Serial.print(DELIMITER);
      Serial.print(motor.rpm());  Serial.print(DELIMITER);
      //Serial.print(phase_shift); Serial.print(DELIMITER);
      //Serial.print((int)last_even - (int)last_odd); Serial.print(DELIMITER);
      Serial.print((int)pwm_level); Serial.print(DELIMITER);
      Serial.print(interrupt_count); Serial.print(DELIMITER);
      Serial.println();
}

static byte compa_count = 0;
ISR(TIMER0_COMPA_vect) {
  disable_timer0_compa();
  //OCR0A = TCNT0 + 1000;  //hardcoded
  if (20 == ++compa_count) {  // each one is a millisecond
    compa_count = 0;
    serial_monitor();
  }
  enable_timer0_compa();
}

void loop() {
  
//  #define ON (_BV(1))
//  #define OFF (_BV(1))
//
//  pwm_initialize(ON);
//  pwm_set_mask_off(OFF);
//  pwm_set_level(32);
//  pwm_start(); 
//  while (true);
//  return;

  motor.start();

  int mult = 0;
  //int dir = -1;
  unsigned int _delay;
  
//  Serial.print("rpm"); Serial.print(DELIMITER);
//  Serial.print("phase_shift"); Serial.print(DELIMITER);
//  Serial.print("even - odd"); Serial.print(DELIMITER);
//  Serial.print("pwm_level"); Serial.print(DELIMITER);
//  Serial.print("millis"); Serial.print(DELIMITER);
//  Serial.println();

  //motor.auto_phase_shift = true;
  do {
    _delay = motor.speed_control();

    if (_delay > 10000) {
      delay(_delay / 1000);
    } else if (_delay > 0) {
      delayMicroseconds(_delay);
    }

  } while (_delay >= 0);
}



