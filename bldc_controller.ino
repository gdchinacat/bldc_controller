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

void setup() {

  pinMode(diag_pin, OUTPUT);
  
  //Serial.begin(115200);
  //Serial.setTimeout(5);
  
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
  //int dir = 1;
  unsigned int _delay;
  
//  Serial.print("pwm_level,rpm,phase_shift"); Serial.println();
  do {
    _delay = motor.speed_control();
    if (_delay > 10000) {
      delay(_delay / 1000);
    } else if (_delay > 0) {
      delayMicroseconds(_delay);
    }

//    if (++mult == 1) {
//      mult = 0;
//
//
//      // phase debugging
////      noInterrupts();
////      int phase_shift = motor.phase_shift += dir;
////      int over = motor.commutation_period * 0.45;
////      int under = motor.commutation_period * 0.35;
////      interrupts();
////      if (phase_shift <= -over) {
////        dir = 1;
////      } else if (phase_shift >= under) {
////        dir = -1;
////      }
//
//      
////      noInterrupts();
////      int period = motor.commutation_period;
////      interrupts();
//      
//      // Speed Control Monitor
//      //Serial.print("\tdesired: "); Serial.print(desired_commutation_period);
//      //Serial.print(period); Serial.print("\t");
//      //Serial.print((int)motor.power_level); Serial.print("\t");
//      Serial.print(pwm_level); Serial.print("\t");
//      Serial.print(motor.rpm());  Serial.print("\t");
//      //Serial.print(phase_shift); Serial.print("\t");
//      //if (motor.sensing) { Serial.print("\tsensing"); }
//      Serial.println();
//
//    }

  } while (_delay >= 0);
}


