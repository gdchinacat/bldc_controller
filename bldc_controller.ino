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

#define HEADER "-=-=-=-="

void setup() {

  pinMode(diag_pin, OUTPUT);

  Serial.begin(4000000);
  //Serial.setTimeout(5);
  Serial.print(HEADER); Serial.println();
  Serial.print(HEADER); Serial.println();
  Serial.print(HEADER); Serial.println();
  Serial.print("millis:>H:time");
  Serial.print(",rpm:>H:gauge");
//  Serial.print(",pwm_level:B:gauge");
//  Serial.print(",interrupt count:>H:count");
//  Serial.print(",desired rpm:>H:gauge");
//  Serial.print(",even-odd:>h:gauge");
  Serial.print(",even:>H:gauge");
  Serial.print(",odd:>H:gauge");
  Serial.print(",period:>H:gauge");
  Serial.println();
  OCR0A = 0;  //hardcode
  enable_timer0_compa();

}

#define DELIMITER ","


#define _write_byte(x) { serial_monitor_buffer[serial_monitor_buffer_idx++] = (byte)x; }
#define _write_int(x) { _write_byte(highByte(x)); _write_byte(lowByte(x));}

void serial_monitor() {
      unsigned int period = motor.commutation_period;
      unsigned int last_even = motor.last_even;
      unsigned int last_odd = motor.last_odd;
//      int phase_shift = motor.phase_shift;
//      unsigned int interrupt_count = motor.interrupt_count;
//      unsigned int desired_rpm = motor.desired_rpm;
//      byte _pwm_level = pwm_level;
      unsigned int _millis = millis();
      interrupts();
      
      unsigned int rpm = motor.rpm(period);
      
      
      // Speed Control Monitor -- text is bulky and slow, use binary
      byte serial_monitor_buffer_idx = 0;
      byte serial_monitor_buffer[20];
      serial_monitor_buffer_idx = 0;
      _write_int(_millis)
      _write_int(rpm);
//      _write_byte(_pwm_level);
//      _write_int(interrupt_count);
//      _write_int(desired_rpm);
//      _write_int((int)last_even-last_odd);
      _write_int(last_even);
      _write_int(last_odd);
      _write_int(period);
      Serial.write(serial_monitor_buffer, serial_monitor_buffer_idx);
}

static byte compa_count = 0;
ISR(TIMER0_COMPA_vect) {
  disable_timer0_compa();
  if (5 == ++compa_count) {  // 1 compa_count is a millisecond
    compa_count = 0;
    serial_monitor();
  }
  enable_timer0_compa();
}

void loop() {

// PWM test code  
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



