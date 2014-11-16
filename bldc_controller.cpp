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
  Serial.print(HEADER); Serial.println();
  Serial.print(HEADER); Serial.println();
  Serial.print(HEADER); Serial.println();

  Serial.print("millis:H:time");
  Serial.print(",rpm:H:gauge");
  Serial.print(",pwm_level:B:gauge");
//  Serial.print(",interrupt count:H:count");
  Serial.print(",desired rpm:H:gauge");
//  Serial.print(",even-odd:h:gauge");
//  Serial.print(",even:H:gauge");
//  Serial.print(",odd:H:gauge");
//  Serial.print(",period:H:gauge");
//  Serial.print(",phase shift:h:gauge");
  Serial.println();

  OCR0A = 0;  //hardcoded
}

#define DELIMITER ","


#define _write_byte(x) { serial_monitor_buffer[serial_monitor_buffer_idx++] = (byte)x; }
#define _write_int(x) { _write_byte(highByte(x)); _write_byte(lowByte(x));}

#define start_serial_monitor() { enable_timer0_compa(); }

static byte compa_count = 0;

unsigned int _millis = 0;
ISR(TIMER0_COMPA_vect) {
  disable_timer0_compa();
  if (SERIAL_MONITOR_INTERVAL == ++compa_count) {  // 1 compa_count is a millisecond
    compa_count = 0;
    unsigned int period = motor.commutation_period;
//    unsigned int last_even = motor.last_even;
//    unsigned int last_odd = motor.last_odd;
//    int phase_shift = motor.phase_shift;
//    unsigned int interrupt_count = motor.interrupt_count;
    unsigned int desired_rpm = motor.desired_rpm;
    byte _pwm_level = pwm_level;
    unsigned int _millis = millis();
    interrupts();

    unsigned int rpm = motor.rpm(period);


    // Speed Control Monitor -- text is bulky and slow, use binary
    byte serial_monitor_buffer_idx = 0;
    byte serial_monitor_buffer[20];
    serial_monitor_buffer_idx = 0;

    _write_int(_millis);
    _write_int(rpm);
    _write_byte(_pwm_level);
//    _write_int(interrupt_count);
    _write_int(desired_rpm);
//    _write_int((int)last_even-last_odd);
//    _write_int(last_even);
//    _write_int(last_odd);
//    _write_int(period);
//    _write_int(phase_shift);

    Serial.write(serial_monitor_buffer, serial_monitor_buffer_idx);
  }
  enable_timer0_compa();
}

void loop() {

  motor.start();
  start_serial_monitor();

  int mult = 0;
  int dir = -1;
  unsigned int _delay;
  
  motor.auto_phase_shift = true;
  do {
    
/*
   if (++mult == 1) {
    	mult = 0;
    	motor.phase_shift += dir;
    	if (motor.phase_shift < -700 || motor.phase_shift > 300) {
    	  dir *= -1;
    	}
    }
*/
    
    _delay = motor.speed_control();

    if (_delay > 10000) {
      delay(_delay / 1000);
    } else if (_delay > 0) {
      delayMicroseconds(_delay);
    }

  } while (_delay >= 0);
}

