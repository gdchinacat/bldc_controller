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

#include "Arduino.h"
#include "bldc_controller.h"
#include "PWM.h"


void pwm_initialize(byte pwm_mask) {
  // Timer 2 is used for PWM interrupt generation.
  pwm_stop();
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);        // connect pwm, fast pwm
  TCCR2B = PWM_TIMER_PRESCALE;

  pwm_set_mask(pwm_mask);
  pwm_set_level(0);
#ifdef COMPLEMENTARY_SWITCHING
  pwm_set_mask_off(0);
#endif

#ifdef PWM_PIN
  pinMode(PWM_PIN, OUTPUT); // show the source pwm
#endif
}

void __inline__ pwm_start() {
  byte ocr2b = OCR2B;
  if (255 == ocr2b) {
	  enable_timer2_overflow();
	  TCNT2 = 0xFF;
  } else if ((255 - PWM_LEVELS) >= ocr2b) {
	  enable_timer2_compb();
	  TCNT2 = ocr2b;
  } else {
	  enable_timer2_interrupts();
	  TCNT2 = 0xFF;
  }
}

void __inline__ pwm_stop() {
  disable_timer2_interrupts();
}

void __inline__ __pwm_off() {
  // This not uses an extra register and mov, com relative to __pwm_on.
  // I'm not worried.
  PWM_PORT &= ~pwm_mask;
#ifdef COMPLEMENTARY_SWITCHING
  deadtime_delay();
  PWM_PORT |= pwm_mask_off;
#endif
}

void __inline__ __pwm_on() {
#ifdef COMPLEMENTARY_SWITCHING
  PWM_PORT &= ~pwm_mask_off;
  deadtime_delay();
#endif
  PWM_PORT |= pwm_mask;
}

ISR(TIMER2_COMPB_vect) {
  __pwm_off();
}

ISR(TIMER2_OVF_vect) {
  TCNT2 = (255 - PWM_LEVELS);
  __pwm_on();
}

