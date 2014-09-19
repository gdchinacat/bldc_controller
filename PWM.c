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

// the pwm mask is what is toggled in the PWM_PORT. A register is reserved for it to
// avoid loading it from main memory each interrupt (they happen fairly frequently,
// so it adds up).
register byte pwm_mask asm("r3");
#ifdef COMPLEMENTARY_SWITCHING
register byte pwm_mask_off asm("r4");
#endif

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

void pwm_start() {
  TCNT2 = 0xFF;
  enable_timer2_interrupts();
  pwm_set_level(pwm_level); // reset the power level
}

void pwm_stop() {
  disable_timer2_interrupts();
}

void __inline__ __pwm_off() {
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
  TCNT2 = (256 - PWM_LEVELS);
  __pwm_on();
}

void pwm_set_mask(byte mask) {
  pwm_mask = mask;
}

#ifdef COMPLEMENTARY_SWITCHING
void pwm_set_mask_off(byte mask_off) {
  pwm_mask_off = mask_off;
}
#endif

void pwm_set_level(byte level) {
  
  level = constrain(level, 0, PWM_LEVELS - 1);
 
  noInterrupts();
  if (level == 0) {
    //this looks weird on the oscilloscope since we ignore the overflow
    //we don't reset the counter to 256-PWM_LEVELS and the diagnostic 
    //port and the generated pwm differ...the source appears to drop to
    //a lower frequency at a higher power. This is just an artifact of 
    //not reseting the counter on each overflow. You've been warned.
    disable_timer2_overflow();  // disable interrupt that turns it on
    __pwm_off();
  } else if (timer2_compb_enabled()) {
    enable_timer2_overflow();   // enable interrupt to turn it on
  }
  
  if (level == PWM_LEVELS - 1) {
    disable_timer2_compb();
    __pwm_on();
  } else if (timer2_overflow_enabled()) {
    enable_timer2_compb();
  }
  OCR2B = level + (256 - PWM_LEVELS);
  interrupts();
}


