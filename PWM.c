
#include "Arduino.h"
#include "PWM.h"
#include "bldc_controller.h"

// the pwm mask is what is toggled in the PWM_PORT. A register is reserved for it to
// avoid loading it from main memory each interrupt (they happen fairly frequently,
// so it adds up).
register byte pwm_mask asm("r3");

void pwm_initialize(byte pwm_mask) {
  // Timer 2 is used for PWM interrupt generation.
  pwm_stop();
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);        // connect pwm, fast pwm
  TCCR2B = PWM_TIMER_PRESCALE;

  pwm_set_mask(pwm_mask);
  pwm_set_level(0);

  pinMode(PWM_PIN, OUTPUT); // show the source pwm
}

void pwm_start() {
  TCNT2 = 0xFF;
  enable_timer2_interrupts();
  
  // TODO HACK - pwm_set_level twiddles the interrupts, but we just turned them on...
  pwm_set_level(pwm_level);
}

void pwm_stop() {
  disable_timer2_interrupts();
}

ISR(TIMER2_COMPB_vect) {
  _PWM_OFF();
}

ISR(TIMER2_OVF_vect) {
  _PWM_ON();
  TCNT2 = (256 - PWM_LEVELS);
}

void pwm_set_mask(byte mask) {
  boolean started = timer2_overflow_enabled() | timer2_compb_enabled();
  if (started) {
    pwm_stop();
  }

  pwm_mask = mask;

  if (started) {
    pwm_start();
  }
}

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
    _PWM_OFF();
  } else if (timer2_compb_enabled()) {
    enable_timer2_overflow();   // enable interrupt to turn it on
  }
  
  if (level == PWM_LEVELS - 1) {
    disable_timer2_compb();
    _PWM_ON();
  } else if (timer2_overflow_enabled()) {
    enable_timer2_compb();
  }
  OCR2B = level + (256 - PWM_LEVELS);
  interrupts();
}


