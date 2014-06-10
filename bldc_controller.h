
#ifndef bldc_controller_h
#define bldc_controller_h

#include "Arduino.h"

// Use the 256x prescaler for a 62.5khz frequency
#define PRESCALE 1<<CS12;
//LH - hard coded since 4 byte floats are dropping the precision
#define TIMER_MICROS 16

// diag_pin is the pin used for diagnostic signals
#define diag_pin 4
#define diag_pin_bit (1 << diag_pin)

__inline__ void raise_diag() {
  PORTD |= diag_pin_bit; // set the 'start of cycle' signal (turned off in loop())
}

__inline__ void drop_diag() {
  PORTD &= ~diag_pin_bit;
}

#endif
