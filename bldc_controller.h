
#ifndef bldc_controller_h
#define bldc_controller_h

#include "Arduino.h"

// diag_pin is the pin used for diagnostic signals
#define diag_pin 13
#define diag_pin_bit (1 << (diag_pin - 8))

#define MAX_RPM 10000

// Use the 256x prescaler for a 62.5khz frequency
#define PRESCALE 1<<CS12;
#define TIMER_FREQ (16000000.0 / 256.0) // period of the timer in microseconds (assumes 16mhz cpu)
//LH - hard coded since 4 byte floats are dropping the precision
#define TIMER_MICROS 16


__inline__ void raise_diag() {
  PORTB |= diag_pin_bit; // set the 'start of cycle' signal (turned off in loop())
}

__inline__ void drop_diag() {
  PORTB &= ~diag_pin_bit;
}

#endif
