
#ifndef bldc_controller_h
#define bldc_controller_h

#include "Arduino.h"

#define MAX_RPM 10000

// the pin the potentiometer is connected to
#define pot_pin A5

// the pin to use for commutation interrupts
#define commutation_interrupt 0

// diag_pin is the pin used for diagnostic signals
#define diag_pin 4
#define diag_pin_bit (1 << diag_pin)

// Use the 256x prescaler for a 62.5khz frequency
#define PRESCALE 1<<CS12;
#define TIMER_FREQ (16000000.0 / 256.0) // period of the timer in microseconds (assumes 16mhz cpu)
//LH - hard coded since 4 byte floats are dropping the precision
#define TIMER_MICROS 16

extern byte commutation;
extern byte commutation_bits[];
extern int commutation_to_skip;

extern void next_commutation(void);

#define ALL_COMMUTATION_BITS_OFF B11000000

__inline__ void all_off() {
  PORTB &= ALL_COMMUTATION_BITS_OFF;
}

__inline__ void raise_diag() {
  PORTD |= diag_pin_bit; // set the 'start of cycle' signal (turned off in loop())
}

__inline__ void drop_diag() {
  PORTD &= ~diag_pin_bit;
}

#endif
