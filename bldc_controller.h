
#ifndef bldc_controller_h
#define bldc_controller_h

#include "Arduino.h"

#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

// diag_pin is the pin used for diagnostic signals
#define diag_pin A0
#define diag_pin_bit (1 << 0)

__inline__ void raise_diag() {
  PORTC |= diag_pin_bit; // set the 'start of cycle' signal (turned off in loop())
}

__inline__ void drop_diag() {
  PORTC &= ~diag_pin_bit;
}

#endif
