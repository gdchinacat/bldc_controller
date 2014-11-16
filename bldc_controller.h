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

// Increase the size of the serial buffer to avoid blocking in the serial
// monitor.
//#define SERIAL_BUFFER_SIZE 256

#ifndef bldc_controller_h
#define bldc_controller_h

#include "Arduino.h"


// should complementary switching be used? (I think I'm doing it right but it gets *really* hot,
// I don't recommend without a fire extinguisher and a will to debug).
// I now think this is just b/c I don't have heat sinks, and it's time...
//#define COMPLEMENTARY_SWITCHING

// Control the RPM rather than the commutation period. rpm tends surge.
#define CONTROL_RPM

// Interval of the serial monitor in ms
#define SERIAL_MONITOR_INTERVAL 1

#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

// diag_pin is the pin used for diagnostic signals
#define diag_pin A0
#define diag_pin_bit (1 << 0)


// deadtime_delay is used to create the necessary delay between turning off a transistor and truning on it's pair.
// it runs with interrupts disabled.
#define deadtime_delay() { __asm__("nop\n\t"); }
//#define deadtime_delay() { };

__inline__ void raise_diag() {
  PORTC |= diag_pin_bit; // set the 'start of cycle' signal (turned off in loop())
}

__inline__ void drop_diag() {
  PORTC &= ~diag_pin_bit;
}


#endif

