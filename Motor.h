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

#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

// Control the RPM rather than the commutation period. rpm tends surge.
#define CONTROL_RPM

// What should the diagnostics pin be raised?
//#define DIAG_ZC_INTERRUPT
//#define DIAG_ZC
#define DIAG_COMMUTATION_CYCLE
//#define DIAG_COMMUTATION_INTERRUPT

#define SPEED_PIN A5

// lots of hard coded timer stuff here....
#define disable_timer1_compb();  (TIMSK1 &= ~_BV(OCIE1B));
#define enable_timer1_compb();  (TIMSK1 |= _BV(OCIE1B));

class Motor {
  
  public:
    Motor(int poles, int speed_pin);
    void start();
    unsigned int rpm();

    void next_commutation(); // advance to the next commutation
    void pwm_on();  // called when pwm should be on 
    void pwm_off(); // called when pwm should be off
    unsigned int speed_control();
    void zero_crossing_interrupt();  // called from global
    
  private:
    int poles;       // number of pole pairs
    int speed_pin;   // pin the speed potentiometer is connected to
    char direction;
    
    boolean sensing; // is commutation driven by interrupts (rather than timing)

    // commutation
    int interrupt_count;              // # of interrupts since last reset()
    int phase_shift;                  // # of ticks to shift commutation by
    unsigned int commutation_period;  // last commutation period, in timer1 ticks
    byte commutation;                 // current commutation step
    byte _commutation;                // the current commutation power bits
    
    void reset();
    void initialize_timers();
   
};

#endif
