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
#include "Motor.h"
extern "C" {
#include "PWM.h"
}

/** the motor */
Motor motor(4, SPEED_PIN);

#define MOTOR_PORT PWM_PORT
#define MOTOR_DDR DDRB

// Since the commutation masks are in a vector there is no need for these
// to be rigidly defined...initialize could take the ports. However, since
// they are all on the same port and we want *that* statically defined for
// runtime performance reasons, I don't see any benefit to not doing this.
#define A  _BV(PINB0)
#define A_ _BV(PINB1)
#define B  _BV(PINB2)
#define B_ _BV(PINB3)
#define C  _BV(PINB4)
#define C_ _BV(PINB5)

#define LOW_COMMUTATION_BITS   (A_ | B_ | C_)
#define HIGH_COMMUTATION_BITS  (A  | B  | C )

/* The commutation sequence */
const byte commutation_bits[6] = {A | C_,
                                  A | B_,
                                  C | B_,
                                  C | A_,
                                  B | A_,
                                  B | C_};

/* 
The plugging braking commutation sequence, essentially swap A and B. 

This does not appear to be compatible with the zero crossing wiring
scheme since plugging required powering the phase that crosses zero.
It is possible to use pwm off time to monitor the phase, but the
current implementation wires the *filtered* signals to the zero
crossing which persents a hall sensor-like square wave. This just
isn't going to happen with the current wiring. There are pins, but
I plan to use them for a second motor instance.

So, I'm leaving it and this comment here for now and moving right
along. A project for another day when I actually need braking, and
it's not just a "why can't I do it". 
*/                                 
const byte braking_commutation_bits[6] = {B | C_,
                                          B | A_,
                                          C | A_,
                                          C | B_,
                                          A | B_,
                                          A | C_};

#define anti_commutation(commutation) (commutation < 3 ? commutation_bits[commutation + 3] : commutation_bits[commutation - 3])

#define ALL_COMMUTATION_BITS      (LOW_COMMUTATION_BITS | HIGH_COMMUTATION_BITS)
#define  ALL_COMMUTATION_BITS_OFF (~ALL_COMMUTATION_BITS)
#define HIGH_COMMUTATION_BITS_OFF (~HIGH_COMMUTATION_BITS)

// the pin change mask the zero crossing signals attached to
#define MOTOR_ZC_MSK PCMSK2
#define MOTOR_ZC_PC _BV(2)

#define ZC_PINA PIND2
#define ZC_PINB PIND3
#define ZC_PINC PIND4

#define _ZC_PINA _BV(ZC_PINA)
#define _ZC_PINB _BV(ZC_PINB)
#define _ZC_PINC _BV(ZC_PINC)

#define ALL_ZC_PINS (_ZC_PINA | _ZC_PINB | _ZC_PINC)

const byte zero_crossing_pin[6] = {_ZC_PINB,
                                   _ZC_PINC,
                                   _ZC_PINA,
                                   _ZC_PINB,
                                   _ZC_PINC,
                                   _ZC_PINA};

// clear the zero-crossing detection interrupts
#define disable_zero_crossing_detection() {\
  MOTOR_ZC_MSK &= ~ALL_ZC_PINS; \
  PCIFR |= MOTOR_ZC_PC; \
}

// set the pullup resistors on the zero crossing pins
#define zc_initialize() {\
  disable_zero_crossing_detection(); \
  PCICR |= MOTOR_ZC_PC; \
  pinMode(ZC_PINA, INPUT); digitalWrite(ZC_PINA, 1); \
  pinMode(ZC_PINB, INPUT); digitalWrite(ZC_PINB, 1); \
  pinMode(ZC_PINC, INPUT); digitalWrite(ZC_PINC, 1); \
}
                                 
/**
 * poles - number of poles, really only used for rpm()
 * speed_pin - the analog pin to read desired speed from
 */
Motor::Motor(int poles, int speed_pin) {
  this->poles = poles;
  this->speed_pin = speed_pin;
  
  reset();
}

// Pin Change Interrupt 2 is used for zero crossing detection
//   The channel to watch is enabled during next_commutation()
ISR(PCINT2_vect) {
#ifdef DIAG_ZC_INTERRUPT
  raise_diag();
#endif
  motor.zero_crossing_interrupt();
#ifdef DIAG_ZC_INTERRUPT
  drop_diag();
#endif
}

ISR(TIMER1_COMPB_vect) {
#ifdef DIAG_COMMUTATION_INTERRUPT
  raise_diag();
#endif
  motor.next_commutation();
#ifdef DIAG_COMMUTATION_INTERRUPT
  drop_diag();
#endif
}

void Motor::initialize_timers() {
  // Timer1 is used for commutation timing
  
  // Timer 1 is used for commutation timing. It is reset on each zero crossing interrupt. COMPB is used to 
  // to time the commutation.
  disable_timer1_compb();
  TCCR1A = 0;                // normal mode
  TCCR1B = PRESCALE_BITS;
}

void Motor::reset() {
  noInterrupts();
  
  pinMode(speed_pin, INPUT);

  MOTOR_DDR |= ALL_COMMUTATION_BITS; // configure commutation pins as output
  MOTOR_PORT &= ALL_COMMUTATION_BITS_OFF;

  zc_initialize();

  last_even = last_odd = 0;
  auto_phase_shift = false;

  direction = 1;
  sensing = false;
  phase_shift = 0;
  
  interrupt_count = 0;
  commutation = 5;
  commutation_period = commutation_period_accumulator = 0;

  initialize_timers();

  pwm_initialize(0);
  
  inputs_sum = 0;
  inputs_idx = 0;
  for (int x = 0; x < 16; x++) {
    inputs[x] = 0;
  }
  
  interrupts();
}

/* Ramp up table of (power_level, commutation_period, delay) tuples */
const unsigned int RAMP_UP[][2] = {{3906, 25},  //hardcoded
                          {0, 0}};

void Motor::start() {

  reset();

  // next commutation on next timer1 tick
  OCR1B = 0;
  TCNT1 = 0xFFFF;
  enable_timer1_compb();

  sensing = true;
  
  while (!sensing) {
    pwm_set_level(0);
    reset();

    // align
    pwm_set_level(PWM_LEVELS);
    next_commutation();
    delay(2000);
    
    // ramp up
    pwm_set_level(PWM_LEVELS);
    int _delay = 5500;
    while (!sensing) {
      next_commutation();
      delayMicroseconds(_delay);
      if (_delay > 2000) {
        _delay -= 1;
      }
      noInterrupts();
      if (interrupt_count > 10000) {
        sensing = true;
      }
      interrupts();
      
    }
    
    // Ooops, got to the end of startup and not sensing...start over
  }
  
  pwm_start();
}



void Motor::zero_crossing_interrupt() {
#ifdef DIAG_ZC
  raise_diag();
#endif

  if (sensing) {

    unsigned int tcnt1 = TCNT1;
    
    //reset the counter and set the overflow timer for 1/2 the measured time since we're 1/2 way through the phase 
    TCNT1 = 0; // todo don't reset tcnt since it can't be shared between motors - support tick counts greater than 16 bit using overflow

    disable_zero_crossing_detection();
    
    OCR1B = (tcnt1 >> 1) + phase_shift;  //hardcoded. average even odd
    enable_timer1_compb(); // next_commutation

    // this is only needed for commutation based rpm monitoring, there
    // may be other better means to monitor rpm (shaft position sensors)
    commutation_period_accumulator += tcnt1;
    if (commutation == 0) {
      commutation_period = commutation_period_accumulator;
      commutation_period_accumulator = 0;
    }
    
    if (commutation & 1) {
      last_odd = tcnt1 - last_commutation;
    } else {
      last_even = tcnt1 - last_commutation;
    }

  }
  interrupt_count++;
#ifdef DIAG_ZC
  drop_diag();
#endif
}

void Motor::next_commutation() {
  
  last_commutation = TCNT1;
  
  disable_timer1_compb();

  // stop the pwm bit flipping
  pwm_stop();

  register byte port = MOTOR_PORT;

//////////////////////////////////////////////////////////////
// advance the commutation
//////////////////////////////////////////////////////////////
  commutation += direction;
  if (commutation==6) {
    commutation = 0;
#ifdef DIAG_COMMUTATION_CYCLE
    raise_diag();
#endif
  } else if (commutation==255) {
    commutation = 5;
#ifdef DIAG_COMMUTATION_CYCLE
    raise_diag();
#endif
  }
  
  // precalculate per-commutation values for the pwm interrupts
  _commutation = commutation_bits[commutation];

  //start watching for zero crossing on idle phase // hardcoded
  MOTOR_ZC_MSK |= zero_crossing_pin[commutation];

//////////////////////////////////////////////////////////////
// apply new commutation
//////////////////////////////////////////////////////////////

  // Turn off everything that isn't in the new commutation 
  MOTOR_PORT = port &= (ALL_COMMUTATION_BITS_OFF | _commutation);
  deadtime_delay();
  if (pwm_level) { //only turn on the new commutation if there is power
    MOTOR_PORT = port |= _commutation;
  }

//////////////////////////////////////////////////////////////
// Set up PWM
//////////////////////////////////////////////////////////////

  // PWM schemes refer to those in this paper that succinctly describes a bunch
  // http://www.drivetechinc.com/articles/SW_BLDCAC5.PDF
  
  // scheme 0: 2 quadrant -- switch either high or low
  //pwm_set_mask(_commutation & HIGH_COMMUTATION_BITS);
  //-- or --
  //pwm_set_mask(_commutation & LOW_COMMUTATION_BITS);

  // scheme 1: 4 quadrant simultaneous -- switch high and low simultaneously
  pwm_set_mask(_commutation);

#ifdef COMPLEMENTARY_SWITCHING
  // scheme 2: 4 quandrant, simultaneous, complementary
  pwm_set_mask_off(anti_commutation(commutation));
#endif

  pwm_start();

#ifdef DIAG_COMMUTATION_CYCLE
  drop_diag();
#endif
}

unsigned int Motor::speed_control() {

  if (!sensing) {
    return 0;
  }
  
//  pwm_set_level(map(analogRead(speed_pin), 0, 1024, 0, PWM_LEVELS));
//  delay(150);
//  return 0;
  
  // how fast should we go
  int input = analogRead(speed_pin);
  inputs_sum -= inputs[inputs_idx];
  inputs[inputs_idx] = input;
  inputs_sum += input;
  inputs_idx = (inputs_idx + 1) & 0b1111;

#ifdef CONTROL_RPM
  int _desired_rpm = map(inputs_sum, 0, 1024 * 16, 600, 9000); //hardcoded, timer1 prescaling sensitive  
//  int _desired_rpm = map(input, 0, 1024, 600, 9000); //hardcoded, timer1 prescaling sensitive  
  noInterrupts();
  desired_rpm = _desired_rpm; 
  int _commutation_period = commutation_period;
  interrupts();
  
  int delta = desired_rpm - rpm(_commutation_period);
  
#else
  unsigned int desired_commutation_period = map(inputs_sum, 0, 1024 * 16, 60000, 600);  //hardcoded, timer1 prescaling sensitive
  noInterrupts();
  int _commutation_period = commutation_period;
  interrupts();

  int delta = _commutation_period - desired_commutation_period;
//  Serial.print("input: "); Serial.print(input);
//  Serial.print( "desire_commutation_period: ") ; Serial.print(desired_commutation_period);
//  Serial.println();
#endif

  // adjust power level accordingly  //hardcoded
  if (delta > 0) { 
    pwm_set_level(pwm_level + 1);
  } else if (delta < -0) {
    byte _pwm_level  pwm_level;
    if (_pwm_level) {
      pwm_set_level(pwm_level - 1);
    }
  }

  int _phase_shift = auto_phase_shift ? - (_commutation_period * 0.25 / 6.0) : motor.phase_shift;
  noInterrupts();
  phase_shift = _phase_shift;
  interrupts();
  
  return (unsigned int)(_commutation_period * MICROSECONDS_PER_TICK);
}

unsigned int Motor::rpm() {
  if (sensing) {
    //calculate it from the commmutation period
    noInterrupts();
    unsigned int __commutation_period = commutation_period;
    interrupts();
    return rpm(__commutation_period);
  } else {
    return 0;
  }
}



