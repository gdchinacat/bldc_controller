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

// TICKS_PER_MICROSECOND is 16 (Mhz) / multiplier PRESCALE
#define PRESCALE 8
#define TICKS_PER_MICROSECOND (16 / PRESCALE)
#define MICROSECONDS_PER_TICK (1.0 / TICKS_PER_MICROSECOND)

void Motor::initialize_timers() {
  // Timer1 is used for commutation timing
  
  // Timer 1 is used for commutation timing. It is reset on each zero crossing interrupt. COMPB is used to 
  // to time the commutation.
  disable_timer1_compb();
  TCCR1A = 0;                // normal mode
  TCCR1B = _BV(CS11);        // 8x prescale, high resolution provides better commutation accuracy (I think...), but too high and you overlow.
}

void Motor::reset() {
  noInterrupts();
  
  pinMode(speed_pin, INPUT);

  MOTOR_DDR |= ALL_COMMUTATION_BITS; // configure commutation pins as output
  MOTOR_PORT &= ALL_COMMUTATION_BITS_OFF;

  zc_initialize();

  direction = 1;
  sensing = false;
  phase_shift = 0;
  
  interrupt_count = 0;
  commutation = 5;

  initialize_timers();

  pwm_initialize(0);
  
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
  
  while (!sensing) {
    pwm_set_level(0);
    reset();

    // align
    pwm_set_level(PWM_LEVELS);
    next_commutation();
    delay(2000);
    
    // ramp up
    for (const unsigned int *c = RAMP_UP[0]; c[0] && !sensing; c+=2) {
      //int period = c[0];
      int _delay = c[1];
      //Serial.print("period: "); Serial.print(period); Serial.print(" delay: "); Serial.print(_delay); Serial.println();
      noInterrupts();
      next_commutation();
      if (interrupt_count > 5) {
        sensing = true;
        interrupts();
        break;
      }
      interrupts();
      delay(_delay);
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
    disable_zero_crossing_detection();

    unsigned int tcnt1 = TCNT1;
    
    //reset the counter and set the overflow timer for 1/2 the measured time since we're 1/2 way through the phase 
    TCNT1 = 0; // todo don't reset tcnt since it can't be shared between motors - support tick counts greater than 16 bit using overflow
    OCR1B = (tcnt1 >> 1) + phase_shift;  //hardcoded
    enable_timer1_compb(); // next_commutation

    if (commutation == 0) { // they aren't accurate enough and it messes up the speed control
      commutation_period = tcnt1;
    }

  }
  interrupt_count++;
#ifdef DIAG_ZC
  drop_diag();
#endif
}

void Motor::next_commutation() {
  
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
  
  int desired_commutation_period = map(input, 0, 1024, 10000, 500);  //hardcoded, timer1 prescaling sensitive

//  Serial.print("input: "); Serial.print(input);
//  Serial.print( "desire_commutation_period: ") ; Serial.print(desired_commutation_period);
//  Serial.println();
  
  // how fast are we going 
  noInterrupts();
  int _commutation_period = commutation_period;
  interrupts();

  // adjust power level accordingly  //hardcoded
  int delta = _commutation_period - desired_commutation_period;
  if (delta > 0) {
    pwm_set_level(pwm_level + 1);
  } else if (delta < 0) {
    byte _pwm_level  pwm_level;
    if (_pwm_level) {
      pwm_set_level(pwm_level - 1);
    }
  }
  return _commutation_period; // not exactly sure what this means, need to work out the math
}

unsigned int Motor::rpm() {
  if (sensing) {
    //calculate it from the commmutation period
    noInterrupts();
    unsigned int __commutation_period = commutation_period;
    interrupts();
    return (unsigned int)(60 * 1000000.0 / (__commutation_period) / (MICROSECONDS_PER_TICK * poles * 6));
  } else {
    return 0;
  }
}


