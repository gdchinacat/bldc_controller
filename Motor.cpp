#include "Arduino.h"
#include "bldc_controller.h"
#include "Motor.h"
extern "C" {
#include "PWM.h"
}

/** the motor */
Motor motor(4, A5);

#define MOTOR_PORT PORTB
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
const byte commutation_bits[6] = {A | C_,  // short (long)
                                  A | B_,  // long
                                  C | B_,  // short
                                  C | A_,  // long (long)
                                  B | A_,  // short
                                  B | C_}; // long

#define ALL_COMMUTATION_BITS      (LOW_COMMUTATION_BITS | HIGH_COMMUTATION_BITS)
#define  ALL_COMMUTATION_BITS_OFF (~ALL_COMMUTATION_BITS)
#define HIGH_COMMUTATION_BITS_OFF (~HIGH_COMMUTATION_BITS)

// the pin change mask the zero crossing signals attached to
#define MOTOR_ZC_MSK PCMSK2
#define MOTOR_ZC_PC _BV(2)

#define ZC_PINA _BV(PIND5)
#define ZC_PINB _BV(PIND6)
#define ZC_PINC _BV(PIND7)

#define ALL_ZC_PINS (ZC_PINA | ZC_PINB | ZC_PINC)

const byte zero_crossing_pin[6] = {ZC_PINB,
                                   ZC_PINC,
                                   ZC_PINA,
                                   ZC_PINB,
                                   ZC_PINC,
                                   ZC_PINA};

// clear the zero-crossing detection interrupts
#define disable_zero_crossing_detection() {\
  MOTOR_ZC_MSK &= ~ALL_ZC_PINS; \
  PCIFR |= MOTOR_ZC_PC; \
}

#define zc_initialize() {\
  disable_zero_crossing_detection(); \
  PCICR |= MOTOR_ZC_PC; \
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
  motor.commutation_intr();
}

ISR(TIMER1_OVF_vect) {
  motor.next_commutation();
}

void Motor::initialize_timers() {
  // Timer1 is used for commutation timing
  
  // Timer 1 is used for commutation timing. It is reset when commutation is advanced and used to time the
  // first half of the commutation step as detected by the zero crossing detector. It then is used to count
  // down the second half of the commutation step until overflow. It doesn't seem possible to use compb
  // since that requires the pin be wired for PWM which conflicts with its use for output.
  TCCR1A = 0;                // normal mode
  TCCR1B = 0;                // disconnected (connected later)
  TCCR1B = _BV(CS11);        // 8x prescale, high resolution provides better commutation accuracy (I think...), but too high and you overlow.
  disable_timer1_overflow();

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

  sensing = true;
  
  TCNT1 = 0xFFFF;
  enable_timer1_overflow(); // next commutation on next timer1 tick
    
  while (!sensing) {
    pwm_set_level(0);
    reset();

    // align
    pwm_set_level(PWM_LEVELS);
    delay(2000);
    
    // ramp up
    for (const unsigned int *c = RAMP_UP[0]; c[0] && !sensing; c+=2) {
      //int period = c[0];
      int _delay = c[1];
      //Serial.print("period: "); Serial.print(period); Serial.print(" delay: "); Serial.print(_delay); Serial.println();
      noInterrupts();
      // todo implement me - not implementing since no good way to test startup...it's busted for other reasons
      interrupts();
      delay(_delay);
    }
    
    // Ooops, got to the end of startup and not sensing...start over
  }
  
  pwm_start();
}

__inline__ void deadtime_delay() {
  // my hunch is clock ticks between the two OUTs is sufficient that this isn't
  // really necessary. It didn't seem to do anything to the warm transistors (which
  // was resolved by not gratuitously turning the transistors off).
  //__asm__("nop\n\t"); //62.5ns deadtime //hardcoded
}


void Motor::commutation_intr() {
  //raise_diag();
  interrupt_count++;
  if (sensing) {
    disable_zero_crossing_detection();

    unsigned int tcnt1 = TCNT1;

    if (commutation & 1) {
      commutation_period = tcnt1 << 1;
    }

    // Timer1 kept counting since it was reset and triggered last commutation
    // todo don't reset tcnt since it can't be shared between motors, use compX instead
    TCNT1 = 0xFFFF - tcnt1 + phase_shift;

    enable_timer1_overflow(); // next_commutation
  }
  //drop_diag();
}

void Motor::next_commutation() {
  //raise_diag();
  
  disable_timer1_overflow();

  // stop the pwm bit flipping
  pwm_stop();

  register byte port = MOTOR_PORT;

//////////////////////////////////////////////////////////////
// advance the commutation
//////////////////////////////////////////////////////////////
  commutation += direction;
  if (commutation==6) {
    commutation = 0;
    raise_diag();
  } else if (commutation==255) {
    commutation = 5;
    raise_diag();
  }
  // precalculate per-commutation values for the pwm interrupts
  _commutation = commutation_bits[commutation];

  // start timer for first half of commutation until zero crossing is detected
  TCNT1 = 0; // todo - can't reset timer counter and have 2 motors share it, use compX instead

  //start watching for zero crossing on idle phase // hardcoded
  MOTOR_ZC_MSK |= zero_crossing_pin[commutation];


//////////////////////////////////////////////////////////////
// apply new commutation
//////////////////////////////////////////////////////////////

  // Turn off everything that isn't in the new commutation 
  MOTOR_PORT = port &= (ALL_COMMUTATION_BITS_OFF | _commutation);
  deadtime_delay();
  MOTOR_PORT = port |= _commutation;

//////////////////////////////////////////////////////////////
// Set up PWM
//////////////////////////////////////////////////////////////

  //hard switching (scheme 1)
  pwm_set_mask(_commutation);
  
  // soft switching (scheme 0) 
  //pwm_set_mask(_commutation & HIGH_COMMUTATION_BITS);
  //pwm_set_mask(_commutation & LOW_COMMUTATION_BITS);

#ifdef COMPLEMENTARY_SWITCHING
  // complementary switching/unipolar switching
  // shift and mask to turn the complementary low transistor on
  // I've seen different literature on complementary switching that
  // says to invert the high and low.
  deadtime_delay();
  MOTOR_PORT = port |= ((_commutation & 0b10101) << 1); //hardcoded
#endif

  pwm_start();

  drop_diag();
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
  int desired_commutation_period = map(input, 0, 1024, 5000, 2750);  //hardcoded, timer1 prescaling sensitive

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
    pwm_set_level(pwm_level - 1);
  }
  return _commutation_period; // not exactly sure what this means, need to work out the math
}

unsigned int Motor::rpm() {
  if (sensing) {
    //calculate it from the commmutation period
    noInterrupts();
    unsigned int __commutation_period = commutation_period;
    interrupts();
    return (unsigned int)(60 * 1000000.0 / (__commutation_period) / (1234  * poles * 6));  // todo - 1234 some prescale factor
  } else {
    return 0;
  }
}


