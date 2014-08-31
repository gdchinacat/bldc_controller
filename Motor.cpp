#include "Arduino.h"
#include "bldc_controller.h"
#include "Motor.h"
extern "C" {
#include "PWM.h"
}

/*
 pins in PORTD are used [8,9], [10,11], [12,13], first is high side, second is low side
 */
const byte commutation_bits[6] = {B100001, // hardcoded
                                  B001001,
                                  B011000,
                                  B010010,
                                  B000110,
                                  B100100};
                                  
const byte zero_crossing_pin[6] = {1<<6, //hardcoded
                                   1<<7,
                                   1<<5,
                                   1<<6,
                                   1<<7,
                                   1<<5};

#define  ALL_COMMUTATION_BITS_OFF B11000000
#define HIGH_COMMUTATION_BITS_OFF B11101010
#define HIGH_COMMUTATION_BITS     B00010101
#define LOW_COMMUTATION_BITS      B00101010

Motor::Motor(int poles, int speed_pin) {
  this->poles = poles;
  this->speed_pin = speed_pin;
  
  reset();
}

/** the motor */
Motor motor(4, A5);

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
  
  /* hardcoded */
  pinMode(speed_pin, INPUT);

  DDRB |= B111111;  // pins 8-13 as output
  PORTB &= B11000000; // pins 8-13 LOW

  PCMSK2 = 0; // no phase selected for zero crossing detection
  PCICR |= 1 << PCIE2;
  /* end hardcoded */

  direction = 1;
  sensing = false;
  phase_shift = 0;
  
  interrupt_count = 0;
  commutation = commutation_bits[0];
  commutation = 5;

  initialize_timers();

  pwm_initialize(0);
  
  interrupts();
}

/* Ramp up table of (power_level, commutation_period, delay) tuples */
const unsigned int RAMP_UP[][2] = {{3906, 25},  //hardcoded
                          {3472, 25},
                          {3125, 25},
                          {2840, 25},
                          {2604, 25},
                          {2403, 25},
                          {2232, 25},
                          {2083, 25},
                          {1953, 25},
                          {1838, 25},
                          {1736, 25},
                          {1644, 25},
                          {1562, 25},
                          {1488, 25},
                          {1420, 25},
                          {1358, 25},
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
    PCMSK2 = 0; //turn off zero crossing interrupts //hardcoded
    PCIFR |= 0b100; //clear pending interrupt //hardcoded

    // Timer1 kept counting since it was reset and triggered last commutation
    unsigned int tcnt1 = TCNT1;
    TCNT1 = 0xFFFF - tcnt1 + phase_shift;

    if (commutation & 1) {
      commutation_period = tcnt1 << 1;
    }

    enable_timer1_overflow(); // next_commutation
  }
  //drop_diag();
}

void Motor::next_commutation() {
  //raise_diag();

  // stop the pwm bit flipping
  pwm_stop();

  register byte portb = PORTB;

  // Turn off all the bits to avoid short circuit while the 
  // high side is turning on and the low side is turning off.
  PORTB = portb &= ALL_COMMUTATION_BITS_OFF; //hardcoded

  // advance the commutation
  commutation += direction;
  if (commutation==6) {
    commutation = 0;
  } else if (commutation==255) {
    commutation = 5;
  }
  if (commutation==0) {
    raise_diag();
  }  
  // precalculate per-commutation values for the pwm interrupts
  _commutation = commutation_bits[commutation];

  // start timer for first half of commutation until zero crossing is detected
  TCNT1 = 0;

  //start watching for zero crossing on idle phase // hardcoded
  PCMSK2 = zero_crossing_pin[commutation];
  disable_timer1_overflow();

#ifdef COMPLEMENTARY_SWITCHING

  // TODO - optimize these bit operations to precalculate as much as possible
  // complementary switching, turn off everything that's not in the commutation.
  // This will turn off the complementary low without effecting the current low
  // or high. However, it will not turn anything on.
  PORTB = portb &= (ALL_COMMUTATION_BITS_OFF | _commutation);  //hardcoded
  deadtime_delay();
#endif
  
  PORTB = portb |= _commutation;              //hardcoded

  // PWM schemes refer to those in this paper that succinctly describes a bunch
  // http://www.drivetechinc.com/articles/SW_BLDCAC5.PDF
  
  // scheme 0: 2 quadrant -- switch either high or low
  //pwm_set_mask(_commutation & HIGH_COMMUTATION_BITS); // speed regulation isn't as consistent
  //-- or --
  //pwm_set_mask(_commutation & LOW_COMMUTATION_BITS);

  // scheme 1: 4 quadrant simultaneous -- switch high and low simultaneously
  pwm_set_mask(_commutation);
  

#ifdef COMPLEMENTARY_SWITCHING
  // complementary switching/unipolar switching
  // shift and mask to turn the complementary low transistor on
  // I've seen different literature on complementary switching that
  // says to invert the high and low.
  deadtime_delay();
  PORTB = portb |= ((_commutation & 0b10101) << 1); //hardcoded
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


