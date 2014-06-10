#include "Arduino.h"
#include "bldc_controller.h"
#include "Motor.h"

/*
 * The PWM bitmasks. 16 levels is 6.5% per level, which isn't
 * ideal, but the controller will maintain a constant speed,
 * effectively switching between power levels as appropriate.
 *
 * Strictly speaking, this isn't pulse width modulation, it's
 * a ...
 */

const byte pwm_bits[17][2] = {{B00000000, B00000000},
                              {B00000000, B00000001},
                              {B00000001, B00000001},
                              {B00000100, B00100001},
                              {B00010001, B00010001},
                              {B00010010, B01001001},
                              {B00100101, B00100101},
                              {B00101010, B01010101},
                              {B01010101, B01010101},
                              {B01010101, B10101011},
                              {B01011011, B01011011},
                              {B01101101, B10110111},
                              {B01110110, B11110111},
                              {B01111011, B11011111},
                              {B01111110, B11111111},
                              {B01111111, B11111111},
                              {B11111111, B11111111}};


/*
 pins in PORTD are used [8,9], [10,11], [12,13], first is high side, second is low side
 */
const byte commutation_bits[6] = {B100001,
                                  B001001,
                                  B011000,
                                  B010010,
                                  B000110,
                                  B100100};
                            
#define  ALL_COMMUTATION_BITS_OFF B11000000
#define HIGH_COMMUTATION_BITS_OFF B11101010

extern Motor motor;
void __commutation_intr() {
  motor.commutation_intr();
}

Motor::Motor(int poles, int commutation_interrupt, int speed_pin) {
  this->poles = poles;
  this->speed_pin = speed_pin;

  reset();
  DDRB |= B111111;  // pins 8-13 as output
  PORTB &= B11000000; // pins 8-13 LOW
  pinMode(speed_pin, INPUT);
  attachInterrupt(commutation_interrupt, __commutation_intr, RISING);
}

void Motor::reset() {
  noInterrupts();
  direction = 1;
  sensing = false;
  phase_shift = 0;
  
  interrupt_count = 0;
  commutation = commutation_bits[0];
  commutation = 5;
  _commutation_ticks = 0;
  
  _pwm_bits = 0;
  pwm_level = 0;
  pwm_level = pwm_bits[power_level];  // start off
  pwm_ticks = 15;

  interrupts();
}

void Motor::set_power(byte _power_level) {
  power_level = constrain(_power_level, 0, 16);
  const byte* __power_level = pwm_bits[power_level];
  noInterrupts();
  pwm_level = __power_level;
  pwm_ticks = 15;
  interrupts();
}

void Motor::next_commutation() {
  //raise_diag();

  // Turn off all the bits to avoid short circuit while the 
  // high side is turning on and the low side is turning off.
  PORTB &= ALL_COMMUTATION_BITS_OFF; 
  commutation += direction;
  if (commutation==6) {
    raise_diag();
    commutation = 0;
  } else if (commutation==255) {
    raise_diag();
    commutation = 5;
  }
  _commutation = commutation_bits[commutation];
}

/* Ramp up table of (power_level, commutation_period, delay) tuples */
const unsigned int RAMP_UP[][2] = {{3906, 25},
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
                          {1302, 25},
                          {1250, 25},
                          {1201, 25},
                          {1157, 25},
                          {1116, 25},
                          {1077, 25},
                          {1041, 25},
                          {1008, 25},
                          {976, 25},
                          {946, 25},
                          {919, 25},
                          {892, 25},
                          {868, 25},
                          {844, 25},
                          {822, 25},
                          {801, 25},
                          {781, 25},
                          {762, 25},
                          {744, 25},
                          {726, 25},
                          {710, 25},
                          {694, 25},
                          {679, 25},
                          {664, 25},
                          {651, 25},
                          {637, 25},
                          {625, 25},
                          {612, 25},
                          {600, 25},
                          {589, 25},
                          {578, 25},
                          {568, 25},
                          {558, 25},
                          {548, 25},
                          {538, 25},
                          {529, 25},
                          {520, 25},
                          {512, 25},
                          {504, 25},
                          {496, 25},
                          {488, 25},
                          {480, 25},
                          {473, 25},
                          {466, 25},
                          {459, 25},
                          {452, 25},
                          {446, 25},
                          {440, 25},
                          {434, 25},
                          {428, 25},
                          {422, 25},
                          {416, 25},
                          {411, 25},
                          {405, 25},
                          {400, 25},
                          {395, 25},
                          {390, 25},
                          {385, 25},
                          {381, 25},
                          {376, 25},
                          {372, 25},
                          {367, 25},
                          {363, 25},
                          {359, 25},
                          {355, 25},
                          {351, 25},
                          {347, 25},
                          {343, 25},
                          {339, 25},
                          {336, 25},
                          {332, 25},
                          {328, 25},
                          {325, 25},
                          {322, 25},
                          {318, 25},
                          {315, 25},

                          {0, 0}};

void Motor::start() {
  while (!sensing) {
    set_power(0);
    reset();

    // align
    set_power(16);
    delay(2000);
    
    // ramp up
    for (const unsigned int *c = RAMP_UP[0]; c[0] && !sensing; c+=2) {
      int period = c[0];
      int _delay = c[1];
      //Serial.print("period: "); Serial.print(period); Serial.print(" delay: "); Serial.print(_delay); Serial.println();
      noInterrupts();
      if (!sensing) {
        _commutation_ticks = period;
      }
      interrupts();
      delay(_delay);
    }
    
    // Ooops, got to the end of startup and not sensing...start over
  }
}

void Motor::tick() {
  drop_diag();
  ticks++;
  if (_commutation_ticks && ticks >= _commutation_ticks) {
    next_commutation();
    if (sensing) {
      _commutation_ticks = 0;  // interrupt sets next commutation
    } else {
      ticks = 0;
      if (interrupt_count > 5) {
        _commutation_ticks = 0;  // interrupt sets next commutation
        sensing = true;
      }
    }
  } else if (ticks > 5000) {
    sensing = false;
  }

  // PWM
  //raise_diag();
  pwm_ticks++;
  if (pwm_ticks == 16) {
    _pwm_bits = pwm_level[0];
    pwm_ticks=0;
  } else if (pwm_ticks == 8) {
    _pwm_bits = pwm_level[1];
  } else {
    _pwm_bits >>= 1;
  }
  
  if (_pwm_bits & 1) {
    PORTB |= _commutation;
  } else {
    // soft switching, doesn't work (well) with my sensorless circuit b/c 
    // BEMF on high phase drops to ground and confuses the zero crossing circuitry
    //PORTB &= HIGH_COMMUTATION_BITS_OFF; 

    // hard switching, powered phases BEMF will be valid during pwm down
    PORTB &= ALL_COMMUTATION_BITS_OFF;
  }
  //drop_diag();
  //raise_diag();
}

void Motor::commutation_intr() {
  //raise_diag();
  interrupt_count++;
  if (sensing) {
    // set the next commutation time if it isn't already set
    if (!_commutation_ticks) { // the first zero crossing in a step is used
      commutation_period = ticks;
      _commutation_ticks = (ticks >> 1) + phase_shift; // zero crossing is 1/2 way through step
      ticks = 0;
    }
  }
}

unsigned int Motor::speed_control() {
    // how fast should we go
    int desired_commutation_period = map(analogRead(speed_pin), 0, 1024, 300, 20);
    
    // how fast are we going 
    noInterrupts();
    int _commutation_period = commutation_period;
    interrupts();
    
    //adjust power level accordingly
    int delta = _commutation_period - desired_commutation_period;
    if (delta > 0) {
      set_power(power_level + 1);
    } else if (delta < -1) {
      set_power(power_level - 1);
    }

    return sensing ? commutation_period * TIMER_MICROS : 0;
}

unsigned int Motor::rpm() {
  if (sensing) {
    //calculate it from the commmutation period
    noInterrupts();
    unsigned int __commutation_period = commutation_period;
    interrupts();
    return (unsigned int)(60 * 1000000.0 / (__commutation_period) / ( TIMER_MICROS * poles * 6));
  } else {
    return 0;
  }
}


