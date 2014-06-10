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
                            
#define ALL_COMMUTATION_BITS_OFF B11000000
#define HIGH_COMMUTATION_BITS_OFF B11101010
#define LOW_COMMUTATION_BITS B101010

extern Motor motor;
void __commutation_intr() {
  motor.commutation_intr();
}

Motor::Motor(int poles, int commutation_interrupt, int speed_pin) {
  this->poles = poles;
  this->speed_pin = speed_pin;

  reset();
  pinMode(speed_pin, INPUT);
  attachInterrupt(commutation_interrupt, __commutation_intr, RISING);
}

void Motor::reset() {
  noInterrupts();
  sensing = false;
  phase_shift = 0;
  
  commutation = commutation_bits[0];
  _commutation = 5;
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

  if (++_commutation==6) {
    raise_diag();
    _commutation = 0;
  }
  commutation = commutation_bits[_commutation];
}

void Motor::set_commutation_period(unsigned int period) {
  noInterrupts();
  if (!sensing) {
    _commutation_ticks = period;
  }
  interrupts();
}

void Motor::start() {
  set_power(16);
  reset();
  delay(2000);
  for (int period = 2250; !sensing; period -=281) {
    set_commutation_period(period);
    delayMicroseconds(2500);
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
      if (commutation_period < 550) { // TODO - use interrupts
        _commutation_ticks = 0;  // interrupt sets next commutation
        sensing = true;
      }
    }
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
    PORTB |= commutation;
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
  // set the next commutation time if it isn't already set
  if (!_commutation_ticks) { // the first zero crossing in a step is used
    commutation_period = ticks;
    _commutation_ticks = (ticks >> 1) + phase_shift; // zero crossing is 1/2 way through step
    ticks = 0;
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

    return commutation_period * TIMER_MICROS;
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


