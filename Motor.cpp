#include "Arduino.h"
#include "bldc_controller.h"
#include "Motor.h"

extern Motor motor;
void __commutation_intr() {
  motor.commutation_intr();
}

Motor::Motor(int poles, int commutation_interrupt) {
  this->poles = poles;
  reset();
  attachInterrupt(commutation_interrupt, __commutation_intr, RISING);
}

void Motor::reset() {
  sensing = false;
  noInterrupts();
  _commutation = 5;
  _commutation_ticks = 0;
  phase_shift = 0;
  interrupts();
}

void Motor::set_commutation_period(unsigned int period) {
  noInterrupts();
  if (!sensing) {
    _commutation_ticks = period;
  }
  interrupts();
}

void Motor::start() {
  reset();
  delay(2000);
  for (int period = 2250; !sensing; period -=281) {
    set_commutation_period(period);
    delayMicroseconds(2500);
  }
}

void Motor::tick() {
  //drop_diag();
  ticks++;
  if (_commutation_ticks && ticks >= _commutation_ticks) {
    next_commutation();
    if (sensing) {
      _commutation_ticks = 0;  // interrupt sets next commutation
    } else {
      if (commutation_period < 550) { // TODO - use interrupts
        _commutation_ticks = 0;  // interrupt sets next commutation
        sensing = true;
      } else {
        ticks = 0;
      }
    }
  }
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



