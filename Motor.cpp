#include "Arduino.h"
#include "bldc_controller.h"
#include "Motor.h"

extern Motor motor;
void __commutation_intr() {
  motor.commutation_intr();
}

Motor::Motor(int poles) {
  this->poles = poles;
  reset();
  attachInterrupt(commutation_interrupt, __commutation_intr, RISING);
}

void Motor::reset() {
  sensing = false;
  noInterrupts();
  _commutation = 5;
  _commutation_period = -1;
  interrupts();
}

void Motor::set_commutation_period(int period) {
  noInterrupts();
  _commutation_period = period;
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
  if (!sensing) {
    if (_commutation_period > 0 && ticks > _commutation_period) {
      next_commutation();
      ticks = 0;
      if (_commutation_period < 550) { // TODO - use interrupts
        sensing = true;
      }
    }
  }
}

void Motor::commutation_intr() {
  //raise_diag();
  if (sensing) {
    if (ticks < 5) { // ignore interrupts during the width of the interrupt signal
      return;
    }
    next_commutation();
    _commutation_period = ticks;
    ticks = 0;
  }
}

int Motor::rpm() {
  if (sensing) {
    //calculate it from the commmutation period
    noInterrupts();
    int __commutation_period = _commutation_period;
    interrupts();
    return (int)(60 * 1000000.0 / (__commutation_period) / ( TIMER_MICROS * poles * 6));
  } else {
    return -1;
  }

}



