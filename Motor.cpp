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
  motor.sensing = false;
  set_rpm(0);
  _commutation = 5;
  sensing = false;
}

void Motor::tick() {
  //drop_diag();
  ticks++;
  if (!sensing) {
    if (ticks > _commutation_period) {
      //raise_diag();
      if (_commutation_period > 0) {
        if (_commutation_period < 450) {
          sensing = true;
        }
        next_commutation();
        ticks = 0;
      }
    }
  } 
}

void Motor::commutation_intr() {
  //raise_diag();
  if (sensing) {
    next_commutation();
    _commutation_period = ticks;
    ticks = 0;
  }
}

int Motor::rpm() {
  if (sensing) {
    //calculate it
    int __commutation_period;
    noInterrupts();
    __commutation_period = _commutation_period;
    interrupts();
    return (int)(60 * 1000000.0 / (__commutation_period) / ( TIMER_MICROS * poles * 6));
  } 
  else {
    return _rpm;
  }
}

void Motor::set_rpm(unsigned int rpm) {
  if (!sensing) {
    this->_rpm = rpm;
    _commutation_period = commutation_period_from_rpm(rpm);
  }
}

int Motor::commutation_period_from_rpm(unsigned int rpm) { // in millis
  if (rpm == 0) {
    return -1;
  } 
  else {
    rpm = min(MAX_RPM, rpm);
    float freq = (((float)rpm) * poles * 6) / 60;
    float period = (1000000.0 / freq);
    return (int)(period / TIMER_MICROS);
  }
}



