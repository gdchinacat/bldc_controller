#include "Arduino.h"
#include "bldc_controller.h"
#include "Motor.h"

extern byte power_level;

Motor::Motor(int poles, int rpm) {
  this->poles = poles;
  _commutation = 5;
  sensing = false;
  set_rpm(rpm);
}

void Motor::tick() {
  ticks++;
  if (!sensing && ticks > _commutation_period) {
    //raise_diag();
    commutate();
  }
}

boolean Motor::set_rpm(unsigned int rpm) {
  if (!sensing) {
    this->rpm = rpm;
    _commutation_period = commutation_period_from_rpm(rpm);
  }
  return sensing;
}

int Motor::commutation_period_from_rpm(unsigned int rpm) { // in millis
  if (rpm == 0) {
    return -1;
  } else {
    rpm = min(MAX_RPM, rpm);
    float freq = (((float)rpm) * poles * 6) / 60;
    float period = (1000000.0 / freq);
    return (int)(period / TIMER_MICROS);
  }
}

void Motor::commutate() {
  next_commutation();
  ticks = 0;
  if (!sensing && _commutation_period < 600) {
    engage();
  }
}

extern Motor* motor;
void __commutate() {
  motor->commutate();
}

void Motor::_commutate() {
  _commutation_period = ticks;
  commutate();
}

void Motor::engage() {
  sensing = true;
  ticks=0;
  attachInterrupt(commutation_interrupt, next_commutation, CHANGE);
}

void Motor::disengage() {
  detachInterrupt(commutation_interrupt);
  sensing = false;
}
