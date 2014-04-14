#include "Arduino.h"
#include "bldc_controller.h"
#include "Motor.h"

extern Motor motor;
void __commutation_intr() {
  motor.commutation_intr();
}

Motor::Motor(int poles, int rpm) {
  this->poles = poles;
  _commutation = 5;
  sensing = false;
  set_rpm(rpm);
  attachInterrupt(commutation_interrupt, __commutation_intr, HIGH);
}

void Motor::tick() {
  ticks++;
  if (!sensing && ticks > _commutation_period) {
    //raise_diag();
    if (_commutation_period > 0) {
      next_commutation();
      if(_commutation_period < 90) {  // TODO - detect interrupts
        sensing = true;
      }
    }
    ticks = 0;
  }
}

void Motor::commutation_intr() {
  if (sensing) {
    next_commutation();
    _commutation_period = ticks;
    ticks = 0;
  }
}

void Motor::set_rpm(unsigned int rpm) {
  if (!sensing) {
    this->rpm = rpm;
    _commutation_period = commutation_period_from_rpm(rpm);
  }
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


