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
  attachInterrupt(commutation_interrupt, __commutation_intr, CHANGE);
}

void Motor::reset() {
  motor.sensing = false;
  set_rpm(0);
  _commutation = 5;
  sensing = false;
}

void Motor::tick() {
  drop_diag();
  ticks++;
  interpolation_ticks++;
  //if (!sensing) {
    if (ticks > _commutation_period) {
      //raise_diag();
      if (_commutation_period > 0) {
        next_commutation();
        if(_commutation_period < 150) {  // TODO - detect interrupts
          //sensing = true;
        }
      }
      ticks = 0;
    }
  //} else {
    if (_commutation_period > 200) {
      sensing = false;
    }
  }
}

void Motor::commutation_intr() {
  raise_diag();
  if (sensing) {
    // full sensing
    //next_commutation();
    //_commutation_period = ticks;         // full sensing mode  
    
    // interpolation - record the period and trigger next_commutation on next tick
    if (interpolation_ticks != -1) {
      ticks = _commutation_period = interpolation_ticks / 3;
      interpolation_ticks = -1; //skip next one, it's dirty from this...
    } else {
        interpolation_ticks = 0;
    }
  } else {
    interpolation_ticks = 0;
  }
}

int Motor::rpm() {
  if (sensing) {
    //calculate it
    return (int)(60 * 1000000.0 / (_commutation_period * TIMER_MICROS * poles * 6));
  } else {
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
  } else {
    rpm = min(MAX_RPM, rpm);
    float freq = (((float)rpm) * poles * 6) / 60;
    float period = (1000000.0 / freq);
    return (int)(period / TIMER_MICROS);
  }
}


