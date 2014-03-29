#include "Arduino.h"
#include "bldc_controller.h"
#include "Motor.h"

Motor::Motor(int poles, int rpm) {
  this->poles = poles;
  set_rpm(rpm);
  }

void Motor::tick() {
  // runs as interrupt, be quick
  if (--_ticks <= 0) {
    next_commutation = true;
    _ticks = _ticks_per_phase; 
  }
  // TODO - inspect the power and rotation and .... I think I'll play with a real motor first
}

void Motor::set_rpm(int rpm) {
  if (rpm <= 0) {
    this->rpm = 0;
    _ticks = _ticks_per_phase = ~0;  // run as slowly as possible without overflow or if for special non-runtime state
  } else {
    this->rpm = rpm;
    float commutations_freq = (((float)rpm) * poles * 6) / 60;
    float commutation_period = (1000000.0 / commutations_freq);
    _ticks_per_phase = commutation_period / TIMER_MICROS;
    if (_ticks_per_phase < _ticks) {
      _ticks = _ticks_per_phase;
    }
  }
 
}
