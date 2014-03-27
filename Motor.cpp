#include "bldc_controller.h"
#include "Motor.h"

extern volatile int pwm_on;

#define STALLED -1

Motor::Motor(int poles, int rpm) {
  this->poles = poles;
  set_rpm(rpm);
}


void Motor::tick() {
  // runs as interrupt, be quick
  if (_ticks_per_phase != STALLED) {
    if (--_ticks <= 0) {
      advance();
    }
  }
  
  // TODO - inspect the power and rotation and .... I think I'll play with a real motor first
}

void Motor::advance() {
  next_commutation = true;
  _ticks = _ticks_per_phase;
}

void Motor::set_rpm(int rpm) {
  if (rpm <= 0) {
    this->rpm = rpm;
    _ticks_per_phase = STALLED;
  } else {
    this->rpm = rpm;
    float commutations_freq = (((float)rpm) * poles * 6) / 60;
    float commutation_period = (1000000.0 / commutations_freq);
    _ticks_per_phase = commutation_period / TIMER_MICROS;
    advance();
  }
 
}
