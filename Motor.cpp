#include "Arduino.h"
#include "bldc_controller.h"
#include "Motor.h"

extern unsigned int ticks;
extern byte power_level;

Motor::Motor(int poles, int rpm) {
  this->poles = poles;
  _commutation = 5;
  set_rpm(rpm);
}

void Motor::tick() {
  
  return;
  
  if (--ticks == 0) {
    //raise_diag();
    
    ticks = _commutation_period;

    if (++_commutation==6) {
      raise_diag();
      _commutation = 0;
    } else {
      drop_diag();
    }
    set_commutation(_commutation);
  }
}

void Motor::loop(byte load) {
  int delta = (MAX_RPM / 16.0) * max(0, ((int)power_level - load)) - rpm;
  set_rpm(rpm + delta / power_level);
}

void Motor::set_rpm(unsigned int rpm) {
  this->rpm = rpm;
  _commutation_period = commutation_period_from_rpm(rpm);
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

