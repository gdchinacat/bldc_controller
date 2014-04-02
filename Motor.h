
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor {
  
  public:
    Motor(int poles, int rpm);
    void tick();
    void loop(byte load); // load is a byte <= 16 indicating zero to full load (stalled)
    void set_rpm(unsigned int rpm);
    int commutation_period_from_rpm(unsigned int rpm);
    
    int poles;
    unsigned int rpm;
    byte _commutation;         // the current commutation
    int _commutation_period;  // in ticks
    int ticks;
};

#endif
