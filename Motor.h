
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor {
  
  public:
    Motor(int poles);
    void tick();
    void set_rpm(unsigned int rpm);
    int commutation_period_from_rpm(unsigned int rpm);
    void commutation_intr();
    int rpm();
    void reset();
    
    int poles;
    unsigned int _rpm;
    byte _commutation;         // the current commutation
    int _commutation_period;  // in ticks
    int ticks;
    int interpolation_ticks;
    boolean sensing;
};

#endif
