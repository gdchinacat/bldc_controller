
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor {
  
  public:
    Motor(int poles, int rpm);
    void tick();
    boolean set_rpm(unsigned int rpm);
    int commutation_period_from_rpm(unsigned int rpm);
    void commutate(); // a commutation occured
    void _commutate(); //interrupt driven commutation
    void disengage();
    void engage();
    
    int poles;
    unsigned int rpm;
    byte _commutation;         // the current commutation
    int _commutation_period;  // in ticks
    int ticks;
    boolean sensing;
};

#endif
