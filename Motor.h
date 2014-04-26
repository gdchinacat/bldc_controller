
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor {
  
  public:
    Motor(int poles);
    void tick();
    void set_rpm(unsigned int rpm);
    int commutation_period_from_rpm(unsigned int rpm); //public for speed control
    void commutation_intr();
    int rpm();
    void reset();
    
    int poles;
    int _commutation_period;  // in ticks, public for speed control
    boolean sensing;

  private:
    unsigned int _rpm;
    byte _commutation;         // the current commutation
    int ticks;

};

#endif
