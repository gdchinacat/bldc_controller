
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor {
  
  public:
    Motor(int poles);
    void tick();
    void commutation_intr();
    unsigned int rpm();
    void reset();
    void start();
    
    int poles;
    boolean sensing;
    int phase_shift;
    unsigned int commutation_period;  // exposed for speed control

  private:
    byte _commutation;         // the current commutation
    unsigned int ticks;               // number of ticks since last interrupt
    unsigned int _commutation_ticks;  // number of ticks when next next commutation should occur
    void set_commutation_period(unsigned int period);
};

#endif
