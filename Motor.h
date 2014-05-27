
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor {
  
  public:
    Motor(int poles);
    void tick();
    void commutation_intr();
    int rpm();
    void reset();
    void start();
    
    int poles;
    int _commutation_period;  // in ticks, public for speed control
    boolean sensing;
    byte phase_ticks;

  private:
    byte _commutation;         // the current commutation
    int ticks;
    void set_commutation_period(int period);
    byte _phase_ticks;

};

#endif
