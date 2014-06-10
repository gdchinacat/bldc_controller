
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor {
  
  public:
    Motor(int poles, int commutation_pin, int speed_pin);
    void tick();
    unsigned int speed_control();
    void commutation_intr();
    unsigned int rpm();
    void reset();
    void start();
    
  private:
    int poles;
    int speed_pin;

    boolean sensing;

    // power
    byte power_level;
    const byte* pwm_level;
    byte pwm_ticks;  // tracks when it's time to pull a new set of pwm_bits in
    byte _pwm_bits;
    
    // commutation
    int phase_shift;
    unsigned int commutation_period;
    byte commutation;
    byte _commutation;         // the current commutation
    unsigned int ticks;               // number of ticks since last interrupt
    unsigned int _commutation_ticks;  // number of ticks when next next commutation should occur
    
    void next_commutation();
    void set_power(byte);
    void set_commutation_period(unsigned int period);
};

#endif
