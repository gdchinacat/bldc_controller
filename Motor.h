
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor {
  
  public:
    Motor(int poles, int commutation_pin, int speed_pin);
    void start();
    unsigned int rpm();

    void tick(); // called once per timer tick
    unsigned int speed_control();
    void commutation_intr();  // called from global
    
  private:
    int poles;       // number of pole pairs
    int speed_pin;   // pin the speed potentiometer is connected to
    byte direction;
    
    boolean sensing; // is commutation driven by interrupts (rather than timing)

    // power
    byte power_level;       // the current power level
    const byte* pwm_level;  // the current power level bitmask
    byte _pwm_bits;         // pwm bitmask, shifted once per tick
    byte pwm_ticks;         // tracks when it's time to pull a new set of pwm_bits in
    
    // commutation
    int phase_shift;                  // # of ticks to shift commutation by
    unsigned int commutation_period;  // last commutation period, in ticks
    byte commutation;                 // current commutation step
    byte _commutation;                // the current commutation power bits
    unsigned int ticks;               // number of ticks since last interrupt
    unsigned int _commutation_ticks;  // number of ticks when next next commutation should occur
    
    void next_commutation();  // advance to the next commutation step at the next timer tick
    void set_power(byte);     // set the power level
    void set_commutation_period(unsigned int period); // set the commutation period for open loop commutation (sensing == false)
    void reset();
};

#endif
