
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

// lots of hard coded timer stuff here....
#define disable_timer1_overflow(); (TIMSK1 &= ~_BV(TOIE1));
#define enable_timer1_overflow(); (TIMSK1 |= _BV(TOIE1));

#define disable_timer1_compb();  (TIMSK1 &= ~_BV(OCIE1B));
#define enable_timer1_compb();  (TIMSK1 |= _BV(OCIE1B));

class Motor {
  
  /*
   * TODO -
   *
   *  1) use pin change interrupts with masks to effectively multiplex the outputs 
         of the three phase comparators to implement better filtering and allowing
         soft switching. Also frees up which ports can be used for interrupts.
   */
  public:
    Motor(int poles, int speed_pin);
    void start();
    unsigned int rpm();

    void next_commutation(); // advance to the next commutation
    void pwm_on();  // called when pwm should be on 
    void pwm_off(); // called when pwm should be off
    unsigned int speed_control();
    void commutation_intr();  // called from global
    
  //private:
    int poles;       // number of pole pairs
    int speed_pin;   // pin the speed potentiometer is connected to
    byte direction;
    
    boolean sensing; // is commutation driven by interrupts (rather than timing)

    // power
    byte power_level;       // the current power level
    
    // commutation
    int interrupt_count;              // # of interrupts since last reset()
    int phase_shift;                  // # of ticks to shift commutation by
    unsigned int commutation_period;  // last commutation period, in timer1 ticks
    byte commutation;                 // current commutation step
    byte _commutation;                // the current commutation power bits
    
    void set_power(byte);     // set the power level
    void reset();
    void initialize_timers();
   
};

#endif
