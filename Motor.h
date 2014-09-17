
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

// What should the diagnostics pin be raised?
//#define DIAG_ZC_INTERRUPT
//#define DIAG_ZC
#define DIAG_COMMUTATION_CYCLE
//#define DIAG_COMMUTATION_INTERRUPT

#define SPEED_PIN A5
//#define COMPLEMENTARY_SWITCHING
#ifdef COMPLEMENTARY_SWITCHING
#error This is not yet implemented to use the pwm module, extend it to support off then on semantics
#endif

// lots of hard coded timer stuff here....
#define disable_timer1_compb();  (TIMSK1 &= ~_BV(OCIE1B));
#define enable_timer1_compb();  (TIMSK1 |= _BV(OCIE1B));

class Motor {
  
  public:
    Motor(int poles, int speed_pin);
    void start();
    unsigned int rpm();

    void next_commutation(); // advance to the next commutation
    void pwm_on();  // called when pwm should be on 
    void pwm_off(); // called when pwm should be off
    unsigned int speed_control();
    void zero_crossing_interrupt();  // called from global
    
  private:
    int poles;       // number of pole pairs
    int speed_pin;   // pin the speed potentiometer is connected to
    char direction;
    
    boolean sensing; // is commutation driven by interrupts (rather than timing)

    // commutation
    int interrupt_count;              // # of interrupts since last reset()
    int phase_shift;                  // # of ticks to shift commutation by
    unsigned int commutation_period;  // last commutation period, in timer1 ticks
    byte commutation;                 // current commutation step
    byte _commutation;                // the current commutation power bits
    
    void reset();
    void initialize_timers();
   
};

#endif
