
#ifndef bldc_controller_h
#define bldc_controller_h

// motor->controller signal to advance to the next commutation
extern volatile bool next_commutation;

// diag_pin is the pin used for diagnostic signals
#define diag_pin 13

// Use the 256x prescaler for a 62.5khz frequency
#define PRESCALE 1<<CS12;
#define TIMER_FREQ (16000000.0 / 256.0) // period of the timer in microseconds (assumes 16mhz cpu)
//LH - hard coded since 4 byte floats are dropping the precision
#define TIMER_MICROS 16
#endif
