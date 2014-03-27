#include "bldc_controller.h"
#include "Motor.h"

// motor->controller signal to advance to the next commutation
volatile bool next_commutation = true;

#define diag_pin_bit (1 << (diag_pin - 8))


/*
 pins in PORTD are used [2,3], [4,5], [6,7], first is high side, second is low side
 */
int commutation_bits[6] = {B10000100,
                           B00100100,
                           B01100000,
                           B01001000,
                           B00011000,
                           B10010000};

// _on and _off determine the characteristics of the PWM (percentage and duration of pulse)
volatile int pwm_on = 0;
volatile int pwm_off = 0;

// simulated "motor" for testing
static Motor motor(4, 0);

void initialize_timer1() {
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0xFFFF;          //schedule it for next tick
  TCCR1B = PRESCALE;
  TIMSK1 |= (1<<TOIE1);    // enable timer overflow interrupt
  interrupts();              // enable all interrupts
}

void setup() {
  pinMode(diag_pin, OUTPUT);
  
  DDRD |= B11111100;  // pins 2-7 as output
  PORTD &= B00000011; // pins 2-7 LOW

  initialize_timer1();
    
  Serial.begin(9600);
  Serial.setTimeout(3); // pseudo non-blocking
}

ISR(TIMER1_OVF_vect)
{
  static int commutation = 5;
  static int pwm_count = 0;
  static int cycles = 1;
  
  TCNT1 = 0xFFFF;  //interrupt on next tick
  
  //raise_diag(); // diagnostic trigger on every timer  
  drop_diag();

  motor.tick(); // motor simulator piggy-backs off this timer...

  pwm_count--;
  if (pwm_off == pwm_count) {
    PORTD &= ~commutation_bits[commutation];
  }
  if (0 >= pwm_count) { // end of pulse
    pwm_count = pwm_on + pwm_off;
    //raise_diag();
    if (next_commutation) {
      next_commutation = false;
      //raise_diag();  // diagnostic trigger on every phase
      commutation++;
      if (6 == commutation) {
        //raise_diag(); // diagnostic trigger on every commutation
        commutation = 0;
        cycles--;
        if (cycles == 0) {
          cycles = motor.poles;
          raise_diag(); // diagnostic trigger on every revolution
        }
      }
    }
    if (pwm_on) {
      PORTD |= commutation_bits[commutation];
    }
  } 

}

void set_power(int on, int off) {
  noInterrupts();
  pwm_on = on;
  pwm_off = off;
  interrupts();
}

void loop() {
  set_power(10, 10);
  while (true) {
    if (Serial.available() > 0) {
      int input = Serial.parseInt();
      input = max(0, min(10000, input));
      motor.set_rpm(input);
      Serial.println(motor.rpm);
    }
  }
  
  int inc = 1;
  for (int i=0; ; i += inc) {
    motor.set_rpm(2000);
    motor.set_rpm(100 * i);    //0 to 2000rpm
    //Serial.println(motor.ticks_per_phase());
    set_power(20-i, i);        //100% to 0 %

    long _delay = 250000;
    ///int _delay = (5000 - ((5000 - 1250) / 20) * i); // one commutation
    //long _delay = (5000 - ((5000 - 1250) / 20) * i) * 6; // one cycle
    //long _delay = (5000.0 - ((5000.0 - 1250) / 20) * i) * 6 * motor.poles; // one revolution
    //long _delay = 100 * (5000 - ((5000 - 1250) / 20) * i) * 6 * motor.poles; // X * revolutions
    //long _delay = (i + 1) * (5000.0 - ((5000.0 - 1250) / 20) * i) * 6 * motor.poles; // one revolution for unit of speed (not rpms!)

    delay_(_delay);
    if (i == 0) inc = 1; else if (i == 20) inc = -1;
  }

}

__inline__ void raise_diag() {
  PORTB |= diag_pin_bit; // set the 'start of cycle' signal (turned off in loop())
}

__inline__ void drop_diag() {
  PORTB &= ~diag_pin_bit;
}

void delay_(long _delay) {
  while (_delay > 0) {
    int __delay = min(16383, _delay);
    delayMicroseconds(__delay);
    _delay -= __delay;
  }
}

