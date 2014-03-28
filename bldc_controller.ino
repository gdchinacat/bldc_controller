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

int all_commutation_bits_off;

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
  for (int i=5; i>=0; i--) {
    all_commutation_bits_off |= commutation_bits[i];
  }
  all_commutation_bits_off = ~all_commutation_bits_off;

  pinMode(diag_pin, OUTPUT);
  
  DDRD |= B11111100;  // pins 2-7 as output
  PORTD &= B00000011; // pins 2-7 LOW

  initialize_timer1();
    
  Serial.begin(9600);
  Serial.setTimeout(3);
}

ISR(TIMER1_OVF_vect)
{
  static int commutation = 5;
  static int pwm_count = 0;
  static int cycles = 1;
  
  TCNT1 = 0xFFFF;  //interrupt on next tick
  
  //raise_diag(); // diagnostic trigger on every timer  
  drop_diag();

  // motor simulator piggy-backs off this timer...real motor should be interrupt driven
  motor.tick();
  
  // commutation
  if (next_commutation) {
    //if (!(pwm_count & 1)) raise_diag();  // diagnostic trigger on every commutation
    next_commutation = false;

    // turn everything off
    PORTD &= all_commutation_bits_off;
    
    if (6 == ++commutation) {
      //raise_diag(); // diagnostic trigger on every commutation
      commutation = 0;
      if (cycles-- == 0) {
        raise_diag();
        //if (1 == pwm_count) { raise_diag(); }; // diagnostic trigger on every revolution that coincides with a pwm cycle
        cycles = motor.poles;
      }
    }
    
    if (pwm_on and pwm_count>pwm_off) {
      PORTD |= commutation_bits[commutation];
    } 

  }
  
  // PWM
   // TODO - better time distribution of on/off, ok for rpms < 2500
  pwm_count--;
  if (0 == pwm_count) { // start a new pulse
    //raise_diag();
    pwm_count = pwm_on + pwm_off;
    if (pwm_on) {
      PORTD |= commutation_bits[commutation];
    } 
  } else if (pwm_off && pwm_count == pwm_off) {
    PORTD &= all_commutation_bits_off;
  } 

}

void set_power(int on, int off) {
  noInterrupts();
  pwm_on = on;
  pwm_off = off;
  interrupts();
}

void loop() {
  set_power(10, 0);
  motor.set_rpm(5000);
  while (true) {
    int _on = read("Enter power_on: ");
    int _off = read("Enter power_off: ");
    set_power(_on, _off);      
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

int read(char* prompt) {
  Serial.print(prompt);
  while (Serial.available() < 1) {};
  int input = Serial.parseInt();
  Serial.print(input);
  Serial.print("\n");
  return input;
}

