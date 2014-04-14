#include <math.h>
#include "bldc_controller.h"
#include "Motor.h"

/*
 pins in PORTD are used [8,9], [10,11], [12,13], first is high side, second is low side
 */
byte commutation_bits[6] = {B100001,
                            B001001,
                            B011000,
                            B010010,
                            B000110,
                            B100100};
#define ALL_COMMUTATION_BITS_OFF B11000000


byte commutation = commutation_bits[0];

static int desired_commutation_period;

/*
 * The PWM bitmasks. 16 levels is 6.5% per level, which isn't
 * ideal, but the controller will maintain a constant speed,
 * effectively switching between power levels as appropriate.
 */
 
byte pwm_bits[17][2] = {{B00000000, B00000000},
                        {B00000000, B00000001},
                        {B00000001, B00000001},
                        {B00000100, B00100001},
                        {B00010001, B00010001},
                        {B00010010, B01001001},
                        {B00100101, B00100101},
                        {B00101010, B01010101},
                        {B01010101, B01010101},
                        {B01010101, B10101011},
                        {B01011011, B01011011},
                        {B01101101, B10110111},
                        {B01110110, B11110111},
                        {B01111011, B11011111},
                        {B01111110, B11111111},
                        {B01111111, B11111111},
                        {B11111111, B11111111}};


/*
 * pwm_level - 0 is off, 17 is full on, >17 on
 */
byte power_level = 0;
byte* pwm_level = pwm_bits[power_level];  // start off

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

  desired_commutation_period = motor.commutation_period_from_rpm(0);
  
  pinMode(diag_pin, OUTPUT);
  pinMode(pot_pin, INPUT);
  
  DDRB |= B111111;  // pins 8-13 as output
  PORTB &= B11000000; // pins 8-13 LOW

  initialize_timer1();
    
  Serial.begin(9600);
  Serial.setTimeout(5);
  
}

static byte _commutation = 5;

void next_commutation() {
  raise_diag();
  if (++_commutation==6) {
    _commutation = 0;
  }
  commutation = commutation_bits[_commutation];
}

ISR(TIMER1_OVF_vect)
{
  drop_diag();
  //raise_diag(); // diagnostic trigger on every timer  
  static byte _commutation = 0;
  static byte pwm_bits = 0;
  static byte pwm_ticks = 15;  // tracks when it's time to pull a new set of pwm_bits in
  
  TCNT1 = 0xFFFF;  //interrupt on next tick

  motor.tick();

  // Turn off all the bits to avoid short circuit while the 
  // high side is turning on and the low side is turning off.
  if (_commutation != commutation) {
    PORTB &= ALL_COMMUTATION_BITS_OFF;
    _commutation = commutation;
    return;
  }
  
  // PWM
  //raise_diag();
  pwm_ticks++;
  if (pwm_ticks == 16) {
    pwm_bits = pwm_level[0];
    pwm_ticks=0;
  } else if (pwm_ticks == 8) {
    pwm_bits = pwm_level[1];
  } else {
    pwm_bits >>= 1;
  }
  if (pwm_bits & 1) {
    PORTB |= _commutation;
  } else {
    PORTB &= ALL_COMMUTATION_BITS_OFF;
  }
  //drop_diag();
  //raise_diag();
}

__inline__ void set_power(byte _power_level) {
  power_level = constrain(_power_level, 0, 16);
  pwm_level = pwm_bits[_power_level];
}


void loop() {
  
  //desired_commutation_period = motor.commutation_period_from_rpm(0);

  set_power(16);
  for (int rpm = 100; rpm < 500; rpm+=3) {
    Serial.println(rpm);
    if (motor.set_rpm(rpm)) {
      break;
    }
    delay(90);
  }
  
  //set_power(11);
  int input = 1;
  
  while (input) {
    Serial.println(motor._commutation_period * TIMER_MICROS);
    /*
    int rpm_input = map(analogRead(pot_pin), 0, 1024, 150, 3000);
    int delta = rpm_input - motor.rpm;
    delta = min(max(delta, -1), 1);
    motor.set_rpm(motor.rpm  + delta);
*/
    if (Serial.available()) {
      input = Serial.parseInt();
      Serial.print("power:"); Serial.print(input);
      Serial.print(" rpm:"); Serial.println(motor.rpm);
        //motor.set_rpm(input);
        set_power(input);
        //commutation_to_skip = input - 1;
    }
  }
  motor.disengage();
  set_power(16);
  commutation = commutation_bits[0];
  delay(1000); 

/*
    if (Serial.available()) {
      int c = Serial.parseInt();
      set_commutation(c);
      Serial.println(c);
      Serial.print("enter commutation: "); 
    }
*/

/*
    desired_commutation_period = motor.commutation_period_from_rpm(5000 + 2500 - throbber);

    int load = (throbber + 5)/1000;
    motor.loop(load); //load varies between 0 and 5

    // speed control
    int delta = motor._commutation_period - desired_commutation_period;
    if (delta > 0) {
      set_power(power_level + 1);
    } else if (delta < 0) {
      set_power(power_level - 1);
    }
*/

    // Speed Control Monitor
    //Serial.print(" desired: "); Serial.print(desired_commutation_period);
    //Serial.print(" period:"); Serial.print(motor._commutation_period); 
    //Serial.print(" load:"); Serial.print(load); 
    //Serial.print(" delta:"); Serial.print(delta); 
    //Serial.print(" power_level:"); Serial.print(power_level); 
    //Serial.print(" rpm: "); Serial.print(motor.rpm);
    //Serial.println();

/*
    if (Serial.available()) {
      //int power = Serial.parseInt();
      //Serial.print("power:"); Serial.println(power);
      //set_power(power);
      
      
      int rpm = Serial.parseInt();
      Serial.print("rpm:"); Serial.println(rpm);
      desired_commutation_period = motor.commutation_period_from_rpm(rpm);
    }
*/
 
}

int read(const char* prompt) {
  Serial.print(prompt);
  while (Serial.available() < 1) {};
  int input = Serial.parseInt();
  Serial.print(input);
  Serial.print("\n");
  return input;
}

