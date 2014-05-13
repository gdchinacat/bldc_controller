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
#define HIGH_COMMUTATION_BITS_OFF B11101010
#define LOW_COMMUTATION_BITS B101010

byte commutation = commutation_bits[0];

static int desired_commutation_period;

/*
 * The PWM bitmasks. 16 levels is 6.5% per level, which isn't
 * ideal, but the controller will maintain a constant speed,
 * effectively switching between power levels as appropriate.
 *
 * Strictly speaking, this isn't pulse width modulation, it's
 * a ...
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

Motor motor(4);

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
  pinMode(pot_pin, INPUT);
  
  DDRB |= B111111;  // pins 8-13 as output
  PORTB &= B11000000; // pins 8-13 LOW

  initialize_timer1();
    
  //Serial.begin(9600);
  //Serial.setTimeout(5);
  
}

static byte _commutation = 5;

void next_commutation() {
  //raise_diag();

  if (++_commutation==6) {
    raise_diag();
    _commutation = 0;
  }
  commutation = commutation_bits[_commutation];

  // Turn off all the bits to avoid short circuit while the 
  // high side is turning on and the low side is turning off.
  PORTB &= ALL_COMMUTATION_BITS_OFF; 

}

static byte pwm_ticks = 15;  // tracks when it's time to pull a new set of pwm_bits in
static byte _pwm_bits = 0;

ISR(TIMER1_OVF_vect)
{
  drop_diag();
  //raise_diag(); // diagnostic trigger on every timer  
  
  TCNT1 = 0xFFFF;  //interrupt on next tick

 motor.tick();
  
  // PWM
  //raise_diag();
  pwm_ticks++;
  if (pwm_ticks == 16) {
    _pwm_bits = pwm_level[0];
    pwm_ticks=0;
  } else if (pwm_ticks == 8) {
    _pwm_bits = pwm_level[1];
  } else {
    _pwm_bits >>= 1;
  }
  
  if (_pwm_bits & 1) {
    PORTB |= commutation;
  } else {
    // soft switching, doesn't work (well) with my sensorless circuit b/c 
    // BEMF on high phase drops to ground and confuses the zero crossing circuitry
    //PORTB &= HIGH_COMMUTATION_BITS_OFF; 

    // hard switching, powered phases BEMF will be valid during pwm down
    PORTB &= ALL_COMMUTATION_BITS_OFF;
  }
  //drop_diag();
  //raise_diag();
}

__inline__ void set_power(byte _power_level) {
  power_level = constrain(_power_level, 0, 16);
  byte* __power_level = pwm_bits[power_level];
  noInterrupts();
  pwm_level = __power_level;
  pwm_ticks = 15;
  interrupts();
}

void loop() {
  
  set_power(16);
  motor.start();
  
  while (motor.sensing) {
    // RPM bases speed control - theoretically more accurate since it can switch between desired commutation
    //                           periods but the floating point math is quite expensive and the resulting
    //                           speed regulation doesn't seem all that much better. While slowing turning
    //                           the knob up it does seem less jerky than commutation period algorithm.
    //int rpm_input = map(analogRead(pot_pin), 0, 1024, 750, 2400);
    //desired_commutation_period = motor.commutation_period_from_rpm(rpm_input);

    // commutation period control - integer math so is significantly faster than the one above.
    desired_commutation_period = map(analogRead(pot_pin), 0, 1024, 300, 30);
    
    noInterrupts();
    int commutation_period = motor._commutation_period;
    interrupts();
    
    int delta = commutation_period - desired_commutation_period;
    if (delta > 0) {
      set_power(power_level + 2);
    } else if (delta < -2) {
      //set_power(0);
      set_power(power_level - 1);
    }
    
    // Speed Control Monitor
    //Serial.print(" rpm_input: "); Serial.print(rpm_input);
    //Serial.print(" desired: "); Serial.print(desired_commutation_period);
    //Serial.print(" period:"); Serial.print(commutation_period); 
    //Serial.print(" delta:"); Serial.print(delta); 
    //Serial.print(" power_level:"); Serial.print(power_level);
    //Serial.print(" rpm:"); Serial.print(motor.rpm());
    //if (motor.sensing) { Serial.print(" sensing"); }
    //Serial.println();

    //wait rouphly more than one commutation period 
    delayMicroseconds(commutation_period * TIMER_MICROS);

  }

  set_power(0);
  motor.reset();
    
}


