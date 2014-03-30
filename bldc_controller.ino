#include "bldc_controller.h"
#include "Motor.h"

// motor->controller signal to advance to the next commutation
volatile bool next_commutation = true;

#define diag_pin_bit (1 << (diag_pin - 8))


/*
 pins in PORTD are used [2,3], [4,5], [6,7], first is high side, second is low side
 */
byte commutation_bits[6] = {B10000100,
                            B00100100,
                            B01100000,
                            B01001000,
                            B00011000,
                            B10010000};
#define ALL_COMMUTATION_BITS_OFF B11

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
byte* pwm_level = pwm_bits[0];  // start off

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
    
  //Serial.begin(9600);
  //Serial.setTimeout(3);
}

ISR(TIMER1_OVF_vect)
{
  drop_diag();
  //raise_diag(); // diagnostic trigger on every timer  
  static byte commutation = 5;
  static byte _commutation;
  static byte pwm_bits = 0;
  static byte pwm_ticks = 15;  // tracks when it's time to pull a new set of pwm_bits in

  
  TCNT1 = 0xFFFF;  //interrupt on next tick
  
  // motor simulator piggy-backs off this timer...real motor should be interrupt driven
  //raise_diag();
  motor.tick();
  //drop_diag();
  
  // commutation
  //raise_diag();
  if (next_commutation) {
    //raise_diag();  // diagnostic trigger on every commutation

    // This is done here, rather than in the PWM, since the low side transistors
    // need to be turned off, and given time to fall (typically nanoseconds, we're
    // working at the us scale here), before turning the high side transistors on.
    // This shouldn't be an issue since the low and high are 1 commutation cycle
    // away, but I'm planning to support skipping commutations (hey...the real
    // world is a nasty place) and this is future proofing for that.
    // PERF - 2 cycles for  IN and AND because I don't want to clobber the rx/tx bits
    PORTD &= ALL_COMMUTATION_BITS_OFF;
    
    next_commutation = false;
    if (commutation--==0) {
      raise_diag(); // diagnostic trigger on every commutation cycle
      commutation = 5;
    }
    _commutation = commutation_bits[commutation];  // don't do this indexing every tick
    
  }
  //drop_diag();
  
  // PWM
  // TODO - need to turn off low before turning on high to avoid short circuit
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
    PORTD |= _commutation;
  } else {
    PORTD &= ALL_COMMUTATION_BITS_OFF;
  }
  //drop_diag();
  //raise_diag();
}

void set_power(byte power_level) {
  pwm_level = pwm_bits[constrain(power_level, 0, 16)];
}

void loop() {
  int power = 0; 
  char dir = 1;

  while (true) {
    power += dir;
    if ((power == 0) || (power == 16)) {
      dir = ~dir + 1;
    }
    long rpm = 5000 + power * 30 ;
    //Serial.println(rpm);
    motor.set_rpm(rpm);
    set_power(power);
    delay(350);
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

