

#define diag_pin 12

// The timer frequency
#define PRESCALE (B100<<CS10);     // 256x prescaler (62.5khz)

// number of timer ticks per PWM
#define ticks 1

/*
 pins in PORTD are used [2,3], [4,5], [6,7], first is high side, second is low side
 */
int commutation_bits[6] = {B10000100,
                           B00100100,
                           B01100000,
                           B01001000,
                           B00011000,
                           B10010000};

#define diag_pin_bit (1 << (diag_pin - 8))
#define timer_delay (0xFFFF - ticks + 1)

#define CYCLES_PER_ROTATION 4

static volatile int pwm_on = 0;
static volatile int pwm_off = 0;
static volatile long cycle_count = 0;
static volatile boolean next_commutation = true;

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
  pinMode(LED_BUILTIN, OUTPUT);

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
  
  TCNT1 = timer_delay;
  
  //raise_diag(); // diagnostic trigger on every quanta
  drop_diag();  

  pwm_count--;
  if (pwm_off == pwm_count) {
    PORTD &= ~commutation_bits[commutation];
  }
  if (0 >= pwm_count) { // end of pulse
    pwm_count = pwm_on + pwm_off;
    if (next_commutation) {
      //raise_diag();  // diagnostic trigger on every phase
      commutation++;
      if (6 == commutation) {
        cycle_count++;
        raise_diag(); // diagnostic trigger on every commutation
        commutation = 0;
      }
      
      next_commutation = false;
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
  raise_diag();
  unsigned long _delay = 1251;

  Serial.println("Ramping up...");  
  for (int phase=0; _delay > 1250 ; phase ++) {
    _delay = max(1250, 41667L - ((long)phase * 337));  // ramp up, 1250 fast period, 41k slow period, 337 ramp up over 5 revolutions
    delay_(_delay);
    next_commutation = true; // make absolute to support skipping missed phases?
  }
  
  Serial.println("enter delay:");
  while (true) {

    if (Serial.available() > 0) {
      int input = Serial.parseInt();
      input = max(0, min(30000, input));
      _delay = input;
      Serial.println(60000000.0f/((float)(_delay * 6 * CYCLES_PER_ROTATION)));
    }
    delay_(_delay);
    next_commutation = true; // make absolute to support skipping missed phases?
  }

}

__inline__ void raise_diag() {
  PORTB |= diag_pin_bit; // set the 'start of cycle' signal (turned off in loop())
}

__inline__ void drop_diag() {
  PORTB &= ~diag_pin_bit;
}

void delay_(unsigned long _delay) {
  while (_delay > 0) {
    int __delay = min(16383, _delay);
    delayMicroseconds(__delay);
    _delay -= __delay;
  }
}
