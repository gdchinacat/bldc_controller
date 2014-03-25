int sync = 12;
int sync_bit = 1 << (sync - 8);
#define PWM_PER_PHASE 10

/*
 pins in PORTD are used [2,3], [4,5], [6,7], first is high side, second is low side
 */
int commutation_bits[6] = {B10000100,
                           B00100100,
                           B01100000,
                           B01001000,
                           B00011000,
                           B10010000};

int timer1_delay;
void initialize_timer1() {
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  // Set timer1_delay to the correct value for our interrupt interval
  timer1_delay = 65535;
  
  TCNT1 = timer1_delay;
  //TCCR1B |= (1<<CS11 | 1<<CS10);     // (overdriven) 64x prescaler (250khz)
  TCCR1B |= (1<<CS12);                 // 256x prescaler (62.5khz)
  //TCCR1B |= (1<<CS12 | 1<<CS10);                 // 1024x prescaler (15.625khz)
  TIMSK1 |= (1<<TOIE1);    // enable timer overflow interrupt
  interrupts();              // enable all interrupts
}

void setup() {
  initialize_timer1();
  
  pinMode(sync, OUTPUT);
  
  DDRD |= B11111100;  // pins 2-7 as output
  PORTD &= B00000011; // pins 2-7 LOW
}

static int commutation = 5; // the current phase of commutation
static int pwm_count;
static volatile boolean advance = false;

ISR(TIMER1_OVF_vect)        // interrupt service routine 
{
  TCNT1 = timer1_delay;
  start_cycle();
  if (--pwm_count == 0) {
    advance = true;
    pwm_count = PWM_PER_PHASE * 2;  // each one toggles it, so double the count to count both on/off
    // turn everything off at end of phase
    PORTD ^= commutation_bits[commutation];
  } else {
    if (0 == --pwm_count & 1) {
      PORTD |= commutation_bits[commutation];
    } else {
      PORTD ^= commutation_bits[commutation];
    }  
  }
}

void loop() {
  
  while(true) {
    if (6 == ++commutation) {
      commutation = 0;
      //start_cycle();
    }

    PORTD |= commutation_bits[commutation];
    while (!advance) {}
    advance = false;   
  }
}

__inline__ void start_cycle() {
  
  // bring sync _HIGH at start of commutation cycle for diagnostics (oscilloscope trigger)
  PORTB |= sync_bit;
  PORTB ^= sync_bit;
}

