int sync = 12;

/*
 pins in PORTD are used [2,3], [4,5], [6,7], first is high side, second is low side
 */
/*
int commutation_bits[6] = {B10000100,
                           B00100100,
                           B01100000,
                           B01001000,
                           B00011000,
                           B10010000};
                           */

int commutation_bits[6] = {B11111100,
                           B11111100,
                           B10000000,
                           B10000000,
                           B10011000,
                           B10010000};


void setup() {
  pinMode(sync, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  DDRD |= B11111100; // pins 2-7 as output
  PORTD &= B00000011;      // pins 2-7 LOW
}

__inline__ void start_cycle() {
  // bring sync _HIGH at start of commutation cycle for diagnostics (oscilloscope trigger)
  digitalWrite(sync, HIGH);
  digitalWrite(sync, LOW);
  
  static int led_toggle = LOW;
  digitalWrite(LED_BUILTIN, led_toggle);
  led_toggle = ~led_toggle;
}

void loop() {
  int _commutation = 5;
  
  while(true) {
    _commutation++;
    if (6 == _commutation) {
      _commutation = 0;
      start_cycle();
    }
    
    PORTD = commutation_bits[_commutation];

    //delayMicroseconds(300);
  }
}
