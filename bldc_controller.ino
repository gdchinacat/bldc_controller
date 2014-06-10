#include <math.h>
#include "bldc_controller.h"
#include "Motor.h"


/** the motor */
Motor motor(4, A5);

void initialize_timer1() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0xFFFF;            //schedule it for next tick
  TCCR1B = PRESCALE;
  TIMSK1 |= (1<<TOIE1);      // enable timer overflow interrupt
  interrupts();
}

void setup() {

  initialize_timer1();

  pinMode(diag_pin, OUTPUT);
      
  //Serial.begin(115200);
  //Serial.setTimeout(5);
  
}

ISR(TIMER1_OVF_vect)
{
  
  TCNT1 = 0xFFFF;  //interrupt on next tick

  motor.tick();  
}

void loop() {
  
  motor.start();
  
  int mult = 0;
  
  unsigned int _delay;
  do {
    _delay = motor.speed_control();
    delayMicroseconds(_delay);

    if (++mult == 20) {
      mult = 0;
    
      // Speed Control Monitor
      //Serial.print(" desired: "); Serial.print(desired_commutation_period);
      //Serial.print(" period:"); Serial.print(commutation_period); 
      //Serial.print(" delta:"); Serial.print(delta); 
      //Serial.print(" power_level:"); Serial.print(power_level);
      //Serial.print(" rpm:"); Serial.print(motor.rpm());
      //if (motor.sensing) { Serial.print(" sensing"); }
      //Serial.println();

    }
  } while (_delay > 0);
}


