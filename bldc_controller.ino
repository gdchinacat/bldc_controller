#include <math.h>
#include "bldc_controller.h"
#include "Motor.h"

extern "C" {
#include "PWM.h"
}

extern Motor motor;

void setup() {

  pinMode(diag_pin, OUTPUT);
  
  //Serial.begin(115200);
  //Serial.setTimeout(5);
  
}

void loop() {

  motor.start();

  int mult = 0;
  unsigned int _delay;
  do {
    _delay = motor.speed_control();
    if (_delay > 10000) {
      delay(_delay / 1000);
    } else if (_delay > 0) {
      delayMicroseconds(_delay);
    }


//    if (++mult == 10) {
//      mult = 0;
//      
//      noInterrupts();
//      int period = motor.commutation_period;
//      interrupts();
//      
//      // Speed Control Monitor
//      //Serial.print(" desired: "); Serial.print(desired_commutation_period);
//      Serial.print(" period:"); Serial.print(period); 
//      Serial.print(" power_level:"); Serial.print(pwm_level);
//      //Serial.print(" rpm:"); Serial.print(motor.rpm());
//      //if (motor.sensing) { Serial.print(" sensing"); }
//      Serial.println();
//
//    }

  } while (_delay >= 0);
}


