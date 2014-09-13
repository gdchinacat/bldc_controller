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

//
//    if (++mult == 1) {
//      mult = 0;
//      
//      noInterrupts();
//      int period = motor.commutation_period;
//      interrupts();
//      
//      // Speed Control Monitor
//      //Serial.print("\tdesired: "); Serial.print(desired_commutation_period);
//      Serial.print("\tperiod:"); Serial.print(period); 
//      Serial.print("\tpower_level:"); Serial.print(pwm_level);
//      Serial.print("\trpm:"); Serial.print(motor.rpm());
//      //if (motor.sensing) { Serial.print("\tsensing"); }
//      Serial.println();
//
//    }

  } while (_delay >= 0);
}


