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
  //int dir = 1;
  unsigned int _delay;
  
//  Serial.print("pwm_level,rpm,phase_shift"); Serial.println();
  do {
    _delay = motor.speed_control();
    if (_delay > 10000) {
      delay(_delay / 1000);
    } else if (_delay > 0) {
      delayMicroseconds(_delay);
    }

//    if (++mult == 1) {
//      mult = 0;
//
//
//      // phase debugging
////      noInterrupts();
////      int phase_shift = motor.phase_shift += dir;
////      int over = motor.commutation_period * 0.45;
////      int under = motor.commutation_period * 0.35;
////      interrupts();
////      if (phase_shift <= -over) {
////        dir = 1;
////      } else if (phase_shift >= under) {
////        dir = -1;
////      }
//
//      
//      noInterrupts();
//      int period = motor.commutation_period;
//      interrupts();
//      
//      // Speed Control Monitor
//      //Serial.print("\tdesired: "); Serial.print(desired_commutation_period);
//      //Serial.print(",period:"); Serial.print(period); 
//      Serial.print(pwm_level); Serial.print("\t");
//      Serial.print(motor.rpm());  Serial.print("\t");
//      Serial.print(phase_shift); Serial.print("\t");
//      //if (motor.sensing) { Serial.print("\tsensing"); }
//      Serial.println();
//
//    }

  } while (_delay >= 0);
}


