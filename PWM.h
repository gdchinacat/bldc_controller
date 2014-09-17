/*
    Copyright (C) 2014 Anthony Hutchinson

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// This is all hardcoded for now -- if you change one thing you probably need to change others as well

// The port that PWM is controlling (PORTx)
#define PWM_PORT PORTB

//PWM_LEVELS defines the number of power levels, and indirectly, the period
//of the PWM. It can be used to trade power resolution with pwm frequency.
#define PWM_LEVELS 64

// The prescale (8x seems to be the lowest reasonable)
#define PWM_TIMER_PRESCALE _BV(CS21)

// Timer2 is used for generating the interrupts used for software based PWM.
#define disable_timer2_overflow(); TIMSK2 &= ~_BV(TOIE2);
#define enable_timer2_overflow();  TIMSK2 |= _BV(TOIE2);

#define disable_timer2_compb();  TIMSK2 &= ~_BV(OCIE2B);
#define enable_timer2_compb();   TIMSK2 |= _BV(OCIE2B);

#define enable_timer2_interrupts();   {TIMSK2 = _BV(TOIE2) | _BV(OCIE2B);}
#define disable_timer2_interrupts();  {TIMSK2 = 0;}

#define timer2_compb_enabled() (TIMSK2 & _BV(OCIE2B))
#define timer2_overflow_enabled() (TIMSK2 & _BV(TOIE2))

// PWM_PIN is the pin the source pwm can be seen on, depends which output compare unit is being used
//#define PWM_PIN 3

void pwm_initialize(byte pwm_mask);
void pwm_start();
void pwm_stop();
void pwm_set_level(byte level);
void pwm_set_mask(byte mask);

#define _PWM_ON();    { PWM_PORT |= pwm_mask;  }
#define _PWM_OFF();   { PWM_PORT &= ~pwm_mask; }

// the current pwm level
#define pwm_level (OCR2B - (256 - PWM_LEVELS))

