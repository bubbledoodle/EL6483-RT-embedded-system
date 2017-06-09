#ifndef LED_PWM_H
#define LED_PWM_H

#include "stdint.h"

// initialize LEDs as timer-based (PWM) pins
void init_LED_PWM();

// set dutycycle of LED i (i=0,1,2,3) to dutycycle (specified as percentage -- 0 to 100)
void set_LED_dutycycle(uint32_t i , uint32_t duty_cycle);


#endif
