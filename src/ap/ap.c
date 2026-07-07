/*
 * ap.c
 *
 *  Created on: 2026. 5. 26.
 *      Author: YDG
 */
#include "ap.h"



void apInit(void)
{

}

void apMain(void)
{
  uint32_t pre_time_motor_slow = millis();
  delay(1000);

  motorStart();
  delay(500);

  motorOpenLoopStart();


  while(1)
  {
    if(millis() - pre_time_motor_slow >= 10)
    {
      pre_time_motor_slow = millis();
      motorLowSpeedTask();
    }
  }
}
