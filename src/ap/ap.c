/*
 * ap.c
 *
 *  Created on: 2026. 5. 26.
 *      Author: YDG
 */
#include "ap.h"



void apInit(void)
{
  uartOpen(_DEF_UART1, 57600);
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

      uartPrintf(_DEF_UART1, "Current Update Count : %d\r\n", adcGetCurrentUpdateCount());
      uartPrintf(_DEF_UART1, "Motor State : %d\r\n", motorGetState());
      uartPrintf(_DEF_UART1, "Motor Fault : %d\r\n", motorGetFault());
      uartPrintf(_DEF_UART1, "Motor VBUS : %d\r\n", motorGetVbus());
    }
  }
}
