/*
 * hw.c
 *
 *  Created on: 2026. 5. 26.
 *      Author: YDG
 */


#include "hw.h"


void hwInit(void)
{
  bspInit();
  cliInit();

  rtcInit();
  resetInit();
  ledInit();
  usbInit();
  uartInit();
  flashInit();

#ifdef _USE_MOTOR_FOC
  pwmInit();
  focInit();
  adcInit();
#endif
}
