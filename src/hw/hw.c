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
  //cliInit();

  //rtcInit();
  //resetInit();
  ledInit();
  //usbInit();
  uartInit();
  //flashInit();

  //motorInit();
}
