/*
 * encoder.c
 *
 *  Created on: 2026. 7. 28.
 *      Author: YDG
 */

#include "encoder.h"


static volatile uint32_t count = 0;
static volatile float speed_e = 0.0f;
static volatile float speed_m = 0.0f;
static volatile float angle_e = 0.0f;
static volatile float angle_m = 0.0f;


bool encoderInit(void)
{
  bool ret = true;

  return ret;
}

uint32_t encoderGetCount(void)
{
  return count;
}

float encoderGetElectricalSpeed(void)
{


  return speed_e;
}

float encoderGetMechanicalSpeed(void)
{


  return speed_m;
}

float encoderGetElectricalAngle(void)
{


  return angle_e;
}

float encoderGetMechanicalAngle(void)
{


  return angle_m;
}
