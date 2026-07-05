/*
 * motor.h
 *
 *  Created on: 2026. 7. 4.
 *      Author: YDG
 */

#ifndef SRC_COMMON_HW_INCLUDE_MOTOR_H_
#define SRC_COMMON_HW_INCLUDE_MOTOR_H_

#include "hw_def.h"
#include "pid.h"
#include "pwm.h"
#include "adc.h"
#include "foc.h"




bool motorInit(void);
void motorStart(void);
void motorStop(void);
void motorOpenLoopStart(void);
void motorControlUpdate(void);
void motorLowSpeedTask(void);

#endif /* SRC_COMMON_HW_INCLUDE_MOTOR_H_ */
