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
#include "hall.h"


bool motorInit(void);
void motorStart(void);
void motorStop(void);

void motorLowSpeedTask(void);
void motorControlUpdate(void);

void motorSetFault(motor_fault_t fault);
void motorClearFault(void);

bool motorGetMonitor(motor_monitor_t *monitor);

void motorSetCurrentReference(float id_ref, float iq_ref);

#endif /* SRC_COMMON_HW_INCLUDE_MOTOR_H_ */
