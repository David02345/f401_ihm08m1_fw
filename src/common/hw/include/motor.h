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


typedef enum
{
  MOTOR_STATE_IDLE,
  MOTOR_STATE_READY,
  MOTOR_STATE_OPEN_LOOP,
  MOTOR_STATE_FAULT,
} motor_state_t;

typedef enum
{
  MOTOR_FAULT_NONE = 0,
  MOTOR_FAULT_INIT_FAIL,
  MOTOR_FAULT_ADC_OFFSET_FAIL,
  MOTOR_FAULT_VBUS_LOW,
  MOTOR_FAULT_ADC_REGULAR_FAIL,
  MOTOR_FAULT_OVERCURRENT,
  MOTOR_FAULT_OPEN_LOOP_FAIL,
} motor_fault_t;

bool motorInit(void);
void motorStart(void);
void motorStop(void);
void motorOpenLoopStart(void);
void motorControlUpdate(void);
void motorLowSpeedTask(void);

motor_state_t motorGetState(void);
motor_fault_t motorGetFault(void);
float motorGetVbus(void);

#endif /* SRC_COMMON_HW_INCLUDE_MOTOR_H_ */
