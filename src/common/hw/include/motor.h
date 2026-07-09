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
  MOTOR_STATE_IDLE             = 0x00U,
  MOTOR_STATE_READY            = 0x01U,
  MOTOR_STATE_OPEN_LOOP        = 0x02U,
  MOTOR_STATE_FAULT            = 0x03U,
} motor_state_t;

typedef enum
{
  MOTOR_FAULT_NONE             = 0x00U,
  MOTOR_FAULT_INIT_FAIL        = 0x01U,
  MOTOR_FAULT_ADC_OFFSET_FAIL  = 0x02U,
  MOTOR_FAULT_VBUS_LOW         = 0x03U,
  MOTOR_FAULT_ADC_REGULAR_FAIL = 0x04U,
  MOTOR_FAULT_OVERCURRENT      = 0x05U,
  MOTOR_FAULT_OPEN_LOOP_FAIL   = 0x06U,
  MOTOR_FAULT_BKIN             = 0x07U,
} motor_fault_t;

bool motorInit(void);
void motorStart(void);
void motorStop(void);
void motorOpenLoopStart(void);
void motorControlUpdate(void);
void motorLowSpeedTask(void);

void motorSetFault(motor_fault_t fault);
void motorClearFault(void);

motor_state_t motorGetState(void);
motor_fault_t motorGetFault(void);
float motorGetVbus(void);


#endif /* SRC_COMMON_HW_INCLUDE_MOTOR_H_ */
