#ifndef SRC_COMMON_HW_INCLUDE_MOTOR_H_
#define SRC_COMMON_HW_INCLUDE_MOTOR_H_

#include "hw_def.h"
#include "motor_types.h"
#include "pid.h"
#include "pwm.h"
#include "adc.h"
#include "foc.h"
#if _USE_HALL_SENSOR
#include "hall.h"
#elif _USE_ENCODER
#include "encoder.h"
#endif



bool motorInit(void);
void motorStart(void);            /* gated by MOTOR_ALLOW_POWER_STAGE */
void motorStop(void);
void motorLowSpeedTask(void);
void motorCurrentLoop(float id_ref, float iq_ref, float theta_e);
void motorControlUpdate(void);    /* ADC injected callback, current FOC */
void motorSetFault(motor_fault_t fault);
void motorClearFault(void);
bool motorGetMonitor(motor_monitor_t *monitor);
void motorSetCurrentReference(float id_ref, float iq_ref);
void motorSetSpeedReference(float w_ref);
motor_state_t motorGetState(void);
motor_fault_t motorGetFault(void);

#endif /* SRC_COMMON_HW_INCLUDE_MOTOR_H_ */
