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
#include "motor_types.h"


bool motorInit(void);
void motorStart(void);
void motorStop(void);

void motorLowSpeedTask(void);
void motorControlUpdate(void);

void motorSetFault(motor_fault_t fault);
void motorClearFault(void);

bool motorGetMonitor(motor_monitor_t *monitor);

void motorSetCurrentReference(float id_ref, float iq_ref);

motor_state_t motorGetState(void);
motor_fault_t motorGetFault(void);

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP) && (_USE_HALL_OFFSET_CALIBRATION)
bool motorGetHallCalibrationEvent(int8_t *sector, int8_t *direction, float *theta_e);
#endif
#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)

// [수정] Fixed alpha diagnostic
void motorCurrentDiagStart(void);
bool motorCurrentDiagIsDone(void);
uint32_t motorCurrentDiagGetSampleCount(void);
void motorCurrentDiagGetDQAverage(float *id_avg, float *iq_avg);
void motorCurrentDiagGetCurrentMinMax(motor_abc_f_t *i_min, motor_abc_f_t *i_max);
#endif
#endif /* SRC_COMMON_HW_INCLUDE_MOTOR_H_ */
