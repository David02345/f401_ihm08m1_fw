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

typedef struct
{
  uint32_t cnt_before_output;
  uint32_t cnt_first_isr;

  uint16_t raw_before_a;
  uint16_t raw_before_b;
  uint16_t raw_before_c;

  uint32_t jeoc_before_output;
  uint32_t adc_irq_pending_before_output;

  uint32_t cycles_to_first_isr;
  bool first_isr_seen;

} motor_diag_timing_t;

void motorCurrentDiagGetTiming(motor_diag_timing_t *timing);
uint32_t motorCurrentDiagGetMoeAfterOff(void);
void motorCurrentDiagStart(void);
uint32_t motorCurrentNoiseGetCount(void);
bool motorCurrentNoiseGetSample(uint32_t index, motor_abc_u16_t *raw, motor_abc_f_t *current);
void motorCurrentDiagGetTripSnapshot(motor_abc_u16_t *raw, motor_abc_f_t *current);

// 분리 예정
bool motorCurrentDiagIsDone(void);
uint32_t motorCurrentDiagGetSampleCount(void);
uint32_t motorCurrentDiagGetStartupTripSample(void);
void motorCurrentDiagGetCurrentMinMax(motor_abc_f_t *i_min, motor_abc_f_t *i_max);

uint32_t motorCurrentLTestGetCount(void);
bool motorCurrentLTestGetSample(uint32_t index, float *id, float *iq, float *vd, float *vq, float *theta_e);
// 분리 예정


#endif

#endif /* SRC_COMMON_HW_INCLUDE_MOTOR_H_ */
