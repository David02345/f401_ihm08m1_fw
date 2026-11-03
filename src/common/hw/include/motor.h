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
#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && (CURRENT_FOC_BENCH_ENABLE == 1U)

void motorFocBenchGetStats(
    uint32_t *count,
    uint32_t *total_cycles,
    uint32_t *max_cycles);

#endif

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


#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && (CURRENT_ADC_RAW_SCAN_ENABLE == 1U)


typedef struct
{
    uint32_t sample_count;

    uint16_t raw_min[3];
    uint16_t raw_max[3];

    float current_min[3];
    float current_max[3];

    uint32_t spike_count[3];

    uint32_t first_spike_sample;
    uint16_t first_spike_raw[3];

    // 새로 추가: 연속 스파이크 통계
    uint32_t spike_streak[3];
    uint32_t spike_max_streak[3];
    uint32_t spike_adjacent_pairs[3];

    // 새로 추가: 0.8A 초과 측정값 통계
    uint32_t oc_sample_count[3];
    uint32_t oc_streak[3];
    uint32_t oc_max_streak[3];

    uint32_t first_oc_sample;
    uint16_t first_oc_raw[3];

} motor_adc_raw_scan_t;


void motorAdcRawScanGetStats(motor_adc_raw_scan_t *stats);

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_ADC_RAW_SCAN_ENABLE == 1U)

void motorAdcRawScanGetSplitStats(
    uint16_t raw_min[2][3],
    uint16_t raw_max[2][3],
    uint32_t spike_count[2][3]);

#endif

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_ADC_RAW_SCAN_ENABLE == 1U) && \
    (CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U)

bool motorAdcPwmOnScanIsDone(void);

#endif

#endif


void motorCurrentDiagGetTiming(motor_diag_timing_t *timing);
uint32_t motorCurrentDiagGetMoeAfterOff(void);
void motorCurrentDiagStart(void);
uint32_t motorCurrentNoiseGetCount(void);
bool motorCurrentNoiseGetSample(uint32_t index, motor_abc_u16_t *raw, motor_abc_f_t *current);
void motorCurrentDiagGetTripSnapshot(motor_abc_u16_t *raw, motor_abc_f_t *current);
void motorCurrentDiagGetSnapshot50ms(motor_abc_f_t *snapshot);
void motorCurrentDiagGetAverage(motor_abc_f_t *avg);
// 분리 예정
bool motorCurrentDiagIsDone(void);
uint32_t motorCurrentDiagGetSampleCount(void);
uint32_t motorCurrentDiagGetStartupTripSample(void);
void motorCurrentDiagGetCurrentMinMax(motor_abc_f_t *i_min, motor_abc_f_t *i_max);

uint32_t motorCurrentLTestGetCount(void);
bool motorCurrentLTestGetSample(uint32_t index, float *id, float *iq, float *vd, float *vq, float *theta_e);
#if CURRENT_ROTATION_TEST_ENABLE
bool motorCurrentLTestGetHallSample(uint32_t index, uint8_t *hall_state, int8_t *hall_sector, uint8_t *hall_valid);
#endif
// 분리 예정


#endif

#endif /* SRC_COMMON_HW_INCLUDE_MOTOR_H_ */
