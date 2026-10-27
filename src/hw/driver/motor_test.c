/*
 * motor_test.c
 *
 *  Created on: 2026. 10. 27.
 *      Author: YDG
 */
#include "motor_test.h"


static volatile motor_state_t motor_state;
static volatile motor_fault_t motor_fault = MOTOR_FAULT_NONE;

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)

// [수정] ADC ISR 기준 시험 시간 관리
static volatile uint32_t current_diag_sample_count = 0U;
static volatile bool current_diag_active = false;
static volatile bool current_diag_done = false;
static volatile motor_abc_f_t current_diag_i_min;
static volatile motor_abc_f_t current_diag_i_max;
static volatile float current_diag_id_sum = 0.0f;
static volatile float current_diag_iq_sum = 0.0f;
#define CURRENT_STARTUP_SETTLE_SAMPLES  4U

static volatile uint32_t current_startup_count = 0U;


static volatile uint32_t current_diag_tail_count = 0U;

static volatile float current_diag_tail_id_sum = 0.0f;
static volatile float current_diag_tail_iq_sum = 0.0f;

static volatile float current_diag_tail_vd_sum = 0.0f;
static volatile float current_diag_tail_vq_sum = 0.0f;

// [L TEST] 첫 3 ms current response 저장
static volatile float current_l_test_id[CURRENT_L_TEST_SAMPLE_COUNT];
static volatile float current_l_test_iq[CURRENT_L_TEST_SAMPLE_COUNT];
static volatile float current_l_test_theta[CURRENT_L_TEST_SAMPLE_COUNT];
static volatile float current_l_test_vq[CURRENT_L_TEST_SAMPLE_COUNT];
static volatile float current_l_test_vd[CURRENT_L_TEST_SAMPLE_COUNT];

static volatile uint32_t current_l_test_count = 0U;

static volatile motor_abc_u16_t current_noise_raw[CURRENT_NOISE_TEST_SAMPLE_COUNT];
static volatile motor_abc_f_t current_noise_amp[CURRENT_NOISE_TEST_SAMPLE_COUNT];
static volatile uint32_t current_noise_count = 0U;

static volatile uint16_t current_trip_raw_a = 0U;
static volatile uint16_t current_trip_raw_b = 0U;
static volatile uint16_t current_trip_raw_c = 0U;

static volatile float current_trip_ia = 0.0f;
static volatile float current_trip_ib = 0.0f;
static volatile float current_trip_ic = 0.0f;

#endif


#if !((MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
      (CURRENT_NEUTRAL_DIAG_ENABLE == 1U))
static motor_duty_t motor_duty;
#endif


static volatile motor_monitor_t motor_monitor;

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)



bool motorTestDiagIsDone(void)
{
  return current_diag_done;
}

uint32_t motorTestDiagGetSampleCount(void)
{
  return current_diag_sample_count;
}

void motorTestDiagGetCurrentMinMax(motor_abc_f_t *i_min, motor_abc_f_t *i_max)
{
  if (i_min != NULL)
  {
    i_min->a = current_diag_i_min.a;
    i_min->b = current_diag_i_min.b;
    i_min->c = current_diag_i_min.c;
  }

  if (i_max != NULL)
  {
    i_max->a = current_diag_i_max.a;
    i_max->b = current_diag_i_max.b;
    i_max->c = current_diag_i_max.c;
  }
}

uint32_t motorLTestGetCount(void)
{
  return current_l_test_count;
}

bool motorLTestGetSample(uint32_t index, float *id, float *iq, float *vd, float *vq, float *theta_e)
{
  if (index >= current_l_test_count)
  {
    return false;
  }

  if (id != NULL)
  {
    *id = current_l_test_id[index];
  }

  if (iq != NULL)
  {
    *iq = current_l_test_iq[index];
  }

  if (vd != NULL)
  {
    *vd = current_l_test_vd[index];
  }

  if (vq != NULL)
  {
    *vq = current_l_test_vq[index];
  }

  if (theta_e != NULL)
  {
    *theta_e = current_l_test_theta[index];
  }

  return true;
}



#endif
