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

/* 0/45/90 electrical degree current-vector diagnostic. The CCR values are
 * TIMER REGISTERS, not a physical oscilloscope measurement of the pins.
 * No offset calibration is performed by this diagnostic build. */
typedef struct
{
  uint32_t tick;
  int32_t encoder_count;
  float theta_e;
  float id_ref, id_meas, iq_meas;
  float vd, vq;
  float ia, ib, ic;
  float duty_u, duty_v, duty_w;
  uint32_t ccr1, ccr2, ccr3, ccer;
  uint8_t moe;
} motor_align_diag_sample_t;

typedef struct
{
  uint32_t valid_mask; /* bits 0,1,2 = 0,45,90 deg */
  int32_t start_count;
  int32_t end_count;
  motor_align_diag_sample_t samples[3];
} motor_align_diag_t;

bool motorGetAlignDiag(motor_align_diag_t *result);

extern volatile uint8_t debug_bkin_stage;
extern volatile uint32_t debug_bkin_sr;
extern volatile uint32_t debug_bkin_bdtr;
extern volatile uint32_t debug_bkin_pa6;
extern volatile uint32_t bdtr_before;
extern volatile uint32_t sr_before;


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
