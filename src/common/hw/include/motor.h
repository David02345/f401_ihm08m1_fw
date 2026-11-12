#ifndef SRC_COMMON_HW_INCLUDE_MOTOR_H_
#define SRC_COMMON_HW_INCLUDE_MOTOR_H_

#include "hw_def.h"
#include "motor_types.h"
#include "pid.h"
#include "pwm.h"
#include "adc.h"
#include "foc.h"
#include "encoder.h"

/* K/L encoder FOC speed profile: sampled logs stored until PWM is OFF. */
typedef enum
{
  MOTOR_SPEED_PHASE_NONE = 0U,
  MOTOR_SPEED_PHASE_STARTUP = 1U,
  MOTOR_SPEED_PHASE_HANDOFF = 2U,
  MOTOR_SPEED_PHASE_RUN = 3U,
  MOTOR_SPEED_PHASE_RESTART = 4U
} motor_speed_phase_t;

typedef struct
{
  uint32_t tick;
  uint8_t phase; /* STARTUP=1, HANDOFF=2, RUN=3, RESTART=4 */
  uint8_t profile_segment; /* 0=pre-run, 1=accel, 2=hold, 3=decel, 4=stop */
  int32_t raw_count, unwrapped_count;
  float rpm_raw, rpm_filtered, rpm_ref, iq_ref, iq_meas;
  float id_meas, vd, vq, vmag;
  float theta_e; /* electrical angle actually used by current loop (rad) */
} motor_speed_pi_sample_t;

typedef struct
{
  uint8_t selected;
  int8_t direction;
  uint8_t result; /* 1=profile stopped,2=align,3=overspeed,4=reverse,5=stall,6=travel,7=prestart,8=startup timeout,9=align check,10=restart timeout,11=stop timeout */
  uint8_t sample_count;
  uint8_t last_phase;
  int32_t align_count, zero_count, stop_raw, total_counts;
  uint32_t startup_end_tick, handoff_end_tick;
  uint32_t profile_start_tick;
  float profile_initial_rpm;
  uint8_t restart_count;
  uint32_t restart_start_tick, restart_recovered_tick;
  /* Diagnostics only: no control or safety behavior changes. */
  int32_t diag_align_begin_count, diag_align_ramp_count;
  int32_t diag_align_final_count, diag_start_zero_count;
  float diag_align_raw_theta, diag_offset_theta;
  float diag_calibrated_theta, diag_start_theta;
  int32_t align_base_count;
  int32_t align_static_settle_peak_counts;
  float rpm_at_startup_end, iq_at_startup_end;
  int32_t maximum_forward_counts;
  uint32_t stop_tick, voltage_saturation_ticks;
  float peak_abs_rpm, stop_rpm, max_abs_vmag, iq_limit_used;
  motor_speed_pi_sample_t samples[MOTOR_SPEED_PI_SAMPLE_COUNT];
} motor_speed_pi_diag_t;

bool motorSelectSpeedPiProbe(int8_t direction);  /* K=+180RPM, L=-180RPM */
bool motorIsSpeedPiProbeSelected(void);
bool motorGetSpeedPiProbeDiag(motor_speed_pi_diag_t *out);


bool motorInit(void);
void motorStart(void);
void motorStop(void);
void motorAlign(void);
void motorLowSpeedTask(void);
void motorCurrentLoop(float id_ref, float iq_ref, float theta_e);
void motorControlUpdate(void);
void motorSetFault(motor_fault_t fault);
void motorClearFault(void);
bool motorGetMonitor(motor_monitor_t *monitor);
motor_state_t motorGetState(void);
motor_fault_t motorGetFault(void);

#endif /* SRC_COMMON_HW_INCLUDE_MOTOR_H_ */
