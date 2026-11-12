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

/* Diagnostic types and function declarations.
 * Configuration values are defined in common/def.h. */

typedef struct
{
  uint32_t tick;
  int32_t encoder_count;
  float theta_e;
  float id_ref, id_meas, iq_ref, iq_meas;
  float vd, vq;
  float ia, ib, ic;
  float duty_u, duty_v, duty_w;
  float pos_ref_counts, pos_meas_counts, pos_error_counts;
  float pos_raw_counts, pos_vel_counts_per_s;
  float pos_p_iq_a, pos_d_iq_a;
  uint32_t ccr1, ccr2, ccr3, ccer;
  uint8_t moe;
} motor_align_diag_sample_t;

typedef struct
{
  uint32_t valid_mask;
  int32_t start_count;
  int32_t align_count;
  int32_t position_zero_count;
  int32_t id_zero_count;
  int32_t plus_end_count;
  int32_t home1_end_count;
  int32_t minus_end_count;
  int32_t home2_end_count;
  int32_t end_count;
  int32_t settle_peak_counts;
  int32_t accumulated_travel_counts; /* 1 kHz diag, NOT a trip */
  int32_t raw_accumulated_travel_counts; /* diagnostic ONLY: 20 kHz, every +/-1 count */
  int32_t peak_abs_error_counts;
  int32_t peak_abs_raw_error_counts;
  int32_t zero_iq_variation_1khz;
  int32_t zero_iq_net_count;
  int32_t zero_iq_peak_1ms_step;
  int32_t drive_variation_1khz;
  int32_t drive_peak_1ms_step;
  int32_t peak_raw_filter_gap_millicounts;
  uint8_t completed_stages; /* bit0=Id0, bit1=+target, bit2=home1,
                                bit3=-target, bit4=home2 */
  float encoder_offset_rad;
  motor_align_diag_sample_t samples[MOTOR_DIAG_SAMPLE_COUNT];
} motor_align_diag_t;

bool motorGetAlignDiag(motor_align_diag_t *result);
uint8_t motorGetAlignGateStatus(void);

/* F/R and T/Y: bounded signed-current direction diagnostics. */
typedef struct
{
  uint32_t tick;                /* ticks since end of alignment, at 20kHz */
  int32_t encoder_count;       /* raw TIM2 modulo 4000 */
  int32_t position_counts;     /* signed from relaxed zero, wrapped */
  float theta_e;
  float id_ref, id_meas, iq_ref, iq_meas, vq;
} motor_torque_probe_sample_t;

typedef struct
{
  int8_t direction;           /* +1 or -1 Iq */
  uint8_t result;             /* 0 pending, 1 complete, 2 align unstable, 3 excursion cut */
  uint16_t sample_mask;
  int32_t align_count;
  int32_t zero_count;
  int32_t end_count;
  int32_t peak_abs_counts;
  uint8_t high_current;         /* 1=T/Y at 3.5 A, 0=F/R at 0.30 A */
  int32_t move_limit_counts;
  uint32_t cut_tick;            /* post-alignment ticks if result=3 */
  int32_t cut_count;            /* raw TIM2 count at excursion cutoff */
  int32_t cut_rel_count;        /* signed count from alignment origin */
  motor_torque_probe_sample_t samples[MOTOR_TORQUE_PROBE_SAMPLE_COUNT];
} motor_torque_probe_diag_t;

bool motorSelectTorqueProbe(int8_t direction);
bool motorSelectHighIqProbe(int8_t direction); /* T (+3.5A) / Y (-3.5A) */
int8_t motorGetTorqueProbeDirection(void);
bool motorGetTorqueProbeDiag(motor_torque_probe_diag_t *result);

/* G/H: one-revolution current-only spin probe. */

typedef struct
{
  uint32_t tick; /* 20 kHz ticks since Iq command starts */
  int32_t raw_count; /* TIM2 counter modulo 4000 */
  int32_t unwrapped_count; /* signed continuous travel from Iq start */
  float rpm_meas; /* 10 ms encoder count derivative, mechanical RPM */
  float theta_e, id_ref, id_meas, iq_ref, iq_meas, vd, vq;
} motor_spin_probe_sample_t;

typedef struct
{
  uint8_t selected;
  int8_t direction;
  uint8_t result; /* 1=time complete,2=align fail,3=1rev,4=overspeed,5=wrong dir,6=stall,7=pre-start movement */
  uint8_t sample_count;
  int32_t align_count, zero_count, end_count;
  int32_t total_counts, peak_abs_counts, maximum_forward_counts;
  float peak_abs_rpm, last_rpm;
  uint32_t stop_tick; /* post-alignment ticks */
  int32_t stop_raw_count, stop_unwrapped_count;
  motor_spin_probe_sample_t samples[MOTOR_SPIN_PROBE_SAMPLE_COUNT];
} motor_spin_probe_diag_t;

bool motorSelectSpinProbe(int8_t direction); /* G: +Iq, H: -Iq */
bool motorIsSpinProbeSelected(void);
bool motorGetSpinProbeDiag(motor_spin_probe_diag_t *result);

/* K/L: startup, handoff, speed PI and bounded restart diagnostics.
 * Gains, timing, current limits and alignment criteria: common/def.h.
 */
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
  int32_t align_base_count, align_plus_count, align_return_count;
  int32_t align_plus_delta, align_return_delta, align_peak_delta;
  uint8_t align_vector_pass;
  uint8_t align_vector_skipped; /* 1=optional 3.5A sweep not run */
  /* Alignment fail mask (unchanged proven K/L gate):
     0x01 MAX_EXCURSION, 0x02 MIN_RESPONSE, 0x04 RETURN_RESIDUAL,
     0x08 FINAL_SETTLE, 0x10 STATIC_SETTLE. */
  uint8_t align_fail_mask;
  /* 0 STATIC, 1 RAMP, 2 UP, 3 HOLD_90, 4 BACK, 5 SETTLE. */
  uint8_t align_fail_stage;
  uint8_t align_plus_valid;
  uint8_t align_return_valid; /* 0 on an early cutoff: return_count is stop-count */
  uint32_t align_fail_v_tick; /* ticks since end of original 2s static align */
  int32_t align_settle_base_count;
  int32_t align_settle_final_delta;
  int32_t align_settle_peak_counts;
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

/* Encoder electrical-angle OFFSET diagnostic (P/N, 1..5). */
typedef struct
{
  uint32_t tick;              /* 20 kHz ticks since alignment end */
  int32_t encoder_count;
  int32_t relative_count;     /* vs. zero-current settled start */
  float theta_e;              /* encoder theta + diagnostic delta */
  float id_ref, id_meas, iq_ref, iq_meas, vd, vq;
  float ia, ib, ic;
} motor_offset_probe_sample_t;

typedef struct
{
  uint8_t selected;
  int8_t direction;           /* +1 or -1 Iq */
  int16_t offset_deg;         /* diagnostic delta; NOT base offset */
  uint8_t result;             /* 1 complete, 2 align unstable, 3 excursion */
  uint16_t sample_mask;
  float base_offset_rad;      /* set by alignment, for reference only */
  int32_t align_count;
  int32_t zero_count;
  int32_t end_count;
  int32_t peak_abs_counts;
  motor_offset_probe_sample_t samples[MOTOR_OFFSET_PROBE_SAMPLE_COUNT];
} motor_offset_probe_diag_t;

bool motorSelectOffsetProbe(int8_t direction, uint8_t offset_index);
bool motorIsOffsetProbeSelected(void);
bool motorGetOffsetProbeDiag(motor_offset_probe_diag_t *result);

/* V: stator-current-vector hysteresis diagnostic. */
typedef struct
{
  uint32_t tick;                  /* ticks after alignment (20kHz) */
  int32_t encoder_count;          /* raw modulo-4000 TIM2 count */
  int32_t relative_count;         /* wrapped count delta from align end */
  float theta_cmd;                /* Park and inverse-Park command, rad */
  float id_ref, id_meas, iq_ref, iq_meas, vd, vq;
  float ia, ib, ic;             /* same ADC sample as current PI */
  float duty_u, duty_v, duty_w; /* same PI update, before next PWM period */
} motor_vector_probe_sample_t;

typedef struct
{
  uint8_t selected;
  uint8_t result;                 /* 1 complete, 2 align unstable, 3 excursion */
  uint16_t sample_mask;
  int32_t align_count;
  int32_t end_count;
  int32_t peak_abs_counts;
  uint32_t cut_tick;              /* valid when result=3, 20 kHz */
  float cut_theta_cmd;            /* planned stator angle at excursion cut */
  int32_t cut_encoder_count;      /* raw TIM2 count at excursion cut */
  motor_vector_probe_sample_t samples[MOTOR_VECTOR_PROBE_SAMPLE_COUNT];
} motor_vector_probe_diag_t;

bool motorSelectVectorProbe(void);
bool motorIsVectorProbeSelected(void);
bool motorGetVectorProbeDiag(motor_vector_probe_diag_t *result);

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
