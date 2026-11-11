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

/* Snapshots for static alignment + signed relative position controller.
 * Captured in the ADC ISR and only printed from the UART main loop. */
#define MOTOR_DIAG_SAMPLE_COUNT 12U

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

/* Original F/R test remains 0.30 A / 70-count limit.
 * New T/Y: encoder-angle closed-loop FOC, Iq=+/-3.50 A, 250-count
 * excursion cutoff. The D-axis command is zero after alignment release.
 * PWM and measured current protections are preserved from def.h.
 * The constants below are local to this new diagnostic, not motor ratings.
 */
#define MOTOR_HIGH_IQ_PROBE_A               3.50f
#define MOTOR_HIGH_IQ_PROBE_MAX_MOVE_COUNTS 250
/* Fixed-Iq direction test; use only when the motor state is IDLE.
 * direction: +1 (+Iq), -1 (-Iq), or 0 (normal position-PD S mode). */
#define MOTOR_TORQUE_PROBE_SAMPLE_COUNT 8U
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

/* G/H - one mechanical revolution FOC current-only spin probe.
 * Uses the already tested encoder angle/current PI. No speed PI, PD, or
 * encoderUpdateSpeed() dependency. Limit is UNWRAPPED signed displacement.
 * This is a bounded commissioning test, NOT a motor current rating.
 */
#define MOTOR_SPIN_PROBE_IQ_A               3.50f
#define MOTOR_SPIN_PROBE_RAMP_TICKS         2000U   /* 100 ms */
#define MOTOR_SPIN_PROBE_HOLD_TICKS        18000U   /* 900 ms */
#define MOTOR_SPIN_PROBE_DECAY_TICKS        2000U   /* 100 ms */
#define MOTOR_SPIN_PROBE_START_TICK         TORQUE_PROBE_START_TICK
#define MOTOR_SPIN_PROBE_TOTAL_TICKS        (MOTOR_SPIN_PROBE_START_TICK + MOTOR_SPIN_PROBE_RAMP_TICKS + MOTOR_SPIN_PROBE_HOLD_TICKS + MOTOR_SPIN_PROBE_DECAY_TICKS)
#define MOTOR_SPIN_PROBE_MAX_TRAVEL_COUNTS  ((int32_t)ENCODER_MAX_COUNT) /* 1 motor rev */
#define MOTOR_SPIN_PROBE_MAX_RPM            1500.0f /* independent 10 ms estimate */
#define MOTOR_SPIN_PROBE_WRONG_COUNTS       200     /* unexpected direction */
#define MOTOR_SPIN_PROBE_STALL_TICKS        4000U   /* 200 ms from Iq start */
#define MOTOR_SPIN_PROBE_STALL_COUNTS       20
#define MOTOR_SPIN_PROBE_SAMPLE_COUNT      16U

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

/* K/L: bounded encoder FOC + speed PI trial, independent of MOTOR_CONTROL_MODE.
 * +Iq moves raw encoder count negative: signed +RPM means +Iq direction.
 * Speed measurement/PI update at 100Hz, current FOC at 20kHz.
 * New speed mode alone gets expanded current-PI voltage headroom;
 * alignment and older T/Y/G/H/V probes retain the 1.8V diagnostic cap.
 * These gains/limits are COMMISSIONING defaults, not a motor rating.
 */
/* K/L speed commissioning settings. These affect ONLY K/L, not old probes.
 * Current is limited by min(IQ_MAX, CURRENT_TEST_OC_LIMIT_A-OC_MARGIN).
 * 3.0A is an initial STARTUP target based on prior successful T/Y/G/H logs,
 * not a motor current rating. Do not remove board/firmware protection. */
#define MOTOR_SPEED_PI_GOAL_RPM            180.0f
#define MOTOR_SPEED_PI_RAMP_RPM_PER_S      400.0f
#define MOTOR_SPEED_PI_KP_A_PER_RPM        0.006f
#define MOTOR_SPEED_PI_KI_A_PER_RPM_S      0.015f
#define MOTOR_SPEED_PI_IQ_MAX_A            3.20f
#define MOTOR_SPEED_PI_OC_MARGIN_A         0.25f
#define MOTOR_SPEED_PI_V_LIMIT_V           4.50f
#define MOTOR_SPEED_PI_TICKS_PER_STEP      200U    /* 10 ms at 20 kHz */
#define MOTOR_SPEED_PI_RUN_TICKS           30000U  /* 1.5 s after Id zero */
#define MOTOR_SPEED_PI_START_TICK          TORQUE_PROBE_START_TICK
#define MOTOR_SPEED_PI_MAX_RPM             500.0f
#define MOTOR_SPEED_PI_MAX_TRAVEL_COUNTS   24000   /* 6 mechanical revolutions */
#define MOTOR_SPEED_PI_REVERSE_COUNTS      400
#define MOTOR_SPEED_PI_STALL_TICKS         6000U   /* 300 ms */
#define MOTOR_SPEED_PI_STALL_COUNTS        20
#define MOTOR_SPEED_PI_FILTER_ALPHA        0.60f
#define MOTOR_SPEED_PI_SAMPLE_COUNT        30U

/* Three-stage startup -> bumpless handoff -> low-gain speed PI. */
#define MOTOR_SPEED_STARTUP_IQ_A           3.00f
#define MOTOR_SPEED_STARTUP_RAMP_TICKS     1600U   /* 80 ms */
#define MOTOR_SPEED_STARTUP_TIMEOUT_TICKS  3400U   /* 170 ms */
#define MOTOR_SPEED_START_DETECT_RPM       20.0f
#define MOTOR_SPEED_START_DETECT_COUNTS    25
#define MOTOR_SPEED_START_CONFIRM_STEPS    2U      /* 2 successive 10ms readings */
#define MOTOR_SPEED_HANDOFF_TICKS          2400U   /* 120 ms */
#define MOTOR_SPEED_HANDOFF_MIN_RPM        40.0f
#define MOTOR_SPEED_IQ_SLEW_A_PER_S        40.0f
#define MOTOR_SPEED_INTEGRAL_TRACK_GAIN    0.15f
#define MOTOR_SPEED_INTEGRAL_LIMIT_A       0.80f
#define MOTOR_SPEED_RUN_STALL_STEPS        25U     /* 250ms below 5RPM */

/* K/L ONLY: active 0 -> +90 -> 0 stator-vector alignment validation.
   This detects a non-reproducible/immobile rotor; it does not prove an
   absolute flux-axis offset. 20kHz timer ticks. */
#define MOTOR_SPEED_ALIGN_ID_A           3.50f
#define MOTOR_SPEED_ALIGN_RAMP_TICKS     2000U  /* 100ms */
#define MOTOR_SPEED_ALIGN_UP_TICKS       6000U  /* 300ms */
#define MOTOR_SPEED_ALIGN_HOLD_TICKS     4000U  /* 200ms */
#define MOTOR_SPEED_ALIGN_BACK_TICKS     6000U  /* 300ms */
#define MOTOR_SPEED_ALIGN_SETTLE_TICKS   6000U  /* 300ms */
#define MOTOR_SPEED_ALIGN_TEST_TICKS (MOTOR_SPEED_ALIGN_RAMP_TICKS + \
  MOTOR_SPEED_ALIGN_UP_TICKS + MOTOR_SPEED_ALIGN_HOLD_TICKS + \
  MOTOR_SPEED_ALIGN_BACK_TICKS + MOTOR_SPEED_ALIGN_SETTLE_TICKS)
#define MOTOR_SPEED_ALIGN_MIN_RESPONSE_COUNTS  35
#define MOTOR_SPEED_ALIGN_MAX_EXCURSION_COUNTS 350
#define MOTOR_SPEED_ALIGN_MAX_RESIDUAL_COUNTS  35
#define MOTOR_SPEED_ALIGN_LAST_SETTLE_COUNTS   8
/* Accelerating is limited; removing accelerating current when target speed
   is reached is allowed to be faster. Does not force reverse torque. */
#define MOTOR_SPEED_IQ_DECEL_SLEW_A_PER_S  180.0f
#define MOTOR_SPEED_STARTUP_SOFT_RPM       90.0f
#define MOTOR_SPEED_STARTUP_STOP_RPM       130.0f
#define MOTOR_SPEED_HANDOFF_EXIT_IQ_A       0.35f


typedef enum
{
  MOTOR_SPEED_PHASE_NONE = 0U,
  MOTOR_SPEED_PHASE_STARTUP = 1U,
  MOTOR_SPEED_PHASE_HANDOFF = 2U,
  MOTOR_SPEED_PHASE_RUN = 3U
} motor_speed_phase_t;

typedef struct
{
  uint32_t tick;
  uint8_t phase; /* STARTUP=1, HANDOFF=2, RUN=3 */
  int32_t raw_count, unwrapped_count;
  float rpm_raw, rpm_filtered, rpm_ref, iq_ref, iq_meas;
  float id_meas, vd, vq, vmag;
} motor_speed_pi_sample_t;

typedef struct
{
  uint8_t selected;
  int8_t direction;
  uint8_t result; /* 1=time,2=align,3=overspeed,4=reverse,5=stall,6=travel,7=prestart,8=startup timeout,9=align vector check */
  uint8_t sample_count;
  uint8_t last_phase;
  int32_t align_count, zero_count, stop_raw, total_counts;
  uint32_t startup_end_tick, handoff_end_tick;
  int32_t align_base_count, align_plus_count, align_return_count;
  int32_t align_plus_delta, align_return_delta, align_peak_delta;
  uint8_t align_vector_pass;
  float rpm_at_startup_end, iq_at_startup_end;
  int32_t maximum_forward_counts;
  uint32_t stop_tick, voltage_saturation_ticks;
  float peak_abs_rpm, stop_rpm, max_abs_vmag, iq_limit_used;
  motor_speed_pi_sample_t samples[MOTOR_SPEED_PI_SAMPLE_COUNT];
} motor_speed_pi_diag_t;

bool motorSelectSpeedPiProbe(int8_t direction);  /* K=+180RPM, L=-180RPM */
bool motorIsSpeedPiProbeSelected(void);
bool motorGetSpeedPiProbeDiag(motor_speed_pi_diag_t *out);

/* Encoder electrical-angle OFFSET diagnostic, independent run per keypress.
 * UART: select 1..5 (-60/-30/0/+30/+60 deg), P=+Iq, N=-Iq.
 * The extra offset is applied only after Id has reached zero, then held
 * through the 150 ms zero-current settling period and bounded Iq pulse.
 * Position controller is bypassed; base alignment offset is not rewritten.
 */
#define MOTOR_OFFSET_PROBE_SAMPLE_COUNT 10U
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

/* V command: +90 degree stator-current-vector HYSTERESIS diagnostic.
 * Hold 3.5A at +90deg, reduce to 2.5A at the SAME commanded angle,
 * hold again, then return to zero. Encoder is telemetry/safety ONLY.
 * Samples are copied only after PWM is disabled.
 */
#define MOTOR_VECTOR_PROBE_SAMPLE_COUNT 16U
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
