/*
 * motor.c
 *
 *  Created on: 2026. 7. 4.
 *      Author: YDG
 */

#include "motor.h"
#include "motor_types.h"
#include "phase_map_diag.h"
#include "fixed_id_diag.h"
#include "uart.h"
#include "util.h"


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
static volatile motor_abc_f_t current_snapshot_50ms;
static volatile motor_abc_f_t current_avg_sum;
static volatile uint32_t current_avg_count = 0U;

static volatile float current_diag_id_sum = 0.0f;
static volatile float current_diag_iq_sum = 0.0f;
#define CURRENT_STARTUP_SETTLE_SAMPLES  4U

static volatile uint32_t current_startup_count = 0U;
// motor.c 진단 변수 영역
static volatile uint32_t current_trip_startup_sample = 0U;
// PWM Enable 및 첫 ADC ISR 타이밍 진단
static volatile uint32_t diag_cnt_before_output = 0U;
static volatile uint32_t diag_cnt_first_isr = 0U;

static volatile uint32_t diag_jeoc_before_output = 0U;
static volatile uint32_t diag_irq_pending_before_output = 0U;

static volatile uint32_t diag_output_cycle = 0U;
static volatile uint32_t diag_first_isr_cycles = 0U;

static volatile bool diag_first_isr_seen = false;

static volatile uint16_t diag_raw_before_a = 0U;
static volatile uint16_t diag_raw_before_b = 0U;
static volatile uint16_t diag_raw_before_c = 0U;

static volatile bool diag_pwm_armed = false;
static volatile uint32_t diag_moe_after_off = 2U;
// 여기까지

static volatile uint32_t current_diag_tail_count = 0U;

static volatile float current_diag_tail_id_sum = 0.0f;
static volatile float current_diag_tail_iq_sum = 0.0f;

static volatile float current_diag_tail_vd_sum = 0.0f;
static volatile float current_diag_tail_vq_sum = 0.0f;

// [L TEST] 첫 3 ms current response 저장
static volatile float current_l_test_id[CURRENT_L_TEST_SAMPLE_COUNT];
static volatile float current_l_test_iq[CURRENT_L_TEST_SAMPLE_COUNT];
static volatile float current_l_test_vq[CURRENT_L_TEST_SAMPLE_COUNT];
static volatile float current_l_test_vd[CURRENT_L_TEST_SAMPLE_COUNT];
static volatile float current_l_test_theta[CURRENT_L_TEST_SAMPLE_COUNT];

static volatile uint8_t current_l_test_hall_state[CURRENT_L_TEST_SAMPLE_COUNT];
static volatile int8_t current_l_test_hall_sector[CURRENT_L_TEST_SAMPLE_COUNT];
static volatile uint8_t current_l_test_hall_valid[CURRENT_L_TEST_SAMPLE_COUNT];



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


static motor_duty_t motor_duty;
static pid_ctrl_t pi_id;
static pid_ctrl_t pi_iq;
static pid_ctrl_t pi_spd;

static volatile float motor_vbus = 0.0f;
static volatile uint16_t motor_speed_cmd_raw = 0;
static uint16_t motor_temp_raw = 0;

static float speed_w_ref  = 0.0f;
static float speed_w_meas = 0.0f;
static float speed_w_target = 0.0f;

static uint16_t speed_loop_divider = 0;

static float current_id_ref = 0.0f;
static float current_iq_ref = 0.0f;

static volatile motor_monitor_t motor_monitor;

#if (CURRENT_ROTATION_TEST_ENABLE == 1U) && (CURRENT_ROTATION_ANGLE_SWEEP_ENABLE == 1U)

static float current_rotation_test_theta_e = 0.0f;

#endif

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && (CURRENT_ADC_RAW_SCAN_ENABLE == 1U)

static volatile motor_adc_raw_scan_t adc_raw_scan;
  #if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U

  #define ADC_SCAN_STARTUP_SAMPLES 4U

  static volatile uint16_t split_raw_min[2][3];
  static volatile uint16_t split_raw_max[2][3];
  static volatile uint32_t split_spike_count[2][3];
  static volatile uint32_t split_sample_count[2];

  #endif

  #if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U

  static volatile bool adc_pwm_on_scan_done = false;
  static volatile bool adc_pwm_on_scan_seen = false;

  #endif

#endif

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && (CURRENT_FOC_BENCH_ENABLE == 1U)

static volatile uint32_t foc_bench_count = 0U;
static volatile uint32_t foc_bench_total_cycles = 0U;
static volatile uint32_t foc_bench_max_cycles = 0U;

#if CURRENT_FOC_BENCH_DIAG_LOAD == 1U

static volatile uint32_t foc_bench_diag_index = 0U;

static volatile float foc_bench_ia_min = 0.0f;
static volatile float foc_bench_ia_max = 0.0f;
static volatile float foc_bench_ib_min = 0.0f;
static volatile float foc_bench_ib_max = 0.0f;
static volatile float foc_bench_ic_min = 0.0f;
static volatile float foc_bench_ic_max = 0.0f;

#endif

#endif

static void motorCurrentLoop(float id_ref, float iq_ref, float theta_e);

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP

static float open_loop_theta_e = 0.0f;
static float open_loop_speed_e = 0.0f;
static float open_loop_vd = 0.0f;
static float open_loop_vq = 0.0f;

static uint32_t open_loop_align_count = 0U;
static bool open_loop_alignment_done = false;




#if OPEN_LOOP_DIAG_ENABLE == 1U
static volatile motor_ol_diag_t open_loop_diag;
static uint8_t open_loop_diag_prev_raw;
static uint8_t open_loop_diag_prev_cached;
static uint8_t open_loop_diag_mismatch_samples;
static int8_t open_loop_diag_last_confirmed_sector;
static int8_t open_loop_diag_confirmed_direction;
static volatile int8_t open_loop_diag_requested_direction = 1;
static volatile uint8_t open_loop_diag_requested_fixed_only = 1U;
static volatile uint8_t open_loop_diag_requested_absolute_start = 0U;

/* T test only: low-current, absolute stator field, independent of Hall angle.
 * Software protection is the existing OPEN_LOOP_DIAG_OC_LIMIT_A (1.5 A).
 * Do not change the H/J current limits or Hall calibration constants. */
#define ABS_SWEEP_I_A                    1.70f
#define ABS_SWEEP_ALIGN_I_A              1.70f

static uint8_t motorOpenLoopDiagReadPins(void);

/* ---------- Isolated Hall six-step diagnostic (UART 'C') ----------
 * Uses the proven F103 COMM_FWD table after reversing the Hall bit order.
 * PWM source uses TIM1 CHx only; sink uses TIM1 CHxN only (hardware BKIN).
 * The floating leg is GPIO-forced to HIN=0, LIN=1: L6398 both gates OFF.
 * Never GPIO-force a gate ON; such a gate could bypass TIM1 Break.
 */
#define SIX_STEP_TEST_MS           250U
#define SIX_STEP_MAX_SAMPLES      (SIX_STEP_TEST_MS * 20U)
#define SIX_STEP_OC_LIMIT_A       2.00f
#define SIX_STEP_START_DUTY       0.025f
#define SIX_STEP_FINAL_DUTY       0.050f
#define SIX_STEP_RAMP_SAMPLES     2000U   /* 100 ms */
#define SIX_STEP_BLANK_SAMPLES    3U      /* 150 us; CCR preload must update */
#define SIX_STEP_TARGET_EDGES     3U

typedef struct
{
  volatile uint32_t samples;
  volatile uint32_t edges;
  volatile uint32_t stop_sample;
  volatile float peak_a, peak_b, peak_c;
  volatile uint8_t selected, complete, initial_hall, last_hall;
  volatile uint8_t overcurrent, bkin, invalid_hall, lost_moe;
  volatile uint8_t reached_edge_target;
  volatile uint8_t wait_preload;
  volatile int8_t last_direction;
} six_step_diag_t;

static six_step_diag_t six_step;

/* Timer pin pairs are U: PA8/PA7, V: PA9/PB0, W: PA10/PB1.
 * L6398 truth table: LIN=1 + HIN=0 => LVG=0, HVG=0.
 * Keep GPIO output latches OFF before changing AF pin modes. */
static void sixStepGatePinsOff(void)
{
  const uint32_t a_mask =
      (3UL << (7U * 2U)) | (3UL << (8U * 2U)) |
      (3UL << (9U * 2U)) | (3UL << (10U * 2U));
  const uint32_t b_mask = (3UL << 0U) | (3UL << 2U);
  const uint32_t a_output =
      (1UL << (7U * 2U)) | (1UL << (8U * 2U)) |
      (1UL << (9U * 2U)) | (1UL << (10U * 2U));
  const uint32_t b_output = (1UL << 0U) | (1UL << 2U);
  GPIOA->BSRR = GPIO_PIN_7 |
       (((uint32_t)(GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10)) << 16);
  GPIOB->BSRR = GPIO_PIN_0 | GPIO_PIN_1;
  GPIOA->MODER = (GPIOA->MODER & ~a_mask) | a_output;
  GPIOB->MODER = (GPIOB->MODER & ~b_mask) | b_output;
}

/* Unlike FOC complementary drive, six-step must enable ONLY the
 * high-side input of the PWM phase.  Leave that phase's LIN GPIO HIGH
 * so the low-side transistor remains OFF during PWM off-time.
 * Both ON commands are still TIM1 outputs, thus hardware BKIN applies. */
static void sixStepHighTimerAF(uint8_t p)
{
  const uint32_t pin = 8U + (uint32_t)p; /* PA8/PA9/PA10 */
  const uint32_t shift = 2U * pin;
  GPIOA->MODER = (GPIOA->MODER & ~(3UL << shift)) |
                 (2UL << shift);
}

/* Enable only the low-side input of the sink phase. Its HIN stays
 * GPIO LOW. With the existing TIM1 CCxNP=1 and CCR=0, LIN is LOW
 * while MOE is active (validated on the prior gate-input test). */
static void sixStepLowTimerAF(uint8_t p)
{
  if (p == 0U) /* U: LIN=PA7 */
    GPIOA->MODER = (GPIOA->MODER & ~(3UL << 14U)) | (2UL << 14U);
  else if (p == 1U) /* V: LIN=PB0 */
    GPIOB->MODER = (GPIOB->MODER & ~(3UL << 0U)) | (2UL << 0U);
  else /* W: LIN=PB1 */
    GPIOB->MODER = (GPIOB->MODER & ~(3UL << 2U)) | (2UL << 2U);
}

static bool sixStepPhasePair(uint8_t hall, uint8_t *positive, uint8_t *negative)
{
  /* F103 PC0=W, PC1=V, PC2=U, while current bit0=U bit1=V bit2=W.
   * Converted F103 COMM_FWD: 001 U+/W-, 101 U+/V-, 100 W+/V-,
   * 110 W+/U-, 010 V+/U-, 011 V+/W-. */
  switch (hall)
  {
    case 1U: *positive = 0U; *negative = 2U; return true;
    case 5U: *positive = 0U; *negative = 1U; return true;
    case 4U: *positive = 2U; *negative = 1U; return true;
    case 6U: *positive = 2U; *negative = 0U; return true;
    case 2U: *positive = 1U; *negative = 0U; return true;
    case 3U: *positive = 1U; *negative = 2U; return true;
    default: return false;
  }
}

static int8_t sixStepHallOrder(uint8_t hall)
{
  switch (hall)
  {
    case 1U: return 0;
    case 5U: return 1;
    case 4U: return 2;
    case 6U: return 3;
    case 2U: return 4;
    case 3U: return 5;
    default: return -1;
  }
}

static void sixStepFinish(motor_fault_t reason)
{
  pwmDisableOutput();
  sixStepGatePinsOff();
  six_step.complete = 1U;
  six_step.stop_sample = six_step.samples;
  if (reason != MOTOR_FAULT_NONE)
  {
    motor_fault = reason;
    motor_state = MOTOR_STATE_FAULT;
  }
}

/* Every commutation: blank ALL phases, update CCR preload, wait 3 ISR periods.
 * No active MOSFET is GPIO driven; only TIM1 drives the ON legs. */
static bool sixStepQueue(uint8_t hall, float high_duty)
{
  uint8_t plus, minus;
  if (!sixStepPhasePair(hall, &plus, &minus)) return false;
  pwmDisableOutput();
  sixStepGatePinsOff();
  float duty[3] = {0.0f, 0.0f, 0.0f};
  duty[plus] = high_duty;
  pwmSetDuty(duty[0], duty[1], duty[2]);
  six_step.wait_preload = SIX_STEP_BLANK_SAMPLES;
  six_step.last_hall = hall;
  return true;
}

static void sixStepUpdate(void)
{
  if (motor_state != MOTOR_STATE_OPEN_LOOP || six_step.complete) return;
  motor_abc_f_t current;
  adcGetPhaseCurrent(&current);
  motor_monitor.ia = current.a;
  motor_monitor.ib = current.b;
  motor_monitor.ic = current.c;
  float a = fabsf(current.a), b = fabsf(current.b), c = fabsf(current.c);
  if (a > six_step.peak_a) six_step.peak_a = a;
  if (b > six_step.peak_b) six_step.peak_b = b;
  if (c > six_step.peak_c) six_step.peak_c = c;

  if (pwmIsBreakFault() || (TIM1->SR & TIM_SR_BIF))
  {
    six_step.bkin = 1U;
    sixStepFinish(MOTOR_FAULT_BKIN);
    return;
  }
  if (a > SIX_STEP_OC_LIMIT_A || b > SIX_STEP_OC_LIMIT_A || c > SIX_STEP_OC_LIMIT_A)
  {
    six_step.overcurrent = 1U;
    sixStepFinish(MOTOR_FAULT_SW_OVERCURRENT);
    return;
  }

  const uint8_t hall = motorOpenLoopDiagReadPins();
  if (sixStepHallOrder(hall) < 0)
  {
    /* Read the three physical GPIOs directly. The EXTI software cache can
     * lag behind this 20kHz ISR by one interrupt; do not abort on that lag.
     * Reject invalid raw 000/111 and non-adjacent changes instead. */
    six_step.invalid_hall = 1U;
    sixStepFinish(MOTOR_FAULT_HALL_INVALID);
    return;
  }

  if (six_step.samples == 0U)
  {
    six_step.initial_hall = hall;
    six_step.last_hall = hall;
    if (!sixStepQueue(hall, SIX_STEP_START_DUTY))
    {
      six_step.invalid_hall = 1U;
      sixStepFinish(MOTOR_FAULT_HALL_INVALID);
      return;
    }
  }
  else if (hall != six_step.last_hall)
  {
    const int8_t old_ord = sixStepHallOrder(six_step.last_hall);
    const int8_t new_ord = sixStepHallOrder(hall);
    const int8_t delta = (int8_t)((new_ord - old_ord + 6) % 6);
    if (delta != 1 && delta != 5)
    {
      six_step.invalid_hall = 1U;
      sixStepFinish(MOTOR_FAULT_HALL_INVALID);
      return;
    }
    six_step.edges++;
    six_step.last_direction = (delta == 1) ? 1 : -1;
    if (six_step.edges >= SIX_STEP_TARGET_EDGES)
    {
      six_step.reached_edge_target = 1U;
      sixStepFinish(MOTOR_FAULT_NONE);
      return;
    }
    float progress = (float)six_step.samples / (float)SIX_STEP_RAMP_SAMPLES;
    if (progress > 1.0f) progress = 1.0f;
    const float duty = SIX_STEP_START_DUTY +
        (SIX_STEP_FINAL_DUTY - SIX_STEP_START_DUTY) * progress;
    if (!sixStepQueue(hall, duty))
    {
      six_step.invalid_hall = 1U;
      sixStepFinish(MOTOR_FAULT_HALL_INVALID);
      return;
    }
  }

  if (six_step.wait_preload)
  {
    /* While all input pins are GPIO-forced OFF, timer runs internally.
     * Wait for CCR preload update; then allow selected legs to TIM1 AF. */
    if (--six_step.wait_preload == 0U)
    {
      uint8_t plus, minus;
      if (!sixStepPhasePair(hall, &plus, &minus))
      {
        sixStepFinish(MOTOR_FAULT_HALL_INVALID);
        return;
      }
      if (pwmIsBreakFault() || (TIM1->SR & TIM_SR_BIF))
      {
        sixStepFinish(MOTOR_FAULT_BKIN);
        return;
      }
      pwmEnableOutput();  /* GPIO still holds all 6 inputs in OFF state */
      sixStepLowTimerAF(minus);   /* ONLY low-side TIM1 CHxN */
      sixStepHighTimerAF(plus);  /* ONLY high-side TIM1 CHx */
    }
  }
  else if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
  {
    six_step.lost_moe = 1U;
    sixStepFinish(MOTOR_FAULT_INIT_FAIL);
    return;
  }
  else
  {
    /* Vary CCR without changing the selected phase pair. */
    float progress = (float)six_step.samples / (float)SIX_STEP_RAMP_SAMPLES;
    if (progress > 1.0f) progress = 1.0f;
    const float duty = SIX_STEP_START_DUTY +
        (SIX_STEP_FINAL_DUTY - SIX_STEP_START_DUTY) * progress;
    uint8_t plus, minus;
    if (!sixStepPhasePair(hall, &plus, &minus))
    {
      sixStepFinish(MOTOR_FAULT_HALL_INVALID);
      return;
    }
    float d[3] = {0.0f, 0.0f, 0.0f};
    d[plus] = duty;
    pwmSetDuty(d[0], d[1], d[2]);
  }

  six_step.samples++;
  if (six_step.samples >= SIX_STEP_MAX_SAMPLES)
  {
    sixStepFinish(MOTOR_FAULT_NONE);
  }
}

void motorSixStepSelect(void)
{
  if (motor_state == MOTOR_STATE_IDLE && (TIM1->BDTR & TIM_BDTR_MOE) == 0U)
  {
    six_step = (six_step_diag_t){0};
    six_step.selected = 1U;
  }
}
bool motorSixStepIsSelected(void) { return six_step.selected != 0U; }
bool motorSixStepIsDone(void) { return six_step.complete != 0U; }
void motorSixStepPrint(void)
{
  uartPrintf(_DEF_UART1, "\r\n===== HALL 6-STEP C TEST =====\r\n");
  uartPrintf(_DEF_UART1, "Hall %u -> %u; edges %lu/3, last dir %d\r\n",
      (unsigned int)six_step.initial_hall, (unsigned int)six_step.last_hall,
      (unsigned long)six_step.edges, (int)six_step.last_direction);
  uartPrintf(_DEF_UART1, "Samples %lu / %u; Stop sample %lu, Done %u, Fault %d\r\n",
      (unsigned long)six_step.samples, (unsigned int)SIX_STEP_MAX_SAMPLES,
      (unsigned long)six_step.stop_sample,
      (unsigned int)six_step.complete, (int)motor_fault);
  uartPrintf(_DEF_UART1, "Peak Ia/Ib/Ic %ld/%ld/%ld mA; OC/BKIN/Hall/MOE %u/%u/%u/%u\r\n",
      (long)(six_step.peak_a * 1000.0f),
      (long)(six_step.peak_b * 1000.0f),
      (long)(six_step.peak_c * 1000.0f),
      (unsigned int)six_step.overcurrent, (unsigned int)six_step.bkin,
      (unsigned int)six_step.invalid_hall, (unsigned int)six_step.lost_moe);
  uartPrintf(_DEF_UART1, "Stop cause: %s\r\n",
      motor_fault != MOTOR_FAULT_NONE ? "FAULT" :
      (six_step.reached_edge_target ? "3_HALL_EDGES" : "TIME_LIMIT"));
  uartPrintf(_DEF_UART1, "C duty 2.5->5.0%% in 100ms; 250ms total; current limit %ldmA.\r\n",
      (long)(SIX_STEP_OC_LIMIT_A * 1000.0f));
  uartPrintf(_DEF_UART1, "===== END HALL 6-STEP C TEST =====\r\n");
}
/* ---------- End isolated six-step diagnostic ---------- */

/* -------- UART 'H': genuine Hall-angle current FOC, NOT a virtual sweep. ------
 * Does not change any C/S/P/A/B path. Must be armed before motorStart().
 * Require hardware BKIN, current-limited PSU and motor fixed securely.
 * Drive 650 ms max; stop after three stable Hall transitions. */
/* H: proven 2->6 baseline with staged current boost */
#define HFOC_IQ_TARGET_A         1.70f
#define HFOC_BOOST_IQ_A          2.00f
#define HFOC_BOOST_START_SAMPLES 5000U  /* 250 ms */
#define HFOC_BOOST_RAMP_SAMPLES  1000U  /* 50 ms: 1.7 -> 2.0 A */
#define HFOC_OC_LIMIT_A          2.30f  /* H ONLY; software phase-current limit */
#define JFOC_OC_LIMIT_A          1.90f  /* J stays unchanged */
#define HFOC_RAMP_SAMPLES        3000U  /* 150 ms at 20 kHz */
#define HFOC_TOTAL_SAMPLES       13000U /* 650 ms */
#define HFOC_TARGET_EDGES        3U
#define HFOC_HALL_STABLE_N     5U     /* 250 us confirmed Hall input */

/* J: FOC current-loop forced-angle spin-up, then guarded Hall handoff.
 * H remains unchanged.  Do NOT increase the 1.9 A protection threshold. */
#define JFOC_TOTAL_SAMPLES       14000U  /* 700 ms at 20 kHz */
#define JFOC_ALIGN_SAMPLES        3000U  /* 150 ms fixed-angle d-axis alignment */
#define JFOC_ALIGN_RAMP_SAMPLES    600U  /* 30 ms gradual Id rise */
#define JFOC_ALIGN_ID_A          0.50f  /* alignment is not a 1.7A torque pulse */
#define JFOC_IQ_TARGET_A         1.50f  /* lower than prior 1.7A J test */
#define JFOC_IQ_RAMP_SAMPLES     3000U  /* 150 ms from alignment end */
#define JFOC_D_TO_Q_SAMPLES      2000U  /* 100ms smooth Id release */
#define JFOC_SWEEP_START_SAMPLE  (JFOC_ALIGN_SAMPLES + JFOC_IQ_RAMP_SAMPLES)
#define JFOC_SPEED_RAMP_SAMPLES   2000U  /* 100 ms acceleration */
#define JFOC_SPEED_MAX_E          8.0f  /* electrical rad/s */
#define JFOC_BLEND_SAMPLES        1200U   /* 60 ms angle blend */
#define JFOC_HANDOFF_MAX_DIFF     1.0472f /* max 60 degrees */
#define JFOC_SUCCESS_EDGES        4U


typedef struct
{
  volatile uint32_t samples, stop_sample, edges;
  volatile float peak_a, peak_b, peak_c;
  volatile float last_iq_ref, last_iq_meas, last_vq;
  volatile float last_theta;
  volatile uint8_t selected, done, first_hall, last_hall;
  volatile uint8_t oc, bkin, invalid_hall, moe_lost;
  volatile uint8_t stopped_on_edges, vq_limited;
  uint8_t candidate_hall, candidate_count;
  volatile uint8_t forced_selected, startup_stage, phase_mismatch, reverse_edge;
  volatile uint32_t forward_edges, handoff_sample;
  volatile uint32_t h_edges_before_boost, h_edges_after_boost;
  volatile uint32_t alignment_edges;
  volatile uint8_t alignment_hall;
  volatile float commanded_theta, handoff_error;
} hall_foc_diag_t;
static hall_foc_diag_t hall_foc;

static void hallFocFinish(motor_fault_t fault)
{
  pwmDisableOutput();
  hall_foc.done = 1U;
  hall_foc.stop_sample = hall_foc.samples;
  if (fault != MOTOR_FAULT_NONE)
  {
    motor_fault = fault;
    motor_state = MOTOR_STATE_FAULT;
  }
}

static bool hallFocValidState(uint8_t hall)
{
  return hall == 1U || hall == 2U || hall == 3U ||
         hall == 4U || hall == 5U || hall == 6U;
}

static void motorHallFocUpdate(void)
{
  if (motor_state != MOTOR_STATE_OPEN_LOOP || hall_foc.done) return;

  motor_abc_f_t current;
  adcGetPhaseCurrent(&current);
  const float a = fabsf(current.a);
  const float b = fabsf(current.b);
  const float c = fabsf(current.c);
  if (a > hall_foc.peak_a) hall_foc.peak_a = a;
  if (b > hall_foc.peak_b) hall_foc.peak_b = b;
  if (c > hall_foc.peak_c) hall_foc.peak_c = c;

  if (pwmIsBreakFault() || ((TIM1->SR & TIM_SR_BIF) != 0U))
  {
    hall_foc.bkin = 1U;
    hallFocFinish(MOTOR_FAULT_BKIN);
    return;
  }
  if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
  {
    hall_foc.moe_lost = 1U;
    hallFocFinish(MOTOR_FAULT_INIT_FAIL);
    return;
  }
  const float hall_oc_limit = hall_foc.forced_selected ?
      JFOC_OC_LIMIT_A : HFOC_OC_LIMIT_A;
  if (a >= hall_oc_limit || b >= hall_oc_limit ||
      c >= hall_oc_limit)
  {
    hall_foc.oc = 1U;
    hallFocFinish(MOTOR_FAULT_SW_OVERCURRENT);
    return;
  }

  const uint8_t raw = motorOpenLoopDiagReadPins();
  if (!hallFocValidState(raw) || !hallIsValid() ||
      !hallFocValidState(hallGetState()))
  {
    hall_foc.invalid_hall = 1U;
    hallFocFinish(MOTOR_FAULT_HALL_INVALID);
    return;
  }

  if (hall_foc.samples == 0U)
  {
    hall_foc.first_hall = raw;
    hall_foc.last_hall = raw;
    hall_foc.candidate_hall = raw;
    hall_foc.candidate_count = 0U;
    hall_foc.commanded_theta = hallGetElectricalAngle();
    hall_foc.alignment_hall = raw;
  }
  else if (raw == hall_foc.last_hall)
  {
    hall_foc.candidate_hall = raw;
    hall_foc.candidate_count = 0U;
  }
  else
  {
    if (hall_foc.candidate_hall != raw)
    {
      hall_foc.candidate_hall = raw;
      hall_foc.candidate_count = 1U;
    }
    else if (hall_foc.candidate_count < HFOC_HALL_STABLE_N)
      hall_foc.candidate_count++;

    if (hall_foc.candidate_count >= HFOC_HALL_STABLE_N &&
        hallGetState() == raw)
    {
      const uint8_t bits = raw ^ hall_foc.last_hall;
      if (bits != 1U && bits != 2U && bits != 4U)
      {
        hall_foc.invalid_hall = 1U;
        hallFocFinish(MOTOR_FAULT_HALL_INVALID);
        return;
      }
      if (hall_foc.forced_selected)
      {
        if (hall_foc.startup_stage == 0U)
        {
          /* Alignment can move either way. Never miscount as startup. */
          hall_foc.alignment_edges++;
          hall_foc.alignment_hall = raw;
        }
        else
        {
          /* Confirmed +Iq direction in the prior C/H tests: 010 -> 110. */
          const int8_t prev_ord = sixStepHallOrder(hall_foc.last_hall);
          const int8_t next_ord = sixStepHallOrder(raw);
          const int8_t dir = (int8_t)((next_ord - prev_ord + 6) % 6);
          if (dir != 5)
          {
            hall_foc.reverse_edge = 1U;
            hallFocFinish(MOTOR_FAULT_HALL_INVALID);
            return;
          }
          hall_foc.forward_edges++;
          if (hall_foc.forward_edges >= 2U && hall_foc.startup_stage == 1U)
            hall_foc.startup_stage = 2U; /* start guarded Hall handoff */
        }
      }
      hall_foc.edges++;
      if (!hall_foc.forced_selected)
      {
        if (hall_foc.samples < HFOC_BOOST_START_SAMPLES)
          hall_foc.h_edges_before_boost++;
        else
          hall_foc.h_edges_after_boost++;
      }
      hall_foc.last_hall = raw;
      hall_foc.candidate_count = 0U;
      if (!hall_foc.forced_selected && hall_foc.edges >= HFOC_TARGET_EDGES)
      {
        hall_foc.stopped_on_edges = 1U;
        hallFocFinish(MOTOR_FAULT_NONE);
        return;
      }
      if (hall_foc.forced_selected && hall_foc.startup_stage == 3U &&
          hall_foc.edges >= JFOC_SUCCESS_EDGES)
      {
        hall_foc.stopped_on_edges = 1U;
        hallFocFinish(MOTOR_FAULT_NONE);
        return;
      }
    }
  }

  const uint32_t n = hall_foc.samples;
  float id_ref = 0.0f;
  float iq_ref = 0.0f;
  float theta_e;

  if (!hall_foc.forced_selected)
  {
    /* Restore the Hall angle interpolation from the H run which
     * actually recorded 010 -> 110. Do not alter J startup. */
    theta_e = hallGetElectricalAngle();

    /* H current command and ramp unchanged. */
    if (n >= 200U)
    {
      float fraction = (float)(n - 200U) / (float)HFOC_RAMP_SAMPLES;
      if (fraction > 1.0f) fraction = 1.0f;
      iq_ref = HFOC_IQ_TARGET_A * fraction;
      if (n >= HFOC_BOOST_START_SAMPLES)
      {
        float boost_frac = (float)(n - HFOC_BOOST_START_SAMPLES) /
                           (float)HFOC_BOOST_RAMP_SAMPLES;
        if (boost_frac > 1.0f) boost_frac = 1.0f;
        iq_ref += (HFOC_BOOST_IQ_A - HFOC_IQ_TARGET_A) * boost_frac;
      }
    }
  }
  else
  {
    theta_e = hallGetElectricalAngle();
    /* J stage 0: fixed stator-axis alignment. Do not rotate theta.
     * Id rises to only 0.5A; both direction and Hall edges are logged,
     * but alignment edges never qualify for the handoff. */
    if (n < JFOC_ALIGN_SAMPLES)
    {
      float fraction = (float)n / (float)JFOC_ALIGN_RAMP_SAMPLES;
      if (fraction > 1.0f) fraction = 1.0f;
      id_ref = JFOC_ALIGN_ID_A * fraction;
      theta_e = hall_foc.commanded_theta;
    }
    else
    {
      if (hall_foc.startup_stage == 0U)
      {
        hall_foc.startup_stage = 1U;
        hall_foc.forward_edges = 0U;
        /* Alignment field position is deliberately retained: don't
         * suddenly jump to a different Hall-sector center here. */
      }
      /* Stage 1: release Id as Iq ramps, then sweep the virtual angle.
       * Current PI stays closed-loop in every stage. */
      if (hall_foc.startup_stage == 1U)
      {
        float release = (float)(n - JFOC_ALIGN_SAMPLES) /
                        (float)JFOC_D_TO_Q_SAMPLES;
        if (release > 1.0f) release = 1.0f;
        id_ref = JFOC_ALIGN_ID_A * (1.0f - release);

        float rise = (float)(n - JFOC_ALIGN_SAMPLES) /
                     (float)JFOC_IQ_RAMP_SAMPLES;
        if (rise > 1.0f) rise = 1.0f;
        iq_ref = JFOC_IQ_TARGET_A * rise;

        if (n >= JFOC_SWEEP_START_SAMPLE)
        {
          float progress = (float)(n - JFOC_SWEEP_START_SAMPLE) /
                           (float)JFOC_SPEED_RAMP_SAMPLES;
          if (progress > 1.0f) progress = 1.0f;
          const float omega = JFOC_SPEED_MAX_E * progress;
          hall_foc.commanded_theta = wrapFloat(
              hall_foc.commanded_theta + omega * CUR_DT, 0.0f, 2.0f * PI);
        }
        theta_e = hall_foc.commanded_theta;
      }
      else
      {
        /* Stage 2/3: Hall handoff using the existing guarded blend. */
        iq_ref = JFOC_IQ_TARGET_A;
        const float hall_theta = hallGetElectricalAngle();
        if (hall_foc.startup_stage == 2U && hall_foc.handoff_sample == 0U)
        {
          const float delta = wrapFloat(
              hall_foc.commanded_theta - hall_theta + PI, 0.0f, 2.0f * PI) - PI;
          if (fabsf(delta) > JFOC_HANDOFF_MAX_DIFF)
          {
            hall_foc.phase_mismatch = 1U;
            hallFocFinish(MOTOR_FAULT_INIT_FAIL);
            return;
          }
          hall_foc.handoff_error = delta;
          hall_foc.handoff_sample = n;
        }
        if (hall_foc.startup_stage == 2U)
        {
          const uint32_t elapsed = n - hall_foc.handoff_sample;
          if (elapsed >= JFOC_BLEND_SAMPLES)
          {
            hall_foc.startup_stage = 3U;
            theta_e = hall_theta;
          }
          else
          {
            const float blend = 1.0f -
                (float)elapsed / (float)JFOC_BLEND_SAMPLES;
            theta_e = wrapFloat(hall_theta + hall_foc.handoff_error * blend,
                                0.0f, 2.0f * PI);
          }
        }
        else
          theta_e = hall_theta;
      }
    }
  }
  motorCurrentLoop(id_ref, iq_ref, theta_e);
  if (motor_state == MOTOR_STATE_FAULT || motor_fault != MOTOR_FAULT_NONE)
  {
    if (motor_fault == MOTOR_FAULT_SW_OVERCURRENT) hall_foc.oc = 1U;
    hallFocFinish(motor_fault);
    return;
  }

  hall_foc.last_theta = theta_e;
  hall_foc.last_iq_ref = motor_monitor.iq_ref;
  hall_foc.last_iq_meas = motor_monitor.iq_meas;
  hall_foc.last_vq = motor_monitor.vq_cmd;
  if (fabsf(motor_monitor.vq_cmd) > (OUTPUT_VQ_REF_MAX - 0.005f))
    hall_foc.vq_limited = 1U;

  hall_foc.samples = n + 1U;
  if (hall_foc.samples >=
      (hall_foc.forced_selected ? JFOC_TOTAL_SAMPLES : HFOC_TOTAL_SAMPLES))
    hallFocFinish(MOTOR_FAULT_NONE);
}

void motorHallFocSelect(void)
{
  if (motor_state == MOTOR_STATE_IDLE &&
      ((TIM1->BDTR & TIM_BDTR_MOE) == 0U))
  {
    hall_foc = (hall_foc_diag_t){0};
    hall_foc.selected = 1U;
  }
}
void motorHallFocForcedSelect(void)
{
  motorHallFocSelect();
  if (hall_foc.selected)
    hall_foc.forced_selected = 1U;
}
bool motorHallFocIsForced(void) { return hall_foc.forced_selected != 0U; }
bool motorHallFocIsSelected(void) { return hall_foc.selected != 0U; }
bool motorHallFocIsDone(void) { return hall_foc.done != 0U; }
void motorHallFocPrint(void)
{
  uartPrintf(_DEF_UART1, hall_foc.forced_selected ?
      "\r\n===== HALL-FOC J STARTUP TEST =====\r\n" :
      "\r\n===== HALL-ANGLE FOC H TEST =====\r\n");
  if (!hall_foc.forced_selected)
    uartPrintf(_DEF_UART1, "Angle: Hall interpolated (original 2->6 H baseline)\r\n");
  if (!hall_foc.forced_selected)
  {
    uartPrintf(_DEF_UART1,
        "H: Iq 1.7A -> 2.0A after 250ms (50ms ramp), SW phase OC 2.30A, max 650ms\r\n");
    uartPrintf(_DEF_UART1,
        "H edges before/after boost: %lu / %lu\r\n",
        (unsigned long)hall_foc.h_edges_before_boost,
        (unsigned long)hall_foc.h_edges_after_boost);
  }
  if (hall_foc.forced_selected)
    uartPrintf(_DEF_UART1,
        "J stage %u (0 align, 1 forced, 2 blend, 3 Hall); forward edges %lu; reverse/phase error %u/%u; handoff error %ld mrad\r\n",
        (unsigned int)hall_foc.startup_stage,
        (unsigned long)hall_foc.forward_edges,
        (unsigned int)hall_foc.reverse_edge,
        (unsigned int)hall_foc.phase_mismatch,
        (long)(hall_foc.handoff_error * 1000.0f));
  if (hall_foc.forced_selected)
    uartPrintf(_DEF_UART1, "J alignment: %lu Hall edges, last Hall %u; Id align %ld mA; Iq drive %ld mA\r\n",
        (unsigned long)hall_foc.alignment_edges,
        (unsigned int)hall_foc.alignment_hall,
        (long)(JFOC_ALIGN_ID_A * 1000.0f),
        (long)(JFOC_IQ_TARGET_A * 1000.0f));
  uartPrintf(_DEF_UART1, "Hall %u -> %u, confirmed edges %lu/%u\r\n",
      (unsigned int)hall_foc.first_hall, (unsigned int)hall_foc.last_hall,
      (unsigned long)hall_foc.edges,
      (unsigned int)(hall_foc.forced_selected ? JFOC_SUCCESS_EDGES : HFOC_TARGET_EDGES));
  uartPrintf(_DEF_UART1, "Samples %lu/%u (stop %lu), done %u, fault %d\r\n",
      (unsigned long)hall_foc.samples,
      (unsigned int)(hall_foc.forced_selected ? JFOC_TOTAL_SAMPLES : HFOC_TOTAL_SAMPLES),
      (unsigned long)hall_foc.stop_sample,
      (unsigned int)hall_foc.done, (int)motor_fault);
  uartPrintf(_DEF_UART1, "Peak |Ia/Ib/Ic| %ld/%ld/%ld mA; OC/BKIN/Hall/MOE %u/%u/%u/%u\r\n",
      (long)(hall_foc.peak_a * 1000.0f),
      (long)(hall_foc.peak_b * 1000.0f),
      (long)(hall_foc.peak_c * 1000.0f),
      (unsigned int)hall_foc.oc, (unsigned int)hall_foc.bkin,
      (unsigned int)hall_foc.invalid_hall,
      (unsigned int)hall_foc.moe_lost);
  uartPrintf(_DEF_UART1, "Last Iq ref/meas %ld/%ld mA, Vq %ld mV, theta %ld mrad, Vq limited %u\r\n",
      (long)(hall_foc.last_iq_ref*1000.0f),
      (long)(hall_foc.last_iq_meas*1000.0f),
      (long)(hall_foc.last_vq*1000.0f),
      (long)(hall_foc.last_theta*1000.0f),
      (unsigned int)hall_foc.vq_limited);
  uartPrintf(_DEF_UART1, "Stop: %s; Iq target %ld mA (150ms ramp), SW OC %ld mA, max %lums.\r\n",
      motor_fault != MOTOR_FAULT_NONE ? "FAULT" :
        (hall_foc.stopped_on_edges ? "HALL_EDGES" : "TIME_LIMIT"),
      (long)((hall_foc.forced_selected ? JFOC_IQ_TARGET_A : HFOC_IQ_TARGET_A)*1000.0f),
      (long)((hall_foc.forced_selected ? JFOC_OC_LIMIT_A : HFOC_OC_LIMIT_A)*1000.0f),
      (unsigned long)(hall_foc.forced_selected ? 700U : 650U));
  uartPrintf(_DEF_UART1, hall_foc.forced_selected ?
      "===== END HALL-FOC J STARTUP TEST =====\r\n" :
      "===== END HALL-ANGLE FOC H TEST =====\r\n");
}
/* -------- End H test -------- */



// P-command phase-to-current-sensor mapping test. Never runs alongside S/F/N/R.
static volatile bool phase_map_diag_selected = false;
static volatile phase_map_diag_result_t phase_map_diag;

static void motorPhaseMapStop(bool fault, motor_fault_t reason)
{
  pwmDisableOutput();
  phase_map_diag.complete = 1U;
  if (fault)
  {
    motor_fault = reason;
    motor_state = MOTOR_STATE_FAULT;
  }
}

// Direct SPWM voltage pulses, deliberately independent of Park/PI/current feedback.
// Six pulses separated by >15ms neutral allow checking actual current-sensor
// channel order AND polarity. All checks and automatic PWM off are in ADC ISR.
static void motorPhaseMapDiagUpdate(void)
{
  if (motor_state != MOTOR_STATE_OPEN_LOOP || phase_map_diag.complete) return;

  // ap.c turns on MOE after motorStart() and 1000 injected offset samples.
  if (phase_map_diag.samples == 0U &&
      ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)) return;

  const uint32_t n = phase_map_diag.samples;
  motor_abc_f_t iabc;
  adcGetPhaseCurrent(&iabc);
  motor_monitor.ia = iabc.a;
  motor_monitor.ib = iabc.b;
  motor_monitor.ic = iabc.c;

  const float a = fabsf(iabc.a);
  const float b = fabsf(iabc.b);
  const float c = fabsf(iabc.c);
  if (a > phase_map_diag.max_abs_current) phase_map_diag.max_abs_current = a;
  if (b > phase_map_diag.max_abs_current) phase_map_diag.max_abs_current = b;
  if (c > phase_map_diag.max_abs_current) phase_map_diag.max_abs_current = c;

  if (pwmIsBreakFault() || ((TIM1->SR & TIM_SR_BIF) != 0U))
  {
    phase_map_diag.bkin_trip = 1U;
    motorPhaseMapStop(true, MOTOR_FAULT_BKIN);
    return;
  }
  if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
  {
    phase_map_diag.moe_lost = 1U;
    motorPhaseMapStop(true, MOTOR_FAULT_INIT_FAIL);
    return;
  }
  if (a > PHASE_MAP_OC_LIMIT_A || b > PHASE_MAP_OC_LIMIT_A || c > PHASE_MAP_OC_LIMIT_A)
  {
    phase_map_diag.oc_trip = 1U;
    motorPhaseMapStop(true, MOTOR_FAULT_SW_OVERCURRENT);
    return;
  }

  // Stop even if a Hall edge is detected; this is a mapping test, not a rotor test.
  const uint8_t hall_now = motorOpenLoopDiagReadPins();
  if (hall_now != phase_map_diag.hall_start)
  {
    phase_map_diag.hall_moved = 1U;
    motorPhaseMapStop(false, MOTOR_FAULT_NONE);
    return;
  }

  const uint32_t test_index = n / PHASE_MAP_FRAME_SAMPLES;
  const uint32_t pos = n % PHASE_MAP_FRAME_SAMPLES;
  if (test_index >= PHASE_MAP_TEST_COUNT)
  {
    motorPhaseMapStop(false, MOTOR_FAULT_NONE);
    return;
  }

  // The last 16 current samples of every neutral baseline.
  if (pos >= PHASE_MAP_BASELINE_SAMPLES - 16U &&
      pos < PHASE_MAP_BASELINE_SAMPLES)
  {
    phase_map_diag.measures[test_index].base_a += iabc.a;
    phase_map_diag.measures[test_index].base_b += iabc.b;
    phase_map_diag.measures[test_index].base_c += iabc.c;
    phase_map_diag.measures[test_index].base_count++;
  }

  // Pulse is 8 ISR intervals; use last 4 measured samples (after PWM update).
  if (pos >= PHASE_MAP_BASELINE_SAMPLES + 4U &&
      pos < PHASE_MAP_BASELINE_SAMPLES + PHASE_MAP_PULSE_SAMPLES)
  {
    phase_map_diag.measures[test_index].pulse_a += iabc.a;
    phase_map_diag.measures[test_index].pulse_b += iabc.b;
    phase_map_diag.measures[test_index].pulse_c += iabc.c;
    phase_map_diag.measures[test_index].pulse_count++;
  }

  if (pos == 0U)
  {
    pwmSetDuty(0.5f, 0.5f, 0.5f);
  }
  if (pos == PHASE_MAP_BASELINE_SAMPLES)
  {
    // 3-wire star: phase voltage commands +V, -V/2, -V/2.
    // U+,U-,V+,V-,W+,W- in order; delta is duty fraction.
    const float delta = PHASE_MAP_VOLTAGE_V / motor_vbus;
    const float sign = (test_index % 2U == 0U) ? 1.0f : -1.0f;
    float du = 0.5f - sign * (delta * 0.5f);
    float dv = du;
    float dw = du;
    if (test_index / 2U == 0U) du = 0.5f + sign * delta;
    else if (test_index / 2U == 1U) dv = 0.5f + sign * delta;
    else dw = 0.5f + sign * delta;
    pwmSetDuty(du, dv, dw);
  }
  else if (pos == PHASE_MAP_BASELINE_SAMPLES + PHASE_MAP_PULSE_SAMPLES)
  {
    pwmSetDuty(0.5f, 0.5f, 0.5f);
  }

  phase_map_diag.samples = n + 1U;
  if (phase_map_diag.samples >= PHASE_MAP_TOTAL_SAMPLES)
  {
    motorPhaseMapStop(false, MOTOR_FAULT_NONE);
  }
}

void motorPhaseMapDiagSelect(void)
{
  if (motor_state == MOTOR_STATE_IDLE && ((TIM1->BDTR & TIM_BDTR_MOE) == 0U))
  {
    phase_map_diag_selected = true;
    phase_map_diag = (phase_map_diag_result_t){0};
  }
}

bool motorPhaseMapDiagIsSelected(void) { return phase_map_diag_selected; }
bool motorPhaseMapDiagIsDone(void) { return phase_map_diag.complete != 0U; }

void motorPhaseMapDiagGet(phase_map_diag_result_t *out)
{
  if (out == NULL) return;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *out = phase_map_diag;
  if (primask == 0U) __enable_irq();
}


// Independent of hall.c cache and EXTI.  The firmware has three Hall GPIOs.
static uint8_t motorOpenLoopDiagReadPins(void)
{
  uint8_t state = 0U;
  if ((GPIOA->IDR & GPIO_PIN_15) != 0U) state |= 1U;
  if ((GPIOB->IDR & GPIO_PIN_3) != 0U) state |= 2U;
  if ((GPIOB->IDR & GPIO_PIN_10) != 0U) state |= 4U;
  return state;
}

static void motorOpenLoopDiagRecordEdge(uint32_t n, uint8_t raw, uint8_t cached,
                                        const motor_abc_f_t *current)
{
  uint32_t k = open_loop_diag.edge_count;
  if (k < OPEN_LOOP_DIAG_MAX_EDGES)
  {
    open_loop_diag.edges[k].sample = n;
    open_loop_diag.edges[k].hall_raw = raw;
    open_loop_diag.edges[k].hall_cached = cached;
    open_loop_diag.edges[k].hall_sector = hallGetSectorIndex();
    open_loop_diag.edges[k].theta_e = open_loop_theta_e;
    open_loop_diag.edges[k].ia = current->a;
    open_loop_diag.edges[k].ib = current->b;
    open_loop_diag.edges[k].ic = current->c;
    open_loop_diag.edge_count = k + 1U;
  }
  else
  {
    open_loop_diag.edge_overflow++;
  }
}

static void motorOpenLoopDiagLog(uint32_t n, uint8_t raw, uint8_t cached,
                                 const motor_abc_f_t *current)
{
  uint32_t k = open_loop_diag.log_count;
  if (k >= OPEN_LOOP_DIAG_MAX_LOGS) return;

  open_loop_diag.log[k].sample = n;
  open_loop_diag.log[k].hall_raw = raw;
  open_loop_diag.log[k].hall_cached = cached;
  open_loop_diag.log[k].hall_valid = hallIsValid() ? 1U : 0U;
  open_loop_diag.log[k].hall_sector = hallGetSectorIndex();
  open_loop_diag.log[k].hall_irq = hallGetIrqCount();
  open_loop_diag.log[k].hall_transition = hallGetTransitionCount();
  open_loop_diag.log[k].theta_e = open_loop_theta_e;
  open_loop_diag.log[k].ia = current->a;
  open_loop_diag.log[k].ib = current->b;
  open_loop_diag.log[k].ic = current->c;
  open_loop_diag.log[k].iq_ref = motor_monitor.iq_ref;
  open_loop_diag.log[k].id_meas = motor_monitor.id_meas;
  open_loop_diag.log[k].iq_meas = motor_monitor.iq_meas;
  open_loop_diag.log[k].vd = open_loop_vd;
  open_loop_diag.log[k].vq = open_loop_vq;
  open_loop_diag.log[k].duty_u = motor_duty.u;
  open_loop_diag.log[k].duty_v = motor_duty.v;
  open_loop_diag.log[k].duty_w = motor_duty.w;
  open_loop_diag.log_count = k + 1U;
}

// Bounded virtual-angle rotating-current-vector diagnostic.
// Park/InvPark use the VIRTUAL angle, not a Hall sector centre once armed.
// Hall GPIO/EXTI is observed solely to check physical rotor motion/direction.
// Stop at 3 valid edges, any opposite-direction edge, fault or 1 s timeout.
static void motorOpenLoopDiagnostic(void)
{
  if (motor_state != MOTOR_STATE_OPEN_LOOP || open_loop_diag.completed) return;

  // MOE is armed by ap.c only after motorStart() returns.
  if (((TIM1->BDTR & TIM_BDTR_MOE) == 0U) &&
      (open_loop_diag.sample_count == 0U)) return;

  uint32_t n = open_loop_diag.sample_count;

  // 700ms 시점에 오실로스코프 트리거 발생
  // PA5 = Nucleo LD2, 현재 프로젝트에서 GPIO 출력으로 초기화됨

  if (n == 0U)
  {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  }
  else if (n == 14000U)
  {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  }
  else if (n == 14002U)
  {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  }

  motor_abc_f_t current;
  adcGetPhaseCurrent(&current);
  motor_monitor.ia = current.a;
  motor_monitor.ib = current.b;
  motor_monitor.ic = current.c;

  // RAW GPIO, separately from the Hall EXTI/cache pathway.
  uint8_t raw = motorOpenLoopDiagReadPins();
  uint8_t cached = hallGetState();
  bool raw_changed = (raw != open_loop_diag_prev_raw);
  bool cache_changed = (cached != open_loop_diag_prev_cached);
  if (raw_changed)
  {
    open_loop_diag.raw_edge_total++;
    motorOpenLoopDiagRecordEdge(n, raw, cached, &current);
    open_loop_diag_prev_raw = raw;
  }
  if (cache_changed)
  {
    open_loop_diag.cached_edge_total++;
    if (!raw_changed)
      motorOpenLoopDiagRecordEdge(n, raw, cached, &current);
    open_loop_diag_prev_cached = cached;
  }

  float a = fabsf(current.a), b = fabsf(current.b), c = fabsf(current.c);
  if (a > open_loop_diag.peak_a) open_loop_diag.peak_a = a;
  if (b > open_loop_diag.peak_b) open_loop_diag.peak_b = b;
  if (c > open_loop_diag.peak_c) open_loop_diag.peak_c = c;

  // Hardware Break / MOE / measured phase-current guard, before new PWM duty.
  // Independent fixed-Id alignment tests use a stricter current threshold.
  const bool fixed_id_test = (open_loop_diag.fixed_only == OL_DIAG_MODE_FIXED_ID_0 ||
                              open_loop_diag.fixed_only == OL_DIAG_MODE_FIXED_ID_90);
  const float diag_oc_limit = fixed_id_test ? OL_FIXED_ID_OC_LIMIT_A : OPEN_LOOP_DIAG_OC_LIMIT_A;
  if ((pwmIsBreakFault() == true) || ((TIM1->SR & TIM_SR_BIF) != 0U))
  {
    open_loop_diag.bkin_trip = 1U;
    motor_fault = MOTOR_FAULT_BKIN;
  }
  else if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
  {
    open_loop_diag.early_moe_off = 1U;
    motor_fault = MOTOR_FAULT_INIT_FAIL;
  }
  else if ((a > diag_oc_limit) ||
           (b > diag_oc_limit) ||
           (c > diag_oc_limit))
  {
    open_loop_diag.oc_trip = 1U;
    open_loop_diag.trip_a = current.a;
    open_loop_diag.trip_b = current.b;
    open_loop_diag.trip_c = current.c;
    open_loop_diag.trip_sample = n;
    motor_fault = MOTOR_FAULT_SW_OVERCURRENT;
  }

  if (motor_fault != MOTOR_FAULT_NONE)
  {
    pwmDisableOutput();
    motor_state = MOTOR_STATE_FAULT;
    open_loop_diag.stop_sample = n;
    open_loop_diag.sample_count = n + 1U;
    open_loop_diag.irq_at_end = hallGetIrqCount();
    open_loop_diag.transition_at_end = hallGetTransitionCount();
    open_loop_diag.completed = 1U;
    return;
  }

  // Capture Hall counts at the end of the 10 ms zero-current settling period.
  if (n == OPEN_LOOP_DIAG_SETTLE_SAMPLES)
  {
    open_loop_diag.raw_edges_before_drive = open_loop_diag.raw_edge_total;
    open_loop_diag.cached_edges_before_drive = open_loop_diag.cached_edge_total;
    open_loop_diag_last_confirmed_sector = hallGetSectorIndex();
    open_loop_diag_confirmed_direction = 0;
  }

  // A broken Hall line must never drive the controller into an invalid sector.
  if (raw == 0U || raw == 7U || cached == 0U || cached == 7U ||
      !hallIsValid() || hallGetSectorIndex() < 0 || hallGetSectorIndex() > 5)
  {
    motor_fault = MOTOR_FAULT_HALL_INVALID;
  }

  // GPIO transitions can precede EXTI cache updates by a few ISR cycles.
  if (raw != cached)
  {
    if (open_loop_diag_mismatch_samples < 6U)
      open_loop_diag_mismatch_samples++;
    if (open_loop_diag_mismatch_samples >= 5U)
      motor_fault = MOTOR_FAULT_HALL_INVALID;
  }
  else
  {
    open_loop_diag_mismatch_samples = 0U;
  }

  if (motor_fault != MOTOR_FAULT_NONE)
  {
    pwmDisableOutput();
    motor_state = MOTOR_STATE_FAULT;
    open_loop_diag.stop_sample = n;

    open_loop_diag.sample_count = n + 1U;
    open_loop_diag.irq_at_end = hallGetIrqCount();
    open_loop_diag.transition_at_end = hallGetTransitionCount();
    open_loop_diag.completed = 1U;
    return;
  }

  // Count only confirmed adjacent sector transitions, with consistent direction.
  // This rejects a Hall wiring glitch that jumps two or more sectors.
  if (n > OPEN_LOOP_DIAG_SETTLE_SAMPLES && raw == cached &&
      hallGetSectorIndex() != open_loop_diag_last_confirmed_sector)
  {
    const int8_t now_sector = hallGetSectorIndex();
    const int8_t step = (int8_t)((now_sector - open_loop_diag_last_confirmed_sector + 6) % 6);
    const int8_t direction = (step == 1) ? 1 : ((step == 5) ? -1 : 0);
    if (direction == 0 ||
        (open_loop_diag_confirmed_direction != 0 &&
         direction != open_loop_diag_confirmed_direction))
    {
      motor_fault = MOTOR_FAULT_HALL_INVALID;
      pwmDisableOutput();
      motor_state = MOTOR_STATE_FAULT;
      open_loop_diag.stop_sample = n;
      open_loop_diag.sample_count = n + 1U;
      open_loop_diag.irq_at_end = hallGetIrqCount();
      open_loop_diag.transition_at_end = hallGetTransitionCount();
      open_loop_diag.completed = 1U;
      return;
    }
    open_loop_diag_last_confirmed_sector = now_sector;
    open_loop_diag_confirmed_direction = direction;
    open_loop_diag.confirmed_edges++;
    if (n < OPEN_LOOP_DIAG_SWEEP_START_SAMPLES)
      open_loop_diag.fixed_confirmed_edges++;
    else
      open_loop_diag.sweep_confirmed_edges++;

    // hall.c maps increasing electrical angle to DECREASING sector index.
    // Stop immediately if first real adjacent Hall step is opposite to the
    // requested field-rotation direction; this is a diagnostic, not a fault.
    const int8_t expected_step = (int8_t)(-open_loop_diag.command_direction);
    if (!open_loop_diag.fixed_only &&
        n >= OPEN_LOOP_DIAG_SWEEP_START_SAMPLES &&
        direction != expected_step)
    {
      pwmDisableOutput();
      open_loop_diag.stopped_on_wrong_direction = 1U;
      open_loop_diag.stop_sample = n;
      motorOpenLoopDiagLog(n, raw, cached, &current);
      open_loop_diag.sample_count = n + 1U;
      open_loop_diag.irq_at_end = hallGetIrqCount();
      open_loop_diag.transition_at_end = hallGetTransitionCount();
      open_loop_diag.completed = 1U;
      return;
    }
  }

  // Test F/N: first confirmed Hall edge proves a detectable step in the fixed field.
  // Test S/R: stop at three confirmed edges to avoid prolonged uncontrolled spin.
  const uint32_t edge_target = open_loop_diag.fixed_only ? 1U : OPEN_LOOP_DIAG_TARGET_EDGES;
  if (n >= OPEN_LOOP_DIAG_SETTLE_SAMPLES &&
      open_loop_diag.confirmed_edges >= edge_target &&
      raw == cached)
  {
    pwmDisableOutput();
    open_loop_diag.stopped_on_hall_edge = 1U;
    open_loop_diag.stop_sample = n;
    motorOpenLoopDiagLog(n, raw, cached, &current);
    open_loop_diag.sample_count = n + 1U;
    open_loop_diag.irq_at_end = hallGetIrqCount();
    open_loop_diag.transition_at_end = hallGetTransitionCount();
    open_loop_diag.completed = 1U;
    return;
  }

  // T: absolute theta=0 stator field, 0.8A Id alignment, 0.8A Iq,
  // then virtual field rotation; Hall is OBSERVATION ONLY.
  // S/R: retained original Hall-estimated initial stator field and 0.4A.
  // Id->Iq blend and theta advance are driven entirely by sample count.
  // A Hall centre is an estimate: physical alignment is NOT guaranteed.
  const float iq_target = open_loop_diag.absolute_start ?
                          ABS_SWEEP_I_A : OPEN_LOOP_DIAG_IQ_REF_A;
  const float align_target = open_loop_diag.absolute_start ?
                             ABS_SWEEP_ALIGN_I_A : OPEN_LOOP_DIAG_ALIGN_ID_A;
  float id_cmd = 0.0f;
  float iq_cmd = 0.0f;
  if (n >= OPEN_LOOP_DIAG_SETTLE_SAMPLES)
  {
    const uint32_t drive_n = n - OPEN_LOOP_DIAG_SETTLE_SAMPLES;
    float fraction = (float)drive_n / (float)OPEN_LOOP_DIAG_RAMP_SAMPLES;
    if (fraction > 1.0f) fraction = 1.0f;

    if (fixed_id_test)
    {
      // A/B: absolute fixed stator field; Hall angle does not set theta.
      // 0 -> 0.4 A in 100 ms; hold for 100 ms; Iq=0 throughout.
      id_cmd = OL_FIXED_ID_REF_A * fraction;
      iq_cmd = 0.0f;
    }
    else if (open_loop_diag.fixed_only == OL_DIAG_MODE_FIXED_IQ)
    {
      iq_cmd = (float)open_loop_diag.command_direction *
               iq_target * fraction;
    }
    else if (n < OPEN_LOOP_DIAG_FIXED_END_SAMPLES)
    {
      id_cmd = align_target * fraction;
    }
    else if (n < OPEN_LOOP_DIAG_SWEEP_START_SAMPLES)
    {
      const float blend = (float)(n - OPEN_LOOP_DIAG_FIXED_END_SAMPLES) /
                          (float)OPEN_LOOP_DIAG_DQ_BLEND_SAMPLES;
      id_cmd = align_target * (1.0f - blend);
      iq_cmd = (float)open_loop_diag.command_direction *
               iq_target * blend;
    }
    else
    {
      iq_cmd = (float)open_loop_diag.command_direction * iq_target;
      const float delta = (float)open_loop_diag.command_direction *
                          OPEN_LOOP_DIAG_SWEEP_SPEED_E * CUR_DT;
      open_loop_diag.angle_swept_e += delta;
      open_loop_theta_e = wrapFloat(open_loop_theta_e + delta,
                                   0.0f, 2.0f * PI);
    }
  }
  open_loop_vd = 0.0f; // overwritten by the PI monitor below
  open_loop_vq = 0.0f;
  motorCurrentLoop(id_cmd, iq_cmd, open_loop_theta_e);

  // The current controller has its own software OC protection. Its trip must
  // produce a completed diagnostic so ap.c can stop peripherals and print it.
  if (motor_state == MOTOR_STATE_FAULT || motor_fault != MOTOR_FAULT_NONE)
  {
    if (motor_fault == MOTOR_FAULT_SW_OVERCURRENT)
    {
      open_loop_diag.oc_trip = 1U;
      open_loop_diag.trip_a = current.a;
      open_loop_diag.trip_b = current.b;
      open_loop_diag.trip_c = current.c;
      open_loop_diag.trip_sample = n;
    }
    pwmDisableOutput();
    open_loop_diag.stop_sample = n;
    open_loop_diag.sample_count = n + 1U;
    open_loop_diag.irq_at_end = hallGetIrqCount();
    open_loop_diag.transition_at_end = hallGetTransitionCount();
    open_loop_diag.completed = 1U;
    return;
  }

  open_loop_vd = motor_monitor.vd_cmd;
  open_loop_vq = motor_monitor.vq_cmd;

  // Mode-specific tracking summary; the existing diagnostic structure stores
  // tracking error in its historical iq_* fields for compatibility.
  if (n >= (open_loop_diag.fixed_only ?
            (OPEN_LOOP_DIAG_SETTLE_SAMPLES + OPEN_LOOP_DIAG_RAMP_SAMPLES) :
            OPEN_LOOP_DIAG_SWEEP_START_SAMPLES))
  {
    const float axis_err = fixed_id_test ?
        fabsf(id_cmd - motor_monitor.id_meas) :
        fabsf(motor_monitor.iq_ref - motor_monitor.iq_meas);
    if (axis_err > open_loop_diag.max_iq_tracking_error)
      open_loop_diag.max_iq_tracking_error = axis_err;
    if (axis_err > 0.2f)
      open_loop_diag.poor_iq_tracking_samples++;
    if (fixed_id_test)
    {
      if (fabsf(open_loop_vd) >= (OUTPUT_VD_REF_MAX - 0.005f))
        open_loop_diag.vq_limit_samples++;
    }
    else if (fabsf(open_loop_vq) >= (OUTPUT_VQ_REF_MAX - 0.005f))
      open_loop_diag.vq_limit_samples++;
  }

  if ((n % OPEN_LOOP_DIAG_LOG_DIVIDER) == 0U ||
      n == OPEN_LOOP_DIAG_SETTLE_SAMPLES ||
      n == (OPEN_LOOP_DIAG_SETTLE_SAMPLES + 200U))
  {
    motorOpenLoopDiagLog(n, raw, cached, &current);
  }

  open_loop_diag.sample_count = n + 1U;
  const uint32_t end_samples = fixed_id_test ? OL_FIXED_ID_END_SAMPLES :
                                (open_loop_diag.fixed_only ?
                                 OPEN_LOOP_DIAG_FIXED_END_SAMPLES :
                                 OPEN_LOOP_DIAG_TOTAL_SAMPLES);
  if (open_loop_diag.sample_count >= end_samples)
  {
    motorOpenLoopDiagLog(n, raw, cached, &current);
    pwmDisableOutput(); // ISR-bounded timeout, not dependent on UART task
    open_loop_diag.stop_sample = open_loop_diag.sample_count;
    open_loop_diag.irq_at_end = hallGetIrqCount();
    open_loop_diag.transition_at_end = hallGetTransitionCount();
    open_loop_diag.completed = 1U;
  }
}
#endif  // OPEN_LOOP_DIAG_ENABLE




#if _USE_HALL_OFFSET_CALIBRATION

static volatile bool hall_cal_event_pending = false;
static volatile int8_t hall_cal_event_sector = -1;
static volatile int8_t hall_cal_event_direction = 0;
static volatile float hall_cal_event_theta_e = 0.0f;

static int8_t hall_cal_prev_sector = -1;

#endif
#endif
#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_ADC_RAW_SCAN_ENABLE == 1U)



static void motorAdcRawScanUpdateStreak(
    const uint16_t raw[3],
    const float offset[3],
    const float current[3])
{
  bool first_oc_found = false;

  for (uint32_t i = 0U; i < 3U; i++)
  {
    float raw_error = fabsf((float)raw[i] - offset[i]);

    bool spike = (raw_error >= (float)CURRENT_ADC_RAW_SPIKE_COUNTS);
    bool overcurrent = (fabsf(current[i]) >= CURRENT_ADC_SCAN_OC_LIMIT_A);

    // 1. 스파이크 연속 발생 길이
    if (spike)
    {
      adc_raw_scan.spike_streak[i]++;

      if (adc_raw_scan.spike_streak[i] > adc_raw_scan.spike_max_streak[i])
      {
        adc_raw_scan.spike_max_streak[i] = adc_raw_scan.spike_streak[i];
      }

      if (adc_raw_scan.spike_streak[i] >= 2U)
      {
        adc_raw_scan.spike_adjacent_pairs[i]++;
      }
    }
    else
    {
      adc_raw_scan.spike_streak[i] = 0U;
    }

    // 2. 0.8A 이상 측정값 기록
    if (overcurrent)
    {
      adc_raw_scan.oc_sample_count[i]++;
      adc_raw_scan.oc_streak[i]++;

      if (adc_raw_scan.oc_streak[i] > adc_raw_scan.oc_max_streak[i])
      {
        adc_raw_scan.oc_max_streak[i] = adc_raw_scan.oc_streak[i];
      }

      if (adc_raw_scan.first_oc_sample == 0U)
      {
        first_oc_found = true;
      }
    }
    else
    {
      adc_raw_scan.oc_streak[i] = 0U;
    }
  }

  // 3. 최초 과전류 측정값의 3상 Raw 저장
  if (first_oc_found &&
      adc_raw_scan.first_oc_sample == 0U)
  {
      adc_raw_scan.first_oc_sample =
          adc_raw_scan.sample_count + 1U;

      adc_raw_scan.first_oc_raw[0] = raw[0];
      adc_raw_scan.first_oc_raw[1] = raw[1];
      adc_raw_scan.first_oc_raw[2] = raw[2];
  }
}


static void motorAdcRawScanUpdate(void)
{
  motor_abc_u16_t raw;
  motor_abc_f_t offset;
  motor_abc_f_t current;

  adcGetCurrentRaw(&raw);
  adcGetCurrentOffset(&offset);
  adcGetPhaseCurrent(&current);

  uint16_t r[3] = {raw.a, raw.b, raw.c};
  float off[3] = {offset.a, offset.b, offset.c};
  float amp[3] = {current.a, current.b, current.c};

  // 기존 motorAdcRawScanUpdate() 내부에서 사용한
  // r[], off[], current[] 배열을 그대로 전달
  motorAdcRawScanUpdateStreak(r, off, amp);

  uint32_t n = adc_raw_scan.sample_count;

#if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U

  // 0~3번 샘플: Startup
  // 4번 이후: Steady
  uint32_t group = (n < ADC_SCAN_STARTUP_SAMPLES) ? 0U : 1U;

  for (uint32_t i = 0U; i < 3U; i++)
  {
    if (split_sample_count[group] == 0U)
    {
      split_raw_min[group][i] = r[i];
      split_raw_max[group][i] = r[i];
    }
    else
    {
      if (r[i] < split_raw_min[group][i])
        split_raw_min[group][i] = r[i];

      if (r[i] > split_raw_max[group][i])
        split_raw_max[group][i] = r[i];
    }

    if (fabsf((float)r[i] - off[i]) >= (float)CURRENT_ADC_RAW_SPIKE_COUNTS)
    {
      split_spike_count[group][i]++;
    }
  }

  split_sample_count[group]++;

#endif


  // 첫 샘플에서 Min/Max 초기화
  for (uint32_t i = 0U; i < 3U; i++)
  {
    if (n == 0U)
    {
      adc_raw_scan.raw_min[i] = r[i];
      adc_raw_scan.raw_max[i] = r[i];

      adc_raw_scan.current_min[i] = amp[i];
      adc_raw_scan.current_max[i] = amp[i];
    }
    else
    {
      if (r[i] < adc_raw_scan.raw_min[i])
          adc_raw_scan.raw_min[i] = r[i];

      if (r[i] > adc_raw_scan.raw_max[i])
          adc_raw_scan.raw_max[i] = r[i];

      if (amp[i] < adc_raw_scan.current_min[i])
          adc_raw_scan.current_min[i] = amp[i];

      if (amp[i] > adc_raw_scan.current_max[i])
          adc_raw_scan.current_max[i] = amp[i];
    }
  }

  // Offset 대비 이상 샘플 검사
  for (uint32_t i = 0U; i < 3U; i++)
  {
    float diff = fabsf((float)r[i] - off[i]);

    if (diff >= (float)CURRENT_ADC_RAW_SPIKE_COUNTS)
    {
      adc_raw_scan.spike_count[i]++;

      // 첫 이상 샘플의 세 채널을 동시에 저장
      if (adc_raw_scan.first_spike_sample == 0U)
      {
        adc_raw_scan.first_spike_sample = n + 1U;

        adc_raw_scan.first_spike_raw[0] = r[0];
        adc_raw_scan.first_spike_raw[1] = r[1];
        adc_raw_scan.first_spike_raw[2] = r[2];
      }
    }
  }

  // 기존 모니터에도 현재 센싱값 반영
  motor_monitor.ia = current.a;
  motor_monitor.ib = current.b;
  motor_monitor.ic = current.c;

  motor_monitor.id_ref = 0.0f;
  motor_monitor.iq_ref = 0.0f;
  motor_monitor.id_meas = 0.0f;
  motor_monitor.iq_meas = 0.0f;

  motor_monitor.vd_cmd = 0.0f;
  motor_monitor.vq_cmd = 0.0f;

  motor_monitor.duty_u = 0.0f;
  motor_monitor.duty_v = 0.0f;
  motor_monitor.duty_w = 0.0f;

  adc_raw_scan.sample_count++;
}

void motorAdcRawScanGetStats(motor_adc_raw_scan_t *stats)
{
  if (stats == NULL)
  {
      return;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  // ISR에서 갱신되는 전체 구조체 복사
  *stats = adc_raw_scan;

  // 모든 필드 초기화
  adc_raw_scan = (motor_adc_raw_scan_t){0};

#if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U

  for (uint32_t g = 0U; g < 2U; g++)
  {
    split_sample_count[g] = 0U;

    for (uint32_t i = 0U; i < 3U; i++)
    {
      split_raw_min[g][i] = 0U;
      split_raw_max[g][i] = 0U;
      split_spike_count[g][i] = 0U;
    }
  }

  adc_pwm_on_scan_done = false;
  adc_pwm_on_scan_seen = false;

#endif


  __set_PRIMASK(primask);
}



#if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U

void motorAdcRawScanGetSplitStats(
    uint16_t raw_min[2][3],
    uint16_t raw_max[2][3],
    uint32_t spike_count[2][3])
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  for (uint32_t g = 0U; g < 2U; g++)
  {
    for (uint32_t i = 0U; i < 3U; i++)
    {
      raw_min[g][i] = split_raw_min[g][i];
      raw_max[g][i] = split_raw_max[g][i];
      spike_count[g][i] =
          split_spike_count[g][i];
    }
  }

  __set_PRIMASK(primask);
}

#endif



#if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U

bool motorAdcPwmOnScanIsDone(void)
{
    return adc_pwm_on_scan_done;
}

#endif

#endif



bool motorInit(void)
{
  bool ret = true;

  motor_state = MOTOR_STATE_IDLE;
  motor_fault = MOTOR_FAULT_NONE;

  if(pidInit(&pi_id, CUR_KP, CUR_KI, CUR_KD, OUTPUT_VD_REF_MIN, OUTPUT_VD_REF_MAX, INTEGRAL_ID_MIN, INTEGRAL_ID_MAX) != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
  if(pidInit(&pi_iq, CUR_KP, CUR_KI, CUR_KD, OUTPUT_VQ_REF_MIN, OUTPUT_VQ_REF_MAX, INTEGRAL_IQ_MIN, INTEGRAL_IQ_MAX) != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
  if(pidInit(&pi_spd, SPD_KP, SPD_KI, SPD_KD, OUTPUT_IQ_REF_MIN, OUTPUT_IQ_REF_MAX ,INTEGRAL_SPD_MIN, INTEGRAL_SPD_MAX) != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }

  if(adcInit() != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
  else
  {
    adcSetInjectedCallback(motorControlUpdate);
  }

  if(pwmInit() != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
  if(focInit() != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }

#if _USE_HALL_SENSOR
  if(hallInit() != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
#elif _USE_ENCODER
  if(encoderInit() != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
#endif
  if (ret != true)
  {
    motorSetFault(MOTOR_FAULT_INIT_FAIL);
  }

  return ret;
}

void motorStart(void)
{
  if(motor_state != MOTOR_STATE_IDLE)
  {
    return;
  }

  if(adcUpdateRegular() != true)
  {
    motorSetFault(MOTOR_FAULT_ADC_REGULAR_FAIL);
    return;
  }

  motor_vbus = adcGetVbusVoltage();
#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP) && (OPEN_LOOP_DIAG_ENABLE == 1U)
  if (phase_map_diag_selected)
  {
    phase_map_diag.vbus_at_start = motor_vbus;
    phase_map_diag.hall_start = motorOpenLoopDiagReadPins();
  }
#endif
  motor_speed_cmd_raw = adcGetSpeedRaw();
  motor_temp_raw = adcGetTempRaw();

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP) && (OPEN_LOOP_DIAG_ENABLE == 1U)
  // This deliberately low-voltage diagnostic is NOT approved for 24 V VBUS.
  if (motor_vbus > 16.0f)
  {
    motorSetFault(MOTOR_FAULT_INIT_FAIL);
    return;
  }
#endif


  if(motor_vbus < MOTOR_VBUS_MIN)
  {
    motorSetFault(MOTOR_FAULT_VBUS_LOW);
    return;
  }

  pidReset(&pi_id);
  pidReset(&pi_iq);
  pidReset(&pi_spd);

  speed_loop_divider = 0;

  speed_w_target = 0.0f;
  speed_w_ref = 0.0f;
  speed_w_meas = 0.0f;

  current_id_ref = 0.0f;
  current_iq_ref = 0.0f;

  adcInjectedStart();
  pwmStart();

  if(adcCalibrateCurrentOffset() != true)
  {
    motorSetFault(MOTOR_FAULT_ADC_OFFSET_FAIL);
    return;
  }


#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT

  motor_state = MOTOR_STATE_CURRENT_LOOP;

#elif MOTOR_CONTROL_MODE == MOTOR_CONTROL_SPEED

  motor_state = MOTOR_STATE_SPEED_LOOP;

#elif MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP

#if OPEN_LOOP_DIAG_ENABLE == 1U
  open_loop_diag = (motor_ol_diag_t){0};
  open_loop_diag.command_direction = open_loop_diag_requested_direction;
  open_loop_diag.fixed_only = open_loop_diag_requested_fixed_only;
  open_loop_diag.absolute_start = open_loop_diag_requested_absolute_start;
  open_loop_diag_prev_raw = motorOpenLoopDiagReadPins();
  open_loop_diag_prev_cached = hallGetState();
  open_loop_diag_mismatch_samples = 0U;
  open_loop_diag_last_confirmed_sector = hallGetSectorIndex();
  open_loop_diag_confirmed_direction = 0;
  open_loop_diag.irq_at_start = hallGetIrqCount();
  open_loop_diag.transition_at_start = hallGetTransitionCount();
  const int8_t start_sector = hallGetSectorIndex();
  if (!hallIsValid() || start_sector < 0 || start_sector > 5 ||
      open_loop_diag_prev_raw != open_loop_diag_prev_cached)
  {
    motorSetFault(MOTOR_FAULT_HALL_INVALID);
    return;
  }
  open_loop_diag.start_hall = open_loop_diag_prev_raw;
  // For T, the virtual current frame is pinned to absolute stator 0 rad.
  // Hall is checked/logged as a sensor ONLY, never used to set theta_cmd.
  // Other S/R/F/N diagnostics retain their original starting Hall estimate.
  if (open_loop_diag.absolute_start)
    open_loop_diag.theta_fixed = 0.0f;
  else if (open_loop_diag.fixed_only == OL_DIAG_MODE_FIXED_ID_0)
    open_loop_diag.theta_fixed = 0.0f;
  else if (open_loop_diag.fixed_only == OL_DIAG_MODE_FIXED_ID_90)
    open_loop_diag.theta_fixed = PI * 0.5f;
  else
    open_loop_diag.theta_fixed = wrapFloat(
        HALL_ELEC_OFFSET - (float)start_sector * HALL_SECTOR_ANGLE_E,
        0.0f, 2.0f * PI);
  open_loop_theta_e = open_loop_diag.theta_fixed;
  open_loop_speed_e = 0.0f;
  open_loop_vd = 0.0f;
  open_loop_vq = 0.0f;
  open_loop_align_count = 0U;
  open_loop_alignment_done = false;
#else

#if _USE_HALL_OFFSET_CALIBRATION

  open_loop_theta_e = 0.0f;
  open_loop_speed_e = 0.0f;

  open_loop_vd = HALL_CAL_ALIGN_VD;
  open_loop_vq = 0.0f;

  open_loop_align_count = 0U;
  open_loop_alignment_done = false;

  hall_cal_event_pending = false;
  hall_cal_event_sector = -1;
  hall_cal_event_direction = 0;
  hall_cal_event_theta_e = 0.0f;

  hall_cal_prev_sector = hallGetSectorIndex();

#else

  open_loop_theta_e = OPEN_LOOP_ALIGN_THETA_E;
  open_loop_speed_e = 0.0f;

  open_loop_vd = 0.0f;
  open_loop_vq = OPEN_LOOP_ALIGN_VQ;

  open_loop_align_count = 0U;
  open_loop_alignment_done = false;

#endif
#endif  // OPEN_LOOP_DIAG_ENABLE

  motor_state = MOTOR_STATE_OPEN_LOOP;

#endif

  //pwmEnableOutput();
}

/* 테스트 함수들 -> Diag... */
#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)

void motorCurrentDiagStart(void)
{
  uint32_t primask;


  /*
   * [수정]
   * Current loop 시작 전 PWM을 neutral 상태로 준비.
   *
   * 첫 ADC ISR부터 P-only current loop가
   * 새로운 duty를 계산한다.
   */
  motor_duty.u = 0.5f;
  motor_duty.v = 0.5f;
  motor_duty.w = 0.5f;

  pwmSetDuty(motor_duty.u, motor_duty.v, motor_duty.w);

  pidReset(&pi_id);
  pidReset(&pi_iq);

#if (CURRENT_ROTATION_TEST_ENABLE == 1U) && (CURRENT_ROTATION_ANGLE_SWEEP_ENABLE == 1U)

  current_rotation_test_theta_e = hallGetElectricalAngle();

#endif
  /*
   * 30 ms test counter reset
   */
  current_diag_sample_count = 0U;
  current_diag_done = false;
  current_startup_count = 0U;
  current_trip_startup_sample = 0U;

  current_diag_id_sum = 0.0f;
  current_diag_iq_sum = 0.0f;

  // [추가] 마지막 100 samples 평균용 초기화
  current_diag_tail_count = 0U;

  current_diag_tail_id_sum = 0.0f;
  current_diag_tail_iq_sum = 0.0f;

  current_diag_tail_vd_sum = 0.0f;
  current_diag_tail_vq_sum = 0.0f;

  current_diag_i_min.a = 0.0f;
  current_diag_i_min.b = 0.0f;
  current_diag_i_min.c = 0.0f;

  current_diag_i_max.a = 0.0f;
  current_diag_i_max.b = 0.0f;
  current_diag_i_max.c = 0.0f;

  current_snapshot_50ms.a = 0.0f;
  current_snapshot_50ms.b = 0.0f;
  current_snapshot_50ms.c = 0.0f;

  current_avg_sum.a = 0.0f;
  current_avg_sum.b = 0.0f;
  current_avg_sum.c = 0.0f;
  current_avg_count = 0U;

  // [L TEST]
  current_l_test_count = 0U;

  for (uint32_t i = 0U;
       i < CURRENT_L_TEST_SAMPLE_COUNT;
       i++)
  {
    current_l_test_id[i] = 0.0f;
    current_l_test_iq[i] = 0.0f;
    current_l_test_vd[i] = 0.0f;
    current_l_test_vq[i] = 0.0f;
    current_l_test_theta[i] = 0.0f;
    current_l_test_hall_state[i] = 0U;
    current_l_test_hall_sector[i] = -1;
    current_l_test_hall_valid[i] = 0U;
  }

#if CURRENT_ADC_NOISE_TEST_ENABLE

  current_noise_count = 0U;

  for (uint32_t i = 0U;
       i < CURRENT_NOISE_TEST_SAMPLE_COUNT;
       i++)
  {
    current_noise_raw[i].a = 0U;
    current_noise_raw[i].b = 0U;
    current_noise_raw[i].c = 0U;

    current_noise_amp[i].a = 0.0f;
    current_noise_amp[i].b = 0.0f;
    current_noise_amp[i].c = 0.0f;
  }

#endif

  current_trip_raw_a = 0U;
  current_trip_raw_b = 0U;
  current_trip_raw_c = 0U;

  current_trip_ia = 0.0f;
  current_trip_ib = 0.0f;
  current_trip_ic = 0.0f;

  // 진단 초기화 완료 후 짧은 구간만 인터럽트 차단
  primask = __get_PRIMASK();
  __disable_irq();

  diag_pwm_armed = false;
  diag_moe_after_off = 2U;
  current_diag_active = true;

  // CPU Cycle Counter 활성화
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // 진단 변수 초기화
  diag_first_isr_seen = false;

  diag_cnt_before_output = TIM1->CNT;
  diag_cnt_first_isr = 0U;

  // PWM Enable 직전 ADC 상태
  diag_jeoc_before_output = ((ADC1->SR & ADC_SR_JEOC) != 0U) ? 1U : 0U;

  diag_irq_pending_before_output = (uint32_t)NVIC_GetPendingIRQ(ADC_IRQn);

  // PWM 출력 전환 직전 ADC Injected 레지스터 확인
  diag_raw_before_a = (uint16_t)ADC1->JDR1;
  diag_raw_before_b = (uint16_t)ADC1->JDR2;
  diag_raw_before_c = (uint16_t)ADC1->JDR3;

  diag_first_isr_cycles = 0U;
  diag_output_cycle = DWT->CYCCNT;

  /*
   * 실제 출력 시작
   */

  pwmDisableOutput();


  if (primask == 0U)
  {
    __enable_irq();
  }
}

void motorCurrentDiagGetTiming(motor_diag_timing_t *timing)
{
  if (timing == NULL)
  {
    return;
  }

  timing->cnt_before_output = diag_cnt_before_output;
  timing->cnt_first_isr = diag_cnt_first_isr;

  timing->raw_before_a = diag_raw_before_a;
  timing->raw_before_b = diag_raw_before_b;
  timing->raw_before_c = diag_raw_before_c;

  timing->jeoc_before_output = diag_jeoc_before_output;
  timing->adc_irq_pending_before_output =
      diag_irq_pending_before_output;

  timing->cycles_to_first_isr = diag_first_isr_cycles;
  timing->first_isr_seen = diag_first_isr_seen;
}
uint32_t motorCurrentDiagGetMoeAfterOff(void)
{
  return diag_moe_after_off;
}

bool motorCurrentDiagIsDone(void)
{
  return current_diag_done;
}

uint32_t motorCurrentDiagGetSampleCount(void)
{
  return current_diag_sample_count;
}
uint32_t motorCurrentDiagGetStartupTripSample(void)
{
  return current_trip_startup_sample;
}
void motorCurrentDiagGetCurrentMinMax(motor_abc_f_t *i_min, motor_abc_f_t *i_max)
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

uint32_t motorCurrentLTestGetCount(void)
{
  return current_l_test_count;
}


bool motorCurrentLTestGetSample(uint32_t index, float *id, float *iq, float *vd, float *vq, float *theta_e)
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

#if CURRENT_ROTATION_TEST_ENABLE

bool motorCurrentLTestGetHallSample(uint32_t index, uint8_t *hall_state, int8_t *hall_sector, uint8_t *hall_valid)
{
  if (index >= current_l_test_count)
  {
    return false;
  }

  if (hall_state != NULL)
    *hall_state = current_l_test_hall_state[index];

  if (hall_sector != NULL)
    *hall_sector = current_l_test_hall_sector[index];

  if (hall_valid != NULL)
    *hall_valid = current_l_test_hall_valid[index];

  return true;
}

#endif

uint32_t motorCurrentNoiseGetCount(void)
{
#if CURRENT_ADC_NOISE_TEST_ENABLE

  return current_noise_count;

#else

  return 0U;

#endif
}

bool motorCurrentNoiseGetSample(uint32_t index, motor_abc_u16_t *raw, motor_abc_f_t *current)
{
#if CURRENT_ADC_NOISE_TEST_ENABLE

  if (index >= current_noise_count)
  {
    return false;
  }

  if (raw != NULL)
  {
    raw->a = current_noise_raw[index].a;
    raw->b = current_noise_raw[index].b;
    raw->c = current_noise_raw[index].c;
  }

  if (current != NULL)
  {
    current->a = current_noise_amp[index].a;
    current->b = current_noise_amp[index].b;
    current->c = current_noise_amp[index].c;
  }

  return true;

#else

  (void)index;
  (void)raw;
  (void)current;

  return false;

#endif
}

void motorCurrentDiagGetTripSnapshot(motor_abc_u16_t *raw,
                                     motor_abc_f_t *current)
{
  if (raw != NULL)
  {
    raw->a = current_trip_raw_a;
    raw->b = current_trip_raw_b;
    raw->c = current_trip_raw_c;
  }

  if (current != NULL)
  {
    current->a = current_trip_ia;
    current->b = current_trip_ib;
    current->c = current_trip_ic;
  }
}

#endif

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)
void motorCurrentDiagGetSnapshot50ms(motor_abc_f_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    snapshot->a = current_snapshot_50ms.a;
    snapshot->b = current_snapshot_50ms.b;
    snapshot->c = current_snapshot_50ms.c;
}
#endif

/* 테스트 함수들 -> Diag... */

void motorStop(void)
{
  pwmDisableOutput();
#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP) && (OPEN_LOOP_DIAG_ENABLE == 1U)
  if (six_step.selected) sixStepGatePinsOff();
#endif
  adcInjectedStop();
  pwmStop();

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)

  // [수정]
  current_diag_active = false;

#endif

  pidReset(&pi_id);
  pidReset(&pi_iq);
  pidReset(&pi_spd);

  speed_loop_divider = 0;

  speed_w_target = 0.0f;
  speed_w_ref = 0.0f;
  speed_w_meas = 0.0f;

  current_id_ref = 0.0f;
  current_iq_ref = 0.0f;

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP

  open_loop_theta_e = 0.0f;
  open_loop_speed_e = 0.0f;
  open_loop_vd = 0.0f;
  open_loop_vq = 0.0f;

  open_loop_align_count = 0U;
  open_loop_alignment_done = false;

#if _USE_HALL_OFFSET_CALIBRATION

  hall_cal_event_pending = false;
  hall_cal_event_sector = -1;
  hall_cal_event_direction = 0;
  hall_cal_event_theta_e = 0.0f;
  hall_cal_prev_sector = -1;

#endif

#endif

  motor_state = MOTOR_STATE_IDLE;
}
void motorLowSpeedTask(void)
{

  if (motor_state == MOTOR_STATE_FAULT)
  {
    return;
  }

#if _USE_HALL_SENSOR
  hallUpdateTimeout();
#endif

  if (pwmIsBreakFault() == true)
  {
    motorSetFault(MOTOR_FAULT_BKIN);
    return;
  }

  if ((motor_state == MOTOR_STATE_CURRENT_LOOP) ||
      (motor_state == MOTOR_STATE_SPEED_LOOP)   ||
      (motor_state == MOTOR_STATE_OPEN_LOOP))
  {
    return;
  }

  if (adcUpdateRegular() != true)
  {
    motorSetFault(MOTOR_FAULT_ADC_REGULAR_FAIL);
    return;
  }

  motor_vbus = adcGetVbusVoltage();
  motor_speed_cmd_raw = adcGetSpeedRaw();
  motor_temp_raw = adcGetTempRaw();

  if (motor_vbus < MOTOR_VBUS_MIN)
  {
    motorSetFault(MOTOR_FAULT_VBUS_LOW);
    return;
  }
}

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)


#if CURRENT_ROTATION_TEST_ENABLE

static bool motorRotationHallReady(void)
{
  uint8_t state = hallGetState();

  if ((hallIsValid() == false) ||
      (hallGetSectorIndex() < 0) ||
      (state == 0U) ||
      (state == 7U))
  {
    return false;
  }

  return true;
}

static void motorRotationDiagAbort(motor_fault_t fault)
{
  // ISR에서도 즉시 출력 차단
  pwmDisableOutput();

  motor_fault = fault;
  motor_state = MOTOR_STATE_FAULT;

  current_diag_active = false;
  current_diag_done = false;
}

#endif


static void motorNeutralPwmDiag(void)
{
  float theta_e;


  /*
   * 30 ms test가 시작되지 않았거나
   * 이미 완료됐다면 아무것도 하지 않는다.
   */
  if (current_diag_active == false)
  {
    return;
  }

#if (CURRENT_ADC_PWM_OFF_TEST_ENABLE == 0U) && \
    (CURRENT_ADC_SEQUENCE_TEST_ENABLE == 0U)

  // 첫 번째 ADC 완료 콜백에서 PWM 출력 활성화
  if (diag_pwm_armed == false)
  {
    // 실제 MOE 활성화 직전의 TIM1 상태 기록
    diag_cnt_before_output = TIM1->CNT;

    // 출력 활성화 직전 ADC Raw 기록
    diag_raw_before_a = (uint16_t)ADC1->JDR1;
    diag_raw_before_b = (uint16_t)ADC1->JDR2;
    diag_raw_before_c = (uint16_t)ADC1->JDR3;

    diag_output_cycle = DWT->CYCCNT;

    #if CURRENT_ROTATION_TEST_ENABLE

    // 1. Break Fault 확인
    if ((pwmIsBreakFault() == true) ||
        ((TIM1->SR & TIM_SR_BIF) != 0U))
    {
      motorRotationDiagAbort(MOTOR_FAULT_BKIN);
      return;
    }

    // 2. Hall 입력 유효성 확인
    if (motorRotationHallReady() == false)
    {
      motorRotationDiagAbort(MOTOR_FAULT_HALL_INVALID);
      return;
    }

    #endif


    // Neutral PWM ON 시험에서도 BKIN 상태 확인
    if ((pwmIsBreakFault() == true) ||
        ((TIM1->SR & TIM_SR_BIF) != 0U))
    {
        pwmDisableOutput();

        motor_fault = MOTOR_FAULT_BKIN;
        motor_state = MOTOR_STATE_FAULT;

        current_diag_active = false;
        current_diag_done = false;

        return;
    }

    // 정상 상태일 때만 PWM 활성화
    diag_pwm_armed = true;

    pwmEnableOutput();

    return;

  }

#endif

#if (CURRENT_ADC_PWM_OFF_TEST_ENABLE == 0U) && \
    (CURRENT_ADC_SEQUENCE_TEST_ENABLE == 0U)

  bool break_fault =
      (pwmIsBreakFault() == true) ||
      ((TIM1->SR & TIM_SR_BIF) != 0U);

  bool moe_disabled =
      ((TIM1->BDTR & TIM_BDTR_MOE) == 0U);

  if (break_fault || moe_disabled)
  {
      pwmDisableOutput();

      motor_fault = break_fault ?
          MOTOR_FAULT_BKIN : MOTOR_FAULT_INIT_FAIL;

      motor_state = MOTOR_STATE_FAULT;

      current_diag_active = false;
      current_diag_done = false;

      return;
  }

#endif

  // 첫 ADC ISR의 TIM1 CNT와 경과 Cycle 기록
  if (diag_first_isr_seen == false)
  {
    diag_cnt_first_isr = TIM1->CNT;

    diag_first_isr_cycles =
        DWT->CYCCNT - diag_output_cycle;

    diag_first_isr_seen = true;
  }

  #if CURRENT_ROTATION_TEST_ENABLE

  // 구동 도중 Break 발생 확인
  if ((pwmIsBreakFault() == true) ||
      ((TIM1->SR & TIM_SR_BIF) != 0U) ||
      ((TIM1->BDTR & TIM_BDTR_MOE) == 0U))
  {
    motorRotationDiagAbort(MOTOR_FAULT_BKIN);
    return;
  }

  // 구동 도중 Hall 신호 이상 확인
  if (motorRotationHallReady() == false)
  {
    motorRotationDiagAbort(MOTOR_FAULT_HALL_INVALID);
    return;
  }

  #endif

  /*
   * [수정]
   * 실제 Current FOC에서 사용할 rotor electrical angle
   */
#if _USE_HALL_SENSOR

  theta_e = hallGetElectricalAngle();

#elif _USE_ENCODER

  theta_e = encoderGetElectricalAngle();

#else

#error "HALL SENSOR / ENCODER ERROR"

#endif

#if (CURRENT_ROTATION_TEST_ENABLE == 1U) && \
    (CURRENT_ROTATION_ANGLE_SWEEP_ENABLE == 1U)

  if ((current_diag_sample_count >= CURRENT_ROTATION_START_SAMPLE) &&
      (current_diag_sample_count < CURRENT_ROTATION_END_SAMPLE))
  {
    current_rotation_test_theta_e +=
        CURRENT_ROTATION_SWEEP_SPEED_E * CUR_DT;

    current_rotation_test_theta_e =
        wrapFloat(current_rotation_test_theta_e,
                  0.0f, 2.0f * PI);
  }

  theta_e = current_rotation_test_theta_e;

#endif

#if CURRENT_ADC_NOISE_TEST_ENABLE

  motor_abc_f_t i_abc;
  motor_abc_u16_t raw;

  /*
   * [ADC NOISE TEST]
   *
   * PI / Clarke / Park / FOC를 사용하지 않는다.
   * PWM은 motorCurrentDiagStart()에서 설정한
   * U = V = W = 50% 상태를 그대로 유지한다.
   */

  adcGetPhaseCurrent(&i_abc);
  adcGetCurrentRaw(&raw);

  motor_monitor.ia = i_abc.a;
  motor_monitor.ib = i_abc.b;
  motor_monitor.ic = i_abc.c;

  motor_monitor.id_ref = 0.0f;
  motor_monitor.iq_ref = 0.0f;

  motor_monitor.id_meas = 0.0f;
  motor_monitor.iq_meas = 0.0f;

  motor_monitor.vd_cmd = 0.0f;
  motor_monitor.vq_cmd = 0.0f;

  motor_monitor.duty_u = 0.5f;
  motor_monitor.duty_v = 0.5f;
  motor_monitor.duty_w = 0.5f;

  float max_phase_current = fabsf(i_abc.a);

  if (fabsf(i_abc.b) > max_phase_current)
  {
    max_phase_current = fabsf(i_abc.b);
  }

  if (fabsf(i_abc.c) > max_phase_current)
  {
    max_phase_current = fabsf(i_abc.c);
  }

  if (max_phase_current > CURRENT_TEST_OC_LIMIT_A)
  {
      // Trip 시점 데이터 저장
      current_trip_raw_a = raw.a;
      current_trip_raw_b = raw.b;
      current_trip_raw_c = raw.c;

      current_trip_ia = i_abc.a;
      current_trip_ib = i_abc.b;
      current_trip_ic = i_abc.c;

      pwmDisableOutput();

      motor_fault = MOTOR_FAULT_SW_OVERCURRENT;
      motor_state = MOTOR_STATE_FAULT;

      current_diag_active = false;
      current_diag_done = false;

      return;
  }

  if (current_noise_count <
      CURRENT_NOISE_TEST_SAMPLE_COUNT)
  {
    current_noise_raw[current_noise_count] = raw;
    current_noise_amp[current_noise_count] = i_abc;

    current_noise_count++;
  }


#else


  /*
   * PWM enable 직후 current-sense settling 구간
   *
   * 4 ISR = 4 x 50 us = 200 us
   * 이 동안 PI를 실행하지 않고
   * U/V/W = 50% neutral PWM만 유지한다.
   */
  if (current_startup_count < CURRENT_STARTUP_SETTLE_SAMPLES)
  {
    motor_abc_f_t i_abc;

    /*
     * ADC/current amplifier 값은 읽어주되
     * 이 startup sample은 PI 및 진단 sample에 포함하지 않는다.
     */
    adcGetPhaseCurrent(&i_abc);

    motor_monitor.ia = i_abc.a;
    motor_monitor.ib = i_abc.b;
    motor_monitor.ic = i_abc.c;

    // Startup 200us 구간에서도 SW 과전류 검사
    if ((fabsf(i_abc.a) > CURRENT_TEST_OC_LIMIT_A) ||
        (fabsf(i_abc.b) > CURRENT_TEST_OC_LIMIT_A) ||
        (fabsf(i_abc.c) > CURRENT_TEST_OC_LIMIT_A))
    {
      motor_abc_u16_t raw;
      adcGetCurrentRaw(&raw);

      // Fault 발생한 Startup 샘플 번호 (1~4)
      current_trip_startup_sample = current_startup_count + 1U;

      // Trip 시점의 ADC Raw 저장
      current_trip_raw_a = raw.a;
      current_trip_raw_b = raw.b;
      current_trip_raw_c = raw.c;

      // Trip 시점의 환산 전류 저장
      current_trip_ia = i_abc.a;
      current_trip_ib = i_abc.b;
      current_trip_ic = i_abc.c;

      // PWM 즉시 차단
      pwmDisableOutput();

      motor_fault = MOTOR_FAULT_SW_OVERCURRENT;
      motor_state = MOTOR_STATE_FAULT;

      current_diag_active = false;
      current_diag_done = false;

      return;
    }

    motor_monitor.id_ref  = 0.0f;
    motor_monitor.id_meas = 0.0f;
    motor_monitor.iq_ref  = 0.0f;
    motor_monitor.iq_meas = 0.0f;

    motor_monitor.vd_cmd = 0.0f;
    motor_monitor.vq_cmd = 0.0f;

    motor_monitor.duty_u = 0.5f;
    motor_monitor.duty_v = 0.5f;
    motor_monitor.duty_w = 0.5f;

    pwmSetDuty(0.5f, 0.5f, 0.5f);

    current_startup_count++;

    /*
     * 마지막 settling sample 이후
     * 실제 current step이 항상 zero integral에서 시작하도록 reset
     */
    if (current_startup_count >= CURRENT_STARTUP_SETTLE_SAMPLES)
    {
      pidReset(&pi_id);
      pidReset(&pi_iq);
    }

    return;
  }

  /*
   * Settling 종료 후부터 정상 Current PI
   */
  motorCurrentLoop(current_id_ref,
                   current_iq_ref,
                   theta_e);

#endif

  if (motor_state == MOTOR_STATE_FAULT)
  {
    current_diag_active = false;
    current_diag_done = false;

    return;
  }

#if CURRENT_ROTATION_TEST_ENABLE

if (current_diag_sample_count == 1000U)
{
    current_snapshot_50ms.a = motor_monitor.ia;
    current_snapshot_50ms.b = motor_monitor.ib;
    current_snapshot_50ms.c = motor_monitor.ic;
}

#endif

#if !CURRENT_ADC_NOISE_TEST_ENABLE

#if CURRENT_ROTATION_TEST_ENABLE

  // 20 ISR마다 1회 저장 = 1ms 간격
  if (((current_diag_sample_count %
        CURRENT_ROTATION_LOG_DIVIDER) == 0U) &&
      (current_l_test_count < CURRENT_L_TEST_SAMPLE_COUNT))

#else

  // 기존 시험은 매 ISR 저장
  if (current_l_test_count < CURRENT_L_TEST_SAMPLE_COUNT)

#endif
  {
    current_l_test_id[current_l_test_count] = motor_monitor.id_meas;
    current_l_test_iq[current_l_test_count] = motor_monitor.iq_meas;
    current_l_test_vd[current_l_test_count] = motor_monitor.vd_cmd;
    current_l_test_vq[current_l_test_count] = motor_monitor.vq_cmd;
    current_l_test_theta[current_l_test_count] = motor_monitor.theta_e;

    #if CURRENT_ROTATION_TEST_ENABLE
    current_l_test_hall_state[current_l_test_count] = hallGetState();
    current_l_test_hall_sector[current_l_test_count] = hallGetSectorIndex();
    current_l_test_hall_valid[current_l_test_count] = hallIsValid() ? 1U : 0U;
    #endif

    current_l_test_count++;
  }

#endif  // !CURRENT_ADC_NOISE_TEST_ENABLE

current_diag_sample_count++;

#if (CURRENT_SW_BREAK_TEST_ENABLE == 1U) && \
    (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)

  if (current_diag_sample_count ==
      CURRENT_SW_BREAK_TRIGGER_SAMPLE)
  {
    if ((TIM1->BDTR & TIM_BDTR_MOE) != 0U)
    {
      // TIM1에 소프트웨어 Break 이벤트 발생
      TIM1->EGR = TIM_EGR_BG;
    }

    return;
  }

#endif

#if (CURRENT_IQ_STEP_TEST_ENABLE == 1U)

  // 100 samples = 5ms
  // 0mA -> +100mA
  if (current_diag_sample_count ==
      CURRENT_IQ_STEP_STAGE_SAMPLES)
  {
    motorSetCurrentReference(
        0.0f,
        CURRENT_IQ_STEP_REF_A);
  }

  // 200 samples = 10ms
  // +100mA -> 0mA
  else if (current_diag_sample_count ==
           (CURRENT_IQ_STEP_STAGE_SAMPLES * 2U))
  {
    motorSetCurrentReference(0.0f, 0.0f);
  }

#endif


#if (CURRENT_ROTATION_TEST_ENABLE == 1U)

// 10ms: Iq 0A -> +0.1A
if (current_diag_sample_count ==
    CURRENT_ROTATION_START_SAMPLE)
  {
    motorSetCurrentReference(
        0.0f,
        CURRENT_ROTATION_REF_A);
  }

// 90ms: Iq +0.1A -> 0A
else if (current_diag_sample_count ==
         CURRENT_ROTATION_END_SAMPLE)
  {
    motorSetCurrentReference(0.0f, 0.0f);
  }

#endif


// ====================================================
// OFF -> ON -> OFF ADC Sequence Test
//
// 0 ~ 99     : PWM OFF
// 100 ~ 299  : PWM ON, Neutral 50%
// 300 ~ 399  : PWM OFF
//
// ADC 샘플 저장 후 PWM 상태 전환.
// Offset 재보정 없음.
// ====================================================
#if (CURRENT_ADC_SEQUENCE_TEST_ENABLE == 1U) && \
    (CURRENT_ADC_NOISE_TEST_ENABLE == 1U)

  if (current_diag_sample_count ==
      CURRENT_SEQUENCE_OFF1_SAMPLES)
  {
    // 100번째 OFF 샘플 저장 완료
    // 다음 샘플부터 Neutral PWM ON
    diag_cnt_before_output = TIM1->CNT;
    diag_raw_before_a = (uint16_t)ADC1->JDR1;
    diag_raw_before_b = (uint16_t)ADC1->JDR2;
    diag_raw_before_c = (uint16_t)ADC1->JDR3;

    diag_output_cycle = DWT->CYCCNT;
    diag_first_isr_seen = false;

    diag_pwm_armed = true;

    // 기존 BKIN 보호 상태를 확인한 경우에만 실행
    if ((motor_state == MOTOR_STATE_CURRENT_LOOP) &&
        (motor_fault == MOTOR_FAULT_NONE) &&
        (pwmIsBreakFault() == false) &&
        ((TIM1->SR & TIM_SR_BIF) == 0U))
    {
      pwmEnableOutput();
    }

    else
    {
      pwmDisableOutput();

      current_diag_active = false;
      current_diag_done = false;

      // 기존 Break 보호 상태라면 Fault로 전환
      if (pwmIsBreakFault() ||
          ((TIM1->SR & TIM_SR_BIF) != 0U))
      {
        motor_fault = MOTOR_FAULT_BKIN;
      }
      else
      {
        // 예상하지 못한 출력 활성화 조건 실패
        motor_fault = MOTOR_FAULT_INIT_FAIL;
      }

      motor_state = MOTOR_STATE_FAULT;
      return;
    }

  }

  if (current_diag_sample_count ==
      (CURRENT_SEQUENCE_OFF1_SAMPLES +
       CURRENT_SEQUENCE_ON_SAMPLES))
  {
    pwmDisableOutput();

    // PWM OFF 명령 직후 MOE 상태 기록
    diag_moe_after_off =
        ((TIM1->BDTR & TIM_BDTR_MOE) != 0U) ? 1U : 0U;
  }

#endif

  current_diag_id_sum += motor_monitor.id_meas;
  current_diag_iq_sum += motor_monitor.iq_meas;

  if ((current_diag_sample_count + CURRENT_DIAG_TAIL_SAMPLES) >
      CURRENT_LOOP_TEST_SAMPLE_COUNT)
  {
    current_diag_tail_count++;

    current_diag_tail_id_sum += motor_monitor.id_meas;
    current_diag_tail_iq_sum += motor_monitor.iq_meas;

    current_diag_tail_vd_sum += motor_monitor.vd_cmd;
    current_diag_tail_vq_sum += motor_monitor.vq_cmd;
  }

  if (current_diag_sample_count == 1U)
  {
    current_diag_i_min.a = motor_monitor.ia;
    current_diag_i_min.b = motor_monitor.ib;
    current_diag_i_min.c = motor_monitor.ic;

    current_diag_i_max.a = motor_monitor.ia;
    current_diag_i_max.b = motor_monitor.ib;
    current_diag_i_max.c = motor_monitor.ic;
  }
  else
  {
    if (motor_monitor.ia < current_diag_i_min.a)
    {
      current_diag_i_min.a = motor_monitor.ia;
    }

    if (motor_monitor.ia > current_diag_i_max.a)
    {
      current_diag_i_max.a = motor_monitor.ia;
    }


    if (motor_monitor.ib < current_diag_i_min.b)
    {
      current_diag_i_min.b = motor_monitor.ib;
    }

    if (motor_monitor.ib > current_diag_i_max.b)
    {
      current_diag_i_max.b = motor_monitor.ib;
    }


    if (motor_monitor.ic < current_diag_i_min.c)
    {
      current_diag_i_min.c = motor_monitor.ic;
    }

    if (motor_monitor.ic > current_diag_i_max.c)
    {
      current_diag_i_max.c = motor_monitor.ic;
    }
  }
  #if CURRENT_ROTATION_TEST_ENABLE

  // 30ms ~ 100ms 구간의 3상 전류 누적
  if ((current_diag_sample_count >= 600U) &&
      (current_diag_sample_count < 2000U))
  {
    current_avg_sum.a += motor_monitor.ia;
    current_avg_sum.b += motor_monitor.ib;
    current_avg_sum.c += motor_monitor.ic;

    current_avg_count++;
  }

  #endif

  /*
   * motorCurrentLoop()에서 0.8 A trip이 발생했다면
   * 이미 MOE는 OFF 되어 있다.
   */

  /*
   * [안전]
   * 600 samples = 30 ms
   *
   * main loop가 아니라 ADC ISR에서 직접 MOE OFF.
   */
  if (current_diag_sample_count >=
      CURRENT_LOOP_TEST_SAMPLE_COUNT)
  {
    pwmDisableOutput();

    current_diag_active = false;
    current_diag_done = true;

    return;
  }
}

#endif

static void motorCurrentLoop(float id_ref, float iq_ref, float theta_e)
{
  motor_abc_f_t i_abc;
  motor_alphabeta_t i_ab;
  motor_dq_t i_dq;
  motor_dq_t v_dq;
  motor_alphabeta_t v_ab;

  adcGetPhaseCurrent(&i_abc);

  motor_monitor.ia = i_abc.a;
  motor_monitor.ib = i_abc.b;
  motor_monitor.ic = i_abc.c;

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP) && (OPEN_LOOP_DIAG_ENABLE == 1U)
  const float current_loop_oc_limit =
      hall_foc.selected ?
        (hall_foc.forced_selected ? JFOC_OC_LIMIT_A : HFOC_OC_LIMIT_A) :
      ((open_loop_diag.fixed_only >= OL_DIAG_MODE_FIXED_ID_0) ?
       OL_FIXED_ID_OC_LIMIT_A : OPEN_LOOP_DIAG_OC_LIMIT_A);
#else
  const float current_loop_oc_limit = CURRENT_TEST_OC_LIMIT_A;
#endif
  if ((fabsf(i_abc.a) > current_loop_oc_limit) ||
      (fabsf(i_abc.b) > current_loop_oc_limit) ||
      (fabsf(i_abc.c) > current_loop_oc_limit))
  {
#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)
    motor_abc_u16_t raw;

    adcGetCurrentRaw(&raw);

    current_trip_raw_a = raw.a;
    current_trip_raw_b = raw.b;
    current_trip_raw_c = raw.c;

    current_trip_ia = i_abc.a;
    current_trip_ib = i_abc.b;
    current_trip_ic = i_abc.c;
#endif
    pwmDisableOutput();

    motor_fault = MOTOR_FAULT_SW_OVERCURRENT;
    motor_state = MOTOR_STATE_FAULT;

    return;
  }


  focClarke(i_abc.a, i_abc.b, i_abc.c, &i_ab);
  focPark(i_ab.alpha, i_ab.beta, theta_e, &i_dq);

  motor_monitor.id_ref = id_ref;
  motor_monitor.id_meas = i_dq.d;
  motor_monitor.iq_ref = iq_ref;
  motor_monitor.iq_meas = i_dq.q;

  motor_monitor.theta_e   = theta_e;


  v_dq.d = piController(&pi_id, id_ref, i_dq.d, CUR_DT);
  v_dq.q = piController(&pi_iq, iq_ref, i_dq.q, CUR_DT);


#if _USE_FOC_SPWM
  focSetVoltageLimit(&v_dq, MOTOR_VLIMIT_SPWM);
#elif _USE_FOC_SVPWM
  focSetVoltageLimit(&v_dq, MOTOR_VLIMIT_SVPWM);
#endif

  motor_monitor.vd_cmd = v_dq.d;
  motor_monitor.vq_cmd = v_dq.q;

  focInvPark(v_dq.d, v_dq.q, theta_e, &v_ab);

#if _USE_FOC_SPWM
  focGenerateSPWM(v_ab.alpha, v_ab.beta, motor_vbus, &motor_duty);
#elif _USE_FOC_SVPWM
  focGenerateSVPWM(v_ab.alpha, v_ab.beta, motor_vbus, &motor_duty);
#endif

  motor_monitor.duty_u = motor_duty.u;
  motor_monitor.duty_v = motor_duty.v;
  motor_monitor.duty_w = motor_duty.w;

  pwmSetDuty(motor_duty.u, motor_duty.v, motor_duty.w);
}

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_SPEED
static void motorSpeedLoop(void)
{
  float ratio;
  float delta;
  float ramp_step;

  if (motor_state != MOTOR_STATE_SPEED_LOOP)
  {
    return;
  }

#if _USE_HALL_SENSOR
  speed_w_meas = hallGetMechanicalSpeed();
  motor_monitor.speed_meas = speed_w_meas;
#elif _USE_ENCODER
  speed_w_meas = encoderGetMechanicalSpeed();
  motor_monitor.speed_meas = speed_w_meas;
#endif

  if (motor_speed_cmd_raw <= SPEED_CMD_DEADBAND_RAW)
  {
    speed_w_target = 0.0f;
  }
  else
  {
    ratio = (float)(motor_speed_cmd_raw - SPEED_CMD_DEADBAND_RAW) /
            (float)(4095U - SPEED_CMD_DEADBAND_RAW);
    ratio = clampFloat(ratio, 0.0f, 1.0f);

    speed_w_target = ratio * OUTPUT_SPD_REF_MAX;
  }

  delta = speed_w_target - speed_w_ref;

  if (delta > 0.0f)
  {
    ramp_step = SPEED_RAMP_UP * SPD_DT;

    if (delta > ramp_step)
    {
      speed_w_ref += ramp_step;
    }
    else
    {
      speed_w_ref = speed_w_target;
    }
  }
  else if (delta < 0.0f)
  {
    ramp_step = SPEED_RAMP_DOWN * SPD_DT;

    if (delta < -ramp_step)
    {
      speed_w_ref -= ramp_step;
    }
    else
    {
      speed_w_ref = speed_w_target;
    }
  }

  speed_w_ref = clampFloat(speed_w_ref, 0.0f, OUTPUT_SPD_REF_MAX);
  motor_monitor.speed_ref = speed_w_ref;

  current_id_ref = 0.0f;
  current_iq_ref = piController(&pi_spd, speed_w_ref, speed_w_meas, SPD_DT);
}
#endif

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP

static void motorOpenLoop(void)
{
#if OPEN_LOOP_DIAG_ENABLE == 1U
  if (six_step.selected)
    sixStepUpdate();
  else if (hall_foc.selected)
    motorHallFocUpdate();
  else if (phase_map_diag_selected)
    motorPhaseMapDiagUpdate();
  else
    motorOpenLoopDiagnostic();
#else

  motor_abc_f_t i_abc;

#if _USE_HALL_OFFSET_CALIBRATION
  int8_t current_sector;
#endif

  if (motor_state != MOTOR_STATE_OPEN_LOOP)
  {
    return;
  }

  adcGetPhaseCurrent(&i_abc);

  motor_monitor.ia = i_abc.a;
  motor_monitor.ib = i_abc.b;
  motor_monitor.ic = i_abc.c;

  // Open-loop 소프트웨어 과전류 보호
  if ((fabsf(i_abc.a) > CURRENT_TEST_OC_LIMIT_A) ||
      (fabsf(i_abc.b) > CURRENT_TEST_OC_LIMIT_A) ||
      (fabsf(i_abc.c) > CURRENT_TEST_OC_LIMIT_A))
  {
      pwmDisableOutput();

      motor_fault = MOTOR_FAULT_SW_OVERCURRENT;
      motor_state = MOTOR_STATE_FAULT;

      return;
  }

#if _USE_HALL_OFFSET_CALIBRATION

  if (open_loop_alignment_done == false)
  {
    open_loop_theta_e = 0.0f;
    open_loop_speed_e = 0.0f;

    open_loop_vd = HALL_CAL_ALIGN_VD;
    open_loop_vq = 0.0f;

    current_sector = hallGetSectorIndex();

    if (current_sector >= 0)
    {
      hall_cal_prev_sector = current_sector;
    }

    open_loop_align_count++;

    if (open_loop_align_count >= HALL_CAL_ALIGN_COUNT)
    {
      open_loop_alignment_done = true;

      open_loop_speed_e = HALL_CAL_DIRECTION * HALL_CAL_SPEED_E;

      open_loop_vd = HALL_CAL_ROTATE_VD;
      open_loop_vq = 0.0f;

      hall_cal_prev_sector = hallGetSectorIndex();
    }
  }
  else
  {
    open_loop_speed_e =
        HALL_CAL_DIRECTION * HALL_CAL_SPEED_E;

    open_loop_theta_e += open_loop_speed_e * OPEN_DT;

    open_loop_theta_e = wrapFloat(open_loop_theta_e, 0.0f, 2.0f * PI);

    open_loop_vd = HALL_CAL_ROTATE_VD;
    open_loop_vq = 0.0f;

    current_sector = hallGetSectorIndex();

    if (current_sector >= 0)
    {
      if (hall_cal_prev_sector < 0)
      {
        hall_cal_prev_sector = current_sector;
      }
      else if (current_sector != hall_cal_prev_sector)
      {
        int8_t sector_diff;
        int8_t event_direction = 0;
        int8_t expected_direction;

        sector_diff = current_sector - hall_cal_prev_sector;

        if ((sector_diff == -1) || (sector_diff == 5))
        {
          event_direction = 1;
        }
        else if ((sector_diff == 1) || (sector_diff == -5))
        {
          event_direction = -1;
        }
        else
        {
          event_direction = 0;
        }

        expected_direction =
            (HALL_CAL_DIRECTION >= 0.0f) ? 1 : -1;

        if (event_direction == expected_direction)
        {
          hall_cal_event_sector = current_sector;
          hall_cal_event_direction = event_direction;
          hall_cal_event_theta_e = open_loop_theta_e;
          hall_cal_event_pending = true;
        }

        hall_cal_prev_sector = current_sector;
      }
    }
  }

#else

  if (open_loop_alignment_done == false)
  {
    open_loop_theta_e = OPEN_LOOP_ALIGN_THETA_E;
    open_loop_speed_e = 0.0f;

    open_loop_vd = 0.0f;
    open_loop_vq = OPEN_LOOP_ALIGN_VQ;

    open_loop_align_count++;

    if (open_loop_align_count >= OPEN_LOOP_ALIGN_COUNT)
    {
      open_loop_alignment_done = true;

      open_loop_speed_e = OPEN_LOOP_START_SPEED_E;

      open_loop_vd = 0.0f;
      open_loop_vq = OPEN_LOOP_TARGET_VQ;
    }
  }
  else
  {
    open_loop_speed_e += OPEN_LOOP_ACCEL_E * OPEN_DT;

    open_loop_speed_e = clampFloat(open_loop_speed_e, OPEN_LOOP_START_SPEED_E, OPEN_LOOP_TARGET_SPEED);

    open_loop_theta_e += open_loop_speed_e * OPEN_DT;

    open_loop_theta_e = wrapFloat(open_loop_theta_e, 0.0f, 2.0f * PI);
  }

#endif

  focRunOpenLoopVoltage(open_loop_vd, open_loop_vq, open_loop_theta_e, motor_vbus, &motor_duty);

  motor_monitor.theta_e = open_loop_theta_e;

  motor_monitor.id_ref = 0.0f;
  motor_monitor.id_meas = 0.0f;
  motor_monitor.iq_ref = 0.0f;
  motor_monitor.iq_meas = 0.0f;

  motor_monitor.vd_cmd = open_loop_vd;
  motor_monitor.vq_cmd = open_loop_vq;

  motor_monitor.duty_u = motor_duty.u;
  motor_monitor.duty_v = motor_duty.v;
  motor_monitor.duty_w = motor_duty.w;

  pwmSetDuty(motor_duty.u,
             motor_duty.v,
             motor_duty.w);
#endif  // OPEN_LOOP_DIAG_ENABLE
}

#endif

void motorControlUpdate(void)
{
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT
  #if CURRENT_NEUTRAL_DIAG_ENABLE == 0U
    float theta_e;
  #endif

    if (motor_state != MOTOR_STATE_CURRENT_LOOP)
    {
      return;
    }


#if CURRENT_ADC_RAW_SCAN_ENABLE == 1U

#if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U

    // 이미 정상 종료한 경우에는 추가 샘플 기록 금지
    if (adc_pwm_on_scan_done == true)
    {
        return;
    }

    // PWM 동작 중 BKIN 발생 여부 확인
    if ((pwmIsBreakFault() == true) ||
        ((TIM1->SR & TIM_SR_BIF) != 0U))
    {
        pwmDisableOutput();

        motor_fault = MOTOR_FAULT_BKIN;
        motor_state = MOTOR_STATE_FAULT;

        adc_pwm_on_scan_done = true;
        return;
    }

    // PWM이 아직 시작되지 않은 경우는 무시
    if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
    {
        // 기존에 PWM ON을 확인했다면 비정상 차단
        if (adc_pwm_on_scan_seen == true)
        {
            pwmDisableOutput();

            motor_fault = MOTOR_FAULT_INIT_FAIL;
            motor_state = MOTOR_STATE_FAULT;

            adc_pwm_on_scan_done = true;
        }

        return;
    }

    adc_pwm_on_scan_seen = true;

    // 기존 Raw Min/Max, Spike Count,
    // Spike Streak, OC Count 계산
    motorAdcRawScanUpdate();

    // 실제 PWM ON 시험에서는 관찰만 하지 않고
    // 0.8A 초과 측정 즉시 출력 차단
    if ((fabsf(motor_monitor.ia) >=
         CURRENT_ADC_SCAN_OC_LIMIT_A) ||
        (fabsf(motor_monitor.ib) >=
         CURRENT_ADC_SCAN_OC_LIMIT_A) ||
        (fabsf(motor_monitor.ic) >=
         CURRENT_ADC_SCAN_OC_LIMIT_A))
    {
        pwmDisableOutput();

        motor_fault = MOTOR_FAULT_SW_OVERCURRENT;
        motor_state = MOTOR_STATE_FAULT;

        adc_pwm_on_scan_done = true;
        return;
    }

    // 지정한 샘플 수에서 PWM 자동 차단
    if (adc_raw_scan.sample_count >=
        ((CURRENT_ADC_PWM_ON_SCAN_MS * 1000U) /
         CURRENT_LOOP_PERIOD_US))
    {
        pwmDisableOutput();

        adc_pwm_on_scan_done = true;
        return;
    }

#else

    // 기존 PWM OFF Raw Scan
    if ((TIM1->BDTR & TIM_BDTR_MOE) != 0U)
    {
        pwmDisableOutput();
        motorSetFault(MOTOR_FAULT_INIT_FAIL);
        return;
    }

    motorAdcRawScanUpdate();

#endif


    #elif CURRENT_FOC_BENCH_ENABLE == 1U

        // 기존 벤치마크 코드 그대로 유지U

  // MOE OFF 유지: 실제 모터 구동 금지
  if ((TIM1->BDTR & TIM_BDTR_MOE) != 0U)
  {
    pwmDisableOutput();
    motorSetFault(MOTOR_FAULT_INIT_FAIL);
    return;
  }

  // FOC + 진단 부하 전체 실행 시간 측정 시작
  uint32_t start_cycles = DWT->CYCCNT;

    #if _USE_HALL_SENSOR

  float theta_e = hallGetElectricalAngle();

#elif _USE_ENCODER

  float theta_e = encoderGetElectricalAngle();

#else

  #error "FOC benchmark requires rotor angle source"

#endif

  // 기존 전류 제어 연산 (지령 0A, MOE OFF)
  motorCurrentLoop(0.0f, 0.0f, theta_e);

  // 과전류 등으로 Fault가 발생했다면 중단
  if (motor_state == MOTOR_STATE_FAULT)
  {
    return;
  }

#if CURRENT_FOC_BENCH_DIAG_LOAD == 1U

  // 실제 회전 진단과 유사한 Hall/BKIN 검사 부하
  bool break_ok =
      (pwmIsBreakFault() == false) &&
      ((TIM1->SR & TIM_SR_BIF) == 0U);

  uint8_t hall_state = hallGetState();
  int8_t hall_sector = hallGetSectorIndex();
  bool hall_valid = hallIsValid();

  if ((!break_ok) ||
      (!hall_valid) ||
      (hall_sector < 0) ||
      (hall_state == 0U) ||
      (hall_state == 7U))
  {
    // 센서/Break 이상 시 PWM 차단 및 Fault
    pwmDisableOutput();

    if (!break_ok)
      motorSetFault(MOTOR_FAULT_BKIN);
    else
      motorSetFault(MOTOR_FAULT_HALL_INVALID);

    return;
  }

  // 진단용 3상 전류 Min/Max 갱신
  float ia = motor_monitor.ia;
  float ib = motor_monitor.ib;
  float ic = motor_monitor.ic;

  if (foc_bench_diag_index == 0U)
  {
    foc_bench_ia_min = foc_bench_ia_max = ia;
    foc_bench_ib_min = foc_bench_ib_max = ib;
    foc_bench_ic_min = foc_bench_ic_max = ic;
  }
  else
  {
    if (ia < foc_bench_ia_min) foc_bench_ia_min = ia;
    if (ia > foc_bench_ia_max) foc_bench_ia_max = ia;

    if (ib < foc_bench_ib_min) foc_bench_ib_min = ib;
    if (ib > foc_bench_ib_max) foc_bench_ib_max = ib;

    if (ic < foc_bench_ic_min) foc_bench_ic_min = ic;
    if (ic > foc_bench_ic_max) foc_bench_ic_max = ic;
  }

  // 기존 회전 진단처럼 200 ISR마다 RAM에 저장
  if ((foc_bench_diag_index %
       CURRENT_ROTATION_LOG_DIVIDER) == 0U)
  {
      uint32_t index =
          (foc_bench_diag_index /
           CURRENT_ROTATION_LOG_DIVIDER)
          % CURRENT_L_TEST_SAMPLE_COUNT;

      current_l_test_id[index] = motor_monitor.id_meas;
      current_l_test_iq[index] = motor_monitor.iq_meas;
      current_l_test_vd[index] = motor_monitor.vd_cmd;
      current_l_test_vq[index] = motor_monitor.vq_cmd;
      current_l_test_theta[index] = motor_monitor.theta_e;

      current_l_test_hall_state[index] = hall_state;
      current_l_test_hall_sector[index] = hall_sector;
      current_l_test_hall_valid[index] = hall_valid ? 1U : 0U;

      if (current_l_test_count < CURRENT_L_TEST_SAMPLE_COUNT)
      {
          current_l_test_count++;
      }
  }

  foc_bench_diag_index++;

    #endif // CURRENT_FOC_BENCH_DIAG_LOAD

  uint32_t elapsed_cycles = DWT->CYCCNT - start_cycles;

  foc_bench_count++;
  foc_bench_total_cycles += elapsed_cycles;

  if (elapsed_cycles > foc_bench_max_cycles)
  {
    foc_bench_max_cycles = elapsed_cycles;
  }

  #elif CURRENT_NEUTRAL_DIAG_ENABLE == 1U


    motorNeutralPwmDiag();

  #else

    #if _USE_HALL_SENSOR
      theta_e = hallGetElectricalAngle();
    #elif _USE_ENCODER
      theta_e = encoderGetElectricalAngle();
    #endif

    motorCurrentLoop(current_id_ref, current_iq_ref, theta_e);
  #endif

#elif MOTOR_CONTROL_MODE == MOTOR_CONTROL_SPEED
  float theta_e;

  if (motor_state != MOTOR_STATE_SPEED_LOOP)
  {
      return;
  }

  #if _USE_HALL_SENSOR
    theta_e = hallGetElectricalAngle();
  #elif _USE_ENCODER
    theta_e = encoderGetElectricalAngle();
  #else
  #error "HALL SENSOR / ENCODER ERROR"
  #endif

  speed_loop_divider++;

  if (speed_loop_divider >= SPEED_LOOP_DIVIDER)
  {
    speed_loop_divider = 0;
    motorSpeedLoop();       // iq_ref만 갱신
  }

  motorCurrentLoop(current_id_ref, current_iq_ref, theta_e);

#endif

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP
  if (motor_state != MOTOR_STATE_OPEN_LOOP)
  {
      return;
  }

  motorOpenLoop();
#endif
}

void motorSetFault(motor_fault_t fault)
{
  pwmDisableOutput();
#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP) && (OPEN_LOOP_DIAG_ENABLE == 1U)
  if (six_step.selected) sixStepGatePinsOff();
#endif

  adcInjectedStop();
  pwmStop();

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)

  // [수정]
  current_diag_active = false;

#endif

  pidReset(&pi_id);
  pidReset(&pi_iq);
  pidReset(&pi_spd);

  speed_w_target = 0.0f;
  speed_w_ref = 0.0f;
  speed_w_meas = 0.0f;

  current_id_ref = 0.0f;
  current_iq_ref = 0.0f;

  motor_fault = fault;
  motor_state = MOTOR_STATE_FAULT;
}

void motorClearFault(void)
{
  if (motor_state != MOTOR_STATE_FAULT)
  {
    return;
  }

  pwmClearBreakFault();

  motor_fault = MOTOR_FAULT_NONE;
  motor_state = MOTOR_STATE_IDLE;
}

bool motorGetMonitor(motor_monitor_t *monitor)
{
  uint32_t primask;

  if (monitor == NULL)
  {
   return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  monitor->ia = motor_monitor.ia;
  monitor->ib = motor_monitor.ib;
  monitor->ic = motor_monitor.ic;

  monitor->id_ref  = motor_monitor.id_ref;
  monitor->id_meas = motor_monitor.id_meas;
  monitor->iq_ref  = motor_monitor.iq_ref;
  monitor->iq_meas = motor_monitor.iq_meas;

  monitor->vd_cmd = motor_monitor.vd_cmd;
  monitor->vq_cmd = motor_monitor.vq_cmd;

  monitor->theta_e = motor_monitor.theta_e;

  monitor->speed_target = speed_w_target;
  monitor->speed_ref    = speed_w_ref;
  monitor->speed_meas   = speed_w_meas;

  monitor->duty_u = motor_monitor.duty_u;
  monitor->duty_v = motor_monitor.duty_v;
  monitor->duty_w = motor_monitor.duty_w;

  monitor->vbus = motor_vbus;
  monitor->temp_raw = motor_temp_raw;
  monitor->speed_cmd_raw = motor_speed_cmd_raw;

  monitor->state = motor_state;
  monitor->fault = motor_fault;

  if (primask == 0U)
  {
    __enable_irq();
  }

  return true;
}

void motorSetCurrentReference(float id_ref, float iq_ref)
{
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT

  current_id_ref = clampFloat(id_ref, OUTPUT_ID_REF_MIN, OUTPUT_ID_REF_MAX);
  current_iq_ref = clampFloat(iq_ref, OUTPUT_IQ_REF_MIN, OUTPUT_IQ_REF_MAX);

#else
  (void)id_ref;
  (void)iq_ref;
#endif
}

motor_state_t motorGetState(void)
{
  return motor_state;
}

motor_fault_t motorGetFault(void)
{
  return motor_fault;
}
#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)

void motorCurrentDiagGetAverage(motor_abc_f_t *avg)
{
    if (avg == NULL)
        return;

    avg->a = 0.0f;
    avg->b = 0.0f;
    avg->c = 0.0f;

    if (current_avg_count == 0U)
        return;

    avg->a = current_avg_sum.a / current_avg_count;
    avg->b = current_avg_sum.b / current_avg_count;
    avg->c = current_avg_sum.c / current_avg_count;
}

#endif

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && (CURRENT_FOC_BENCH_ENABLE == 1U)

void motorFocBenchGetStats( uint32_t *count, uint32_t *total_cycles, uint32_t *max_cycles)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    if (count != NULL)
        *count = foc_bench_count;

    if (total_cycles != NULL)
        *total_cycles = foc_bench_total_cycles;

    if (max_cycles != NULL)
        *max_cycles = foc_bench_max_cycles;

    // 다음 측정 구간을 위해 초기화
    foc_bench_count = 0U;
    foc_bench_total_cycles = 0U;
    foc_bench_max_cycles = 0U;

    if (primask == 0U)
    {
        __enable_irq();
    }
}

#endif

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP) && (OPEN_LOOP_DIAG_ENABLE == 1U)
void motorOpenLoopDiagSetDirection(int8_t direction)
{
  // Accepted only before motorStart, while PWM is not enabled.
  if (motor_state == MOTOR_STATE_IDLE)
    open_loop_diag_requested_direction = (direction < 0) ? -1 : 1;
}

void motorOpenLoopDiagSetFixedOnly(uint8_t fixed_only)
{
  if (motor_state == MOTOR_STATE_IDLE &&
      fixed_only <= OL_DIAG_MODE_FIXED_ID_90)
    open_loop_diag_requested_fixed_only = fixed_only;
}

void motorOpenLoopDiagSetAbsoluteStart(uint8_t enabled)
{
  if (motor_state == MOTOR_STATE_IDLE)
    open_loop_diag_requested_absolute_start = enabled ? 1U : 0U;
}

bool motorOpenLoopDiagIsDone(void)
{
  return open_loop_diag.completed != 0U;
}

void motorOpenLoopDiagGet(motor_ol_diag_t *out)
{
  if (out == NULL) return;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *out = open_loop_diag;
  if (primask == 0U) __enable_irq();
}
#endif

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP) && (_USE_HALL_OFFSET_CALIBRATION)

bool motorGetHallCalibrationEvent(int8_t *sector, int8_t *direction, float *theta_e)
{
  bool ret = false;
  uint32_t primask;

  if ((sector == NULL) || (direction == NULL) || (theta_e == NULL))
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  if (hall_cal_event_pending == true)
  {
    *sector = hall_cal_event_sector;
    *direction = hall_cal_event_direction;
    *theta_e = hall_cal_event_theta_e;

    hall_cal_event_pending = false;
    ret = true;
  }

  if (primask == 0U)
  {
    __enable_irq();
  }

  return ret;
}

#endif


