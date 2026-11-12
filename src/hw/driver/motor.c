/* K/L encoder FOC portfolio firmware.
 * 20kHz current PI, 100Hz speed PI, guarded startup/handoff and
 * 500/3000/500ms speed profile. Legacy diagnostics removed.
 * MOTOR_ALLOW_POWER_STAGE=0 keeps the power stage disabled.
 */
#include "motor.h"
#include "util.h"

static volatile motor_state_t motor_state = MOTOR_STATE_IDLE;
static volatile motor_fault_t motor_fault = MOTOR_FAULT_NONE;
static volatile motor_monitor_t motor_monitor;
static motor_duty_t motor_duty;
static pid_ctrl_t pi_id, pi_iq, pi_spd;
static volatile float motor_vbus = 0.0f;
static volatile uint16_t motor_speed_cmd_raw = 0U;
static volatile uint16_t motor_temp_raw = 0U;
static float current_id_ref = 0.0f;
static float current_iq_ref = 0.0f;
static float speed_w_ref = 0.0f;
static float speed_w_meas = 0.0f;
static volatile bool power_output_armed = false;
static uint32_t align_count = 0U;
static int32_t align_settle_reference_count = 0;
static int32_t align_settle_peak = 0;
static uint32_t closed_loop_ticks = 0U;
static uint32_t probe_start_ms = 0U;

/* Selection is made from UART while IDLE and used in the 20kHz ISR. */
static volatile bool speed_probe_selected = false;
static volatile int8_t speed_probe_direction = 0;
static volatile motor_speed_pi_diag_t speed_pi_diag;
static int32_t speed_probe_prev_raw = 0;
static int32_t speed_probe_unwrapped = 0;
static int32_t speed_probe_prev_10ms = 0;
static float speed_probe_raw_rpm = 0.0f;
static float speed_probe_filtered_rpm = 0.0f;
static float speed_probe_target_rpm = 0.0f;
static uint32_t speed_profile_start_tick = 0U;
static float speed_profile_start_rpm = 0.0f;
static uint8_t speed_profile_segment = 0U;
static uint8_t speed_profile_stop_steps = 0U;
static float speed_probe_iq_limit = 0.0f;
static volatile bool speed_probe_voltage_enabled = false;
static motor_speed_phase_t speed_probe_phase = MOTOR_SPEED_PHASE_NONE;
static uint32_t speed_probe_phase_start_tick = 0U;
static uint8_t speed_probe_start_confirm = 0U;
static uint8_t speed_probe_run_stall_steps = 0U;
static uint8_t speed_probe_handoff_run_confirm = 0U;
static uint8_t speed_probe_handoff_stall_steps = 0U;
static bool speed_probe_handoff_fast_ramp = false;
static uint8_t speed_probe_restart_count = 0U;
static int32_t speed_probe_start_forward_count = 0;
static float speed_probe_start_current_a = 0.0f;

static int32_t motorAbsCount(int32_t x) { return x < 0 ? -x : x; }
static int32_t motorCountDelta(int32_t a, int32_t b)
{
  int32_t delta = a - b;
  const int32_t n = (int32_t)ENCODER_MAX_COUNT;
  if (delta > n / 2) delta -= n;
  if (delta < -(n / 2)) delta += n;
  return delta;
}

bool motorSelectSpeedPiProbe(int8_t direction)
{
  if (motor_state != MOTOR_STATE_IDLE || motor_fault != MOTOR_FAULT_NONE ||
      (direction != +1 && direction != -1)) return false;
  if (CURRENT_TEST_OC_LIMIT_A <= (MOTOR_SPEED_PI_OC_MARGIN_A + 0.5f))
    return false;
  speed_probe_selected = true;
  speed_probe_direction = direction;
  return true;
}

bool motorIsSpeedPiProbeSelected(void) { return speed_probe_selected; }

bool motorGetSpeedPiProbeDiag(motor_speed_pi_diag_t *result)
{
  if (result == NULL) return false;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *result = speed_pi_diag;
  __set_PRIMASK(primask);
  return result->selected != 0U;
}


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
  if(pidInit(&pi_spd, MOTOR_SPEED_PI_KP_A_PER_RPM, MOTOR_SPEED_PI_KI_A_PER_RPM_S, 0.0f, OUTPUT_IQ_REF_MIN, OUTPUT_IQ_REF_MAX, INTEGRAL_SPD_MIN, INTEGRAL_SPD_MAX) != true)
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
  if (encoderInit() != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
  if (ret != true)
  {
    motorSetFault(MOTOR_FAULT_INIT_FAIL);
  }
  return ret;
}

/* 0=none, 1=BKIN flag before MOE, 2=MOE failed after enable */
volatile uint8_t motor_bkin_test_stage = 0U;
volatile uint32_t motor_bkin_test_sr = 0U;
volatile uint32_t motor_bkin_test_bdtr = 0U;
volatile uint32_t motor_bkin_test_pa6 = 0U;
volatile uint8_t motor_bkin_test_latched = 0U;

void motorStart(void)
{
#if MOTOR_ALLOW_POWER_STAGE == 0U
  pwmDisableOutput();
  return;
#else
  motor_bkin_test_stage = 0U;
  motor_bkin_test_latched = 0U;
  if (motor_state != MOTOR_STATE_IDLE || !speed_probe_selected ||
      motor_fault != MOTOR_FAULT_NONE) return;
  if (!adcUpdateRegular())
  {
    motorSetFault(MOTOR_FAULT_ADC_REGULAR_FAIL);
    return;
  }
  motor_vbus = adcGetVbusVoltage();
  if (motor_vbus < MOTOR_VBUS_MIN)
  {
    motorSetFault(MOTOR_FAULT_VBUS_LOW);
    return;
  }
  if (motor_vbus > ALIGN_DIAG_VBUS_MAX_V)
  {
    motorSetFault(MOTOR_FAULT_VBUS_HIGH);
    return;
  }
  motor_speed_cmd_raw = adcGetSpeedRaw();
  motor_temp_raw = adcGetTempRaw();

  pidReset(&pi_id);
  pidReset(&pi_iq);
  pidReset(&pi_spd);
  pi_id.out_min = OUTPUT_VD_REF_MIN;
  pi_id.out_max = OUTPUT_VD_REF_MAX;
  pi_iq.out_min = OUTPUT_VQ_REF_MIN;
  pi_iq.out_max = OUTPUT_VQ_REF_MAX;
  pi_id.integral_min = INTEGRAL_ID_MIN;
  pi_id.integral_max = INTEGRAL_ID_MAX;
  pi_iq.integral_min = INTEGRAL_IQ_MIN;
  pi_iq.integral_max = INTEGRAL_IQ_MAX;
  pi_spd.kp = MOTOR_SPEED_PI_KP_A_PER_RPM;
  pi_spd.ki = MOTOR_SPEED_PI_KI_A_PER_RPM_S;
  pi_spd.kd = 0.0f;
  pi_spd.out_min = OUTPUT_IQ_REF_MIN;
  pi_spd.out_max = OUTPUT_IQ_REF_MAX;
  pi_spd.integral_min = INTEGRAL_SPD_MIN;
  pi_spd.integral_max = INTEGRAL_SPD_MAX;
  speed_probe_voltage_enabled = false;

  align_count = 0U;
  align_settle_reference_count = 0;
  align_settle_peak = 0;
  closed_loop_ticks = 0U;
  memset((void *)&speed_pi_diag, 0, sizeof(speed_pi_diag));
  speed_pi_diag.selected = 1U;
  speed_pi_diag.direction = speed_probe_direction;
  speed_probe_unwrapped = 0;
  speed_probe_prev_raw = 0;
  speed_probe_prev_10ms = 0;
  speed_probe_raw_rpm = 0.0f;
  speed_probe_filtered_rpm = 0.0f;
  speed_probe_target_rpm = 0.0f;
  speed_probe_iq_limit = fminf(MOTOR_SPEED_PI_IQ_MAX_A,
                    CURRENT_TEST_OC_LIMIT_A - MOTOR_SPEED_PI_OC_MARGIN_A);
  speed_pi_diag.iq_limit_used = speed_probe_iq_limit;
  speed_probe_phase = MOTOR_SPEED_PHASE_NONE;
  speed_probe_phase_start_tick = 0U;
  speed_probe_restart_count = 0U;
  speed_probe_start_forward_count = 0;
  speed_probe_start_current_a = 0.0f;
  speed_probe_start_confirm = 0U;
  speed_probe_run_stall_steps = 0U;
  speed_probe_handoff_run_confirm = 0U;
  speed_probe_handoff_stall_steps = 0U;
  speed_probe_handoff_fast_ramp = false;
  speed_profile_start_tick = 0U;
  speed_profile_start_rpm = 0.0f;
  speed_profile_segment = 0U;
  speed_profile_stop_steps = 0U;
  encoderClearCalibration();
  encoderResetSpeedTracking();

  current_id_ref = 0.0f;
  current_iq_ref = 0.0f;
  speed_w_ref = 0.0f;
  speed_w_meas = 0.0f;
  pwmSetDuty(0.5f, 0.5f, 0.5f);
  adcInjectedStart();
  pwmStart();
  if (!adcCalibrateCurrentOffset())
  {
    motorSetFault(MOTOR_FAULT_ADC_OFFSET_FAIL);
    return;
  }
  if (pwmIsBreakFault() || (TIM1->SR & TIM_SR_BIF))
  {
    motor_bkin_test_stage = 1U;
    motorSetFault(MOTOR_FAULT_BKIN);
    return;
  }
  probe_start_ms = HAL_GetTick();
  /* Keep PWM enable and the software armed flag atomic with respect to IRQs.
     TIM1 hardware break remains operational even with IRQs masked. */
  const uint32_t saved_primask = __get_PRIMASK();
  __disable_irq();
  motor_state = MOTOR_STATE_ALIGN;
  pwmEnableOutput();
  /* Do not reject MOE on the immediate readback; check on ADC ISR instead. */
  power_output_armed = true;
  if (saved_primask == 0U) __enable_irq();
#endif
}

void motorStop(void)
{
  power_output_armed = false;
  pwmDisableOutput();
  adcInjectedStop();
  pwmStop();
  pidReset(&pi_id);
  pidReset(&pi_iq);
  pidReset(&pi_spd);
  current_id_ref = 0.0f;
  current_iq_ref = 0.0f;
  speed_w_ref = 0.0f;
  speed_w_meas = 0.0f;
  speed_probe_selected = false;
  speed_probe_direction = 0;
  speed_probe_voltage_enabled = false;
  motor_state = MOTOR_STATE_IDLE;
}

/* Same 2A, 2s static alignment used by the successful K/L profile runs.
 * The optional 3.5A vector-sweep diagnostic is not part of this firmware.
 * Static settle checking remains mandatory before installing the offset. */
void motorAlign(void)
{
  if (motor_state != MOTOR_STATE_ALIGN) return;
  const uint32_t t = align_count;
  float id_ref = ALIGN_ID;
  if (t < ALIGN_RAMP_TICKS)
    id_ref = ALIGN_ID * ((float)(t + 1U) / (float)ALIGN_RAMP_TICKS);

  /* Capture encoder response through directional alignment. */
  if (t == 0U)
    speed_pi_diag.diag_align_begin_count = (int32_t)encoderGetCount();
  if (t == ALIGN_RAMP_TICKS)
    speed_pi_diag.diag_align_ramp_count = (int32_t)encoderGetCount();

  /* Approach the same electrical zero from the requested rotation side.
   * A smooth 90-degree electrical pre-alignment uses the existing 2A/2s
   * alignment window; its final 1.6s still holds theta=0 for settling. */
  float align_theta = 0.0f;
  if (t < ALIGN_RAMP_TICKS)
    align_theta = -(float)speed_probe_direction * (0.5f * PI) *
        (1.0f - (float)t / (float)ALIGN_RAMP_TICKS);

  motorCurrentLoop(id_ref, 0.0f, align_theta);
  if (motor_state == MOTOR_STATE_FAULT) return;
  if (t == ALIGN_TOTAL_TICKS - ALIGN_SETTLE_TICKS)
  {
    align_settle_reference_count = (int32_t)encoderGetCount();
    align_settle_peak = 0;
  }
  if (t >= ALIGN_TOTAL_TICKS - ALIGN_SETTLE_TICKS)
  {
    const int32_t delta = motorAbsCount(motorCountDelta(
        (int32_t)encoderGetCount(), align_settle_reference_count));
    if (delta > align_settle_peak) align_settle_peak = delta;
  }
  align_count++;
  if (align_count < ALIGN_TOTAL_TICKS) return;

  const int32_t final_count = (int32_t)encoderGetCount();
  speed_pi_diag.diag_align_final_count = final_count;
  speed_pi_diag.align_base_count = final_count;
  speed_pi_diag.align_static_settle_peak_counts = align_settle_peak;
  if (align_settle_peak > ALIGN_SETTLE_MAX_COUNTS)
  {
    speed_pi_diag.result = 2U;
    speed_pi_diag.align_count = final_count;
    speed_pi_diag.stop_raw = final_count;
    power_output_armed = false;
    pwmDisableOutput();
    motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
    return; /* Never calibrate or enter current loop after failed alignment. */
  }

  const float raw_theta = encoderGetElectricalAngle();
  if (!isfinite(raw_theta))
  {
    motorSetFault(MOTOR_FAULT_ENCODER_INVALID);
    return;
  }
  speed_pi_diag.diag_align_raw_theta = raw_theta;
  speed_pi_diag.diag_offset_theta = wrapFloat(-raw_theta, 0.0f, 2.0f * PI);
  encoderSetElectricalOffset(-raw_theta);
  speed_pi_diag.diag_calibrated_theta = encoderGetElectricalAngle();
  if (!encoderIsCalibrated())
  {
    motorSetFault(MOTOR_FAULT_ENCODER_INVALID);
    return;
  }
  speed_pi_diag.align_count = final_count;
  closed_loop_ticks = 0U;
  /* Preserve original current-PI integral across Id alignment release. */
  motor_state = MOTOR_STATE_CURRENT_LOOP;
}


static bool motorSpeedProfileSampleDue(uint32_t u)
{
  if (u <= 8000U) return (u % 200U) == 0U;
  return (u % 1000U) == 0U;
}

static void motorCaptureSpeedPiSample(uint32_t u, int32_t raw)
{
  if (speed_pi_diag.sample_count >= MOTOR_SPEED_PI_SAMPLE_COUNT) return;
  volatile motor_speed_pi_sample_t *s =
      &speed_pi_diag.samples[speed_pi_diag.sample_count];
  s->tick = u;
  s->phase = (uint8_t)speed_probe_phase;
  s->profile_segment = speed_profile_segment;
  s->raw_count = raw;
  s->unwrapped_count = speed_probe_unwrapped;
  s->rpm_raw = speed_probe_raw_rpm;
  s->rpm_filtered = speed_probe_filtered_rpm;
  s->rpm_ref = speed_probe_target_rpm;
  s->iq_ref = motor_monitor.iq_ref;
  s->iq_meas = motor_monitor.iq_meas;
  s->id_meas = motor_monitor.id_meas;
  s->vd = motor_monitor.vd_cmd;
  s->vq = motor_monitor.vq_cmd;
  s->vmag = sqrtf(s->vd * s->vd + s->vq * s->vq);
  s->theta_e = motor_monitor.theta_e;
  speed_pi_diag.sample_count++;
}

static void motorFinishSpeedPiProbe(uint8_t result, uint32_t t, int32_t raw)
{
  speed_pi_diag.result = result;
  speed_pi_diag.last_phase = (uint8_t)speed_probe_phase;
  speed_pi_diag.stop_tick = t;
  speed_pi_diag.stop_raw = raw;
  speed_pi_diag.total_counts = speed_probe_unwrapped;
  speed_pi_diag.stop_rpm = speed_probe_filtered_rpm;
  speed_probe_voltage_enabled = false;
  power_output_armed = false;
  pwmDisableOutput();
  motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
}

static void motorCommissionSpeedPiProbe(void)
{
  const uint32_t t = closed_loop_ticks;
  const int32_t raw = (int32_t)encoderGetCount();
  const int32_t from_align = motorCountDelta(raw, speed_pi_diag.align_count);
  const float theta = encoderGetElectricalAngle();
  if (!isfinite(theta))
  {
    motorSetFault(MOTOR_FAULT_ENCODER_INVALID);
    return;
  }

  /* Alignment Id decay and zero-current settle: keep legacy voltage cap. */
  if (t < MOTOR_SPEED_PI_START_TICK)
  {
    if (motorAbsCount(from_align) > MOTOR_SPEED_ALIGN_RELEASE_MAX_MOVE_COUNTS)
    {
      motorFinishSpeedPiProbe(7U, t, raw);
      return;
    }
    float id_ref = 0.0f;
    if (t < MOTOR_SPEED_ID_DECAY_TICKS)
      id_ref = ALIGN_ID * (1.0f - (float)t / (float)MOTOR_SPEED_ID_DECAY_TICKS);
    motorCurrentLoop(id_ref, 0.0f, theta);
    if (motor_state == MOTOR_STATE_FAULT) return;
    closed_loop_ticks++;
    return;
  }

  const uint32_t u = t - MOTOR_SPEED_PI_START_TICK;
  if (u == 0U)
  {
    speed_pi_diag.zero_count = raw;
    speed_pi_diag.diag_start_zero_count = raw;
    speed_pi_diag.diag_start_theta = theta;
    speed_probe_prev_raw = raw;
    speed_probe_unwrapped = 0;
    speed_probe_prev_10ms = 0;
    speed_probe_raw_rpm = 0.0f;
    speed_probe_filtered_rpm = 0.0f;
    speed_probe_target_rpm = 0.0f;
    speed_probe_phase = MOTOR_SPEED_PHASE_STARTUP;
    speed_profile_start_tick = 0U;
    speed_profile_start_rpm = 0.0f;
    speed_profile_segment = 0U;
    speed_profile_stop_steps = 0U;
    speed_probe_phase_start_tick = 0U;
    speed_probe_restart_count = 0U;
    speed_probe_start_forward_count = 0;
    speed_probe_start_current_a = 0.0f;
    speed_probe_start_confirm = 0U;
    speed_probe_run_stall_steps = 0U;
    speed_probe_handoff_run_confirm = 0U;
    speed_probe_handoff_stall_steps = 0U;
    speed_probe_handoff_fast_ramp = false;
    current_iq_ref = 0.0f;
    encoderResetSpeedTracking();

    /* Expand BOTH current PI axes plus final vector limit, speed mode only. */
    pi_id.out_min = -MOTOR_SPEED_PI_V_LIMIT_V;
    pi_id.out_max = MOTOR_SPEED_PI_V_LIMIT_V;
    pi_iq.out_min = -MOTOR_SPEED_PI_V_LIMIT_V;
    pi_iq.out_max = MOTOR_SPEED_PI_V_LIMIT_V;
    pi_id.integral_min = -MOTOR_SPEED_PI_V_LIMIT_V;
    pi_id.integral_max = MOTOR_SPEED_PI_V_LIMIT_V;
    pi_iq.integral_min = -MOTOR_SPEED_PI_V_LIMIT_V;
    pi_iq.integral_max = MOTOR_SPEED_PI_V_LIMIT_V;
    pidReset(&pi_id);
    pidReset(&pi_iq);
    pidReset(&pi_spd);
    pi_spd.kp = MOTOR_SPEED_PI_KP_A_PER_RPM;
    pi_spd.ki = MOTOR_SPEED_PI_KI_A_PER_RPM_S;
    pi_spd.kd = 0.0f;
    pi_spd.out_min = -speed_probe_iq_limit;
    pi_spd.out_max = speed_probe_iq_limit;
    pi_spd.integral_min = -MOTOR_SPEED_INTEGRAL_LIMIT_A;
    pi_spd.integral_max = MOTOR_SPEED_INTEGRAL_LIMIT_A;
    speed_probe_voltage_enabled = true;
  }
  else
  {
    /* Per-current-loop delta, so 0<->3999 rollover cannot hide turns. */
    const int32_t d = motorCountDelta(raw, speed_probe_prev_raw);
    speed_probe_unwrapped += d;
    speed_probe_prev_raw = raw;
  }

  const int32_t forward = -((int32_t)speed_probe_direction) * speed_probe_unwrapped;
  if (forward > speed_pi_diag.maximum_forward_counts)
    speed_pi_diag.maximum_forward_counts = forward;

  /* Update speed, state machine and Iq command at exactly 100 Hz. */
  if (u > 0U && (u % MOTOR_SPEED_PI_TICKS_PER_STEP) == 0U)
  {
    encoderUpdateSpeed(0.010f);
    const int32_t d10 = speed_probe_unwrapped - speed_probe_prev_10ms;
    speed_probe_prev_10ms = speed_probe_unwrapped;
    speed_probe_raw_rpm = -(float)d10 *
        (60.0f * 100.0f / (float)ENCODER_MAX_COUNT);
    speed_probe_filtered_rpm += MOTOR_SPEED_PI_FILTER_ALPHA *
        (speed_probe_raw_rpm - speed_probe_filtered_rpm);
    if (fabsf(speed_probe_raw_rpm) > speed_pi_diag.peak_abs_rpm)
      speed_pi_diag.peak_abs_rpm = fabsf(speed_probe_raw_rpm);

    const float iq_step = MOTOR_SPEED_IQ_SLEW_A_PER_S * 0.010f;
    const float goal = (float)speed_probe_direction * MOTOR_SPEED_PI_GOAL_RPM;

    /* Crucial: detect successful startup BEFORE applying the current command
       for this 10ms step. The old code first drove +3A at K's 80ms transition
       and only then set HANDOFF; L happened to be already softened. */
    if (speed_probe_phase == MOTOR_SPEED_PHASE_STARTUP ||
        speed_probe_phase == MOTOR_SPEED_PHASE_RESTART)
    {
      const uint32_t elapsed = u - speed_probe_phase_start_tick;
      const int32_t forward_from_start = forward - speed_probe_start_forward_count;
      const float rpm_along = (float)speed_probe_direction * speed_probe_raw_rpm;

      if (forward_from_start >= MOTOR_SPEED_START_DETECT_COUNTS &&
          rpm_along >= MOTOR_SPEED_START_DETECT_RPM)
      {
        if (speed_probe_start_confirm < MOTOR_SPEED_START_CONFIRM_STEPS)
          speed_probe_start_confirm++;
      }
      else
        speed_probe_start_confirm = 0U;

      const bool decisive_start =
          (forward_from_start >= 100 &&
           (float)speed_probe_direction * speed_probe_filtered_rpm >= 60.0f);
      if (speed_probe_start_confirm >= MOTOR_SPEED_START_CONFIRM_STEPS ||
          decisive_start)
      {
        const bool was_restart =
            speed_probe_phase == MOTOR_SPEED_PHASE_RESTART;
        speed_probe_phase = MOTOR_SPEED_PHASE_HANDOFF;
        speed_probe_phase_start_tick = u;
        if (was_restart)
          speed_pi_diag.restart_recovered_tick = u;
        else
          speed_pi_diag.startup_end_tick = u;
        speed_pi_diag.rpm_at_startup_end = speed_probe_filtered_rpm;
        speed_probe_handoff_fast_ramp =
            ((float)speed_probe_direction * speed_probe_filtered_rpm <
             MOTOR_SPEED_HANDOFF_FAST_THRESHOLD_RPM);
        speed_probe_handoff_run_confirm = 0U;
        speed_probe_handoff_stall_steps = 0U;

        /* Track the already attained speed, while respecting the goal. */
        const float speed_abs = clampFloat(
            (float)speed_probe_direction * speed_probe_filtered_rpm,
            MOTOR_SPEED_HANDOFF_MIN_RPM, MOTOR_SPEED_PI_GOAL_RPM);
        speed_probe_target_rpm = (float)speed_probe_direction * speed_abs;
        pidReset(&pi_spd);
        const float e = speed_probe_target_rpm - speed_probe_filtered_rpm;
        pi_spd.integral = clampFloat(current_iq_ref - pi_spd.kp * e,
                                     pi_spd.integral_min, pi_spd.integral_max);
        /* Fall through to HANDOFF control on THIS update: do not send
           another startup-current step after detecting actual rotation. */
      }
      else if (elapsed >= MOTOR_SPEED_STARTUP_TIMEOUT_TICKS)
      {
        motorFinishSpeedPiProbe(
            speed_probe_phase == MOTOR_SPEED_PHASE_RESTART ? 10U : 8U,
            t, raw);
        return;
      }
      else
      {
        const float startup_iq = fminf(MOTOR_SPEED_STARTUP_IQ_A,
                                         speed_probe_iq_limit);
        const float ramp = clampFloat(
            (float)elapsed / (float)MOTOR_SPEED_STARTUP_RAMP_TICKS,
            0.0f, 1.0f);
        /* During retry, keep the already applied positive drive current
           continuous. A fresh first start always ramps from zero. */
        const float ramp_a = speed_probe_start_current_a +
            (startup_iq - speed_probe_start_current_a) * ramp;
        float desired = (float)speed_probe_direction * ramp_a;
        if (rpm_along >= MOTOR_SPEED_STARTUP_STOP_RPM) desired = 0.0f;
        else if (rpm_along > MOTOR_SPEED_STARTUP_SOFT_RPM)
          desired *= (MOTOR_SPEED_STARTUP_STOP_RPM - rpm_along) /
                     (MOTOR_SPEED_STARTUP_STOP_RPM - MOTOR_SPEED_STARTUP_SOFT_RPM);
        const float step = ((float)speed_probe_direction * desired <
            (float)speed_probe_direction * current_iq_ref) ?
            MOTOR_SPEED_IQ_DECEL_SLEW_A_PER_S * 0.010f : iq_step;
        current_iq_ref += clampFloat(desired - current_iq_ref,
                                     -step, step);
      }
    }

    if (speed_probe_phase == MOTOR_SPEED_PHASE_HANDOFF ||
        speed_probe_phase == MOTOR_SPEED_PHASE_RUN)
    {
      /* L: retain legacy 400RPM/s; K's low-speed handoff uses the
       * faster profile latched when STARTUP was confirmed. */
      /* In RUN, profile reference is generated at 100Hz (same PI tick).
       * The first RUN tick remains continuous with HANDOFF's reference. */
      float speed_goal = goal;
      if (speed_probe_phase == MOTOR_SPEED_PHASE_RUN)
      {
        const uint32_t profile_ms =
            (u - speed_profile_start_tick) / 20U;
        const uint32_t accel_end = MOTOR_SPEED_PROFILE_ACCEL_MS;
        const uint32_t hold_end = accel_end + MOTOR_SPEED_PROFILE_HOLD_MS;
        const uint32_t decel_end = hold_end + MOTOR_SPEED_PROFILE_DECEL_MS;
        float magnitude;
        if (profile_ms < accel_end)
        {
          speed_profile_segment = 1U;
          magnitude = speed_profile_start_rpm +
              (MOTOR_SPEED_PI_GOAL_RPM - speed_profile_start_rpm) *
              ((float)profile_ms / (float)MOTOR_SPEED_PROFILE_ACCEL_MS);
        }
        else if (profile_ms < hold_end)
        {
          speed_profile_segment = 2U;
          magnitude = MOTOR_SPEED_PI_GOAL_RPM;
        }
        else if (profile_ms < decel_end)
        {
          speed_profile_segment = 3U;
          magnitude = MOTOR_SPEED_PI_GOAL_RPM *
              (1.0f - (float)(profile_ms - hold_end) /
              (float)MOTOR_SPEED_PROFILE_DECEL_MS);
        }
        else
        {
          speed_profile_segment = 4U;
          magnitude = 0.0f;
        }
        speed_goal = (float)speed_probe_direction * magnitude;
      }
      const float ramp_rpm_s =
          (speed_probe_phase == MOTOR_SPEED_PHASE_HANDOFF &&
           speed_probe_handoff_fast_ramp) ?
          MOTOR_SPEED_HANDOFF_FAST_RAMP_RPM_PER_S :
          MOTOR_SPEED_PI_RAMP_RPM_PER_S;
      const float increment = ramp_rpm_s * 0.010f;
      if (speed_probe_phase == MOTOR_SPEED_PHASE_RUN)
        speed_probe_target_rpm = speed_goal;
      else
        speed_probe_target_rpm += clampFloat(goal - speed_probe_target_rpm,
                                             -increment, increment);
      const float pi_request = piControllerAntiWindup(&pi_spd,
          speed_probe_target_rpm, speed_probe_filtered_rpm, 0.010f);
      float bounded_request = clampFloat(pi_request,
                                      -speed_probe_iq_limit, speed_probe_iq_limit);
      /* Brake and settling must not demand a large reverse current. */
      if (speed_probe_phase == MOTOR_SPEED_PHASE_RUN &&
          speed_profile_segment >= 3U)
        bounded_request = clampFloat(bounded_request,
            -MOTOR_SPEED_PROFILE_BRAKE_IQ_A,
            MOTOR_SPEED_PROFILE_BRAKE_IQ_A);
      const bool removing_drive =
          ((float)speed_probe_direction * bounded_request <
           (float)speed_probe_direction * current_iq_ref);
      const float step = removing_drive ?
          MOTOR_SPEED_IQ_DECEL_SLEW_A_PER_S * 0.010f : iq_step;
      current_iq_ref += clampFloat(bounded_request - current_iq_ref,
                                   -step, step);
      /* Prevent PI integrator from winding up against slew limits. */
      pi_spd.integral = clampFloat(
          pi_spd.integral + MOTOR_SPEED_INTEGRAL_TRACK_GAIN *
          (current_iq_ref - bounded_request),
          pi_spd.integral_min, pi_spd.integral_max);

      /* Record the current actually commanded on the first HANDOFF tick. */
      if (speed_probe_restart_count == 0U &&
          speed_probe_phase == MOTOR_SPEED_PHASE_HANDOFF &&
          speed_pi_diag.startup_end_tick == u)
        speed_pi_diag.iq_at_startup_end = current_iq_ref;

      bool restart_requested = false;
      if (speed_probe_phase == MOTOR_SPEED_PHASE_HANDOFF)
      {
        const uint32_t handoff_elapsed = u - speed_probe_phase_start_tick;
        const float rpm_along = (float)speed_probe_direction *
                                speed_probe_filtered_rpm;

        /* Require 3 consecutive speed samples in the commanded direction.
         * A current-only handoff must not transition into RUN at 0RPM. */
        if (rpm_along >= MOTOR_SPEED_HANDOFF_RUN_MIN_RPM)
        {
          if (speed_probe_handoff_run_confirm <
              MOTOR_SPEED_HANDOFF_RUN_CONFIRM_STEPS)
            speed_probe_handoff_run_confirm++;
        }
        else
          speed_probe_handoff_run_confirm = 0U;

        /* A slow/motionless HANDOFF needs the SAME bounded retry as RUN.
         * Only consider actual near-zero speed (not reverse rotation). */
        if (handoff_elapsed >= MOTOR_SPEED_HANDOFF_TICKS &&
            fabsf(speed_probe_filtered_rpm) <
                MOTOR_SPEED_HANDOFF_STALL_ABS_RPM)
        {
          if (speed_probe_handoff_stall_steps <
              MOTOR_SPEED_HANDOFF_STALL_STEPS)
            speed_probe_handoff_stall_steps++;
        }
        else
          speed_probe_handoff_stall_steps = 0U;

        if (handoff_elapsed >= MOTOR_SPEED_HANDOFF_TICKS &&
            speed_probe_handoff_run_confirm >=
                MOTOR_SPEED_HANDOFF_RUN_CONFIRM_STEPS &&
            fabsf(bounded_request - current_iq_ref) <=
                MOTOR_SPEED_HANDOFF_EXIT_IQ_A)
        {
          speed_probe_phase = MOTOR_SPEED_PHASE_RUN;
          speed_pi_diag.handoff_end_tick = u;
          speed_probe_run_stall_steps = 0U;
          /* Bumpless transfer: start profile from the current speed reference,
           * not from 0RPM while the rotor is already moving. */
          speed_profile_start_tick = u;
          speed_profile_start_rpm = clampFloat(
              (float)speed_probe_direction * speed_probe_target_rpm,
              0.0f, MOTOR_SPEED_PI_GOAL_RPM);
          speed_pi_diag.profile_start_tick = u;
          speed_pi_diag.profile_initial_rpm =
              (float)speed_probe_direction * speed_profile_start_rpm;
          speed_profile_segment = 1U;
          speed_profile_stop_steps = 0U;
        }
        else if (speed_probe_handoff_stall_steps >=
                     MOTOR_SPEED_HANDOFF_STALL_STEPS)
        {
          restart_requested = true;
        }
        else if (handoff_elapsed >= MOTOR_SPEED_HANDOFF_MAX_TICKS)
        {
          /* No stable RUN transition: stop rather than re-boost a rotor
           * that may still be moving. A retry is only for near-zero stall. */
          motorFinishSpeedPiProbe(5U, t, raw);
          return;
        }
      }

      if (speed_probe_phase == MOTOR_SPEED_PHASE_RUN)
      {
        if (speed_profile_segment < 3U &&
            (float)speed_probe_direction * speed_probe_filtered_rpm < 5.0f)
        {
          if (speed_probe_run_stall_steps < MOTOR_SPEED_RUN_STALL_STEPS)
            speed_probe_run_stall_steps++;
        }
        else
          speed_probe_run_stall_steps = 0U;
        if (speed_probe_run_stall_steps >= MOTOR_SPEED_RUN_STALL_STEPS)
          restart_requested = true;
      }

      if (restart_requested)
      {
        /* One retry at most, regardless of whether HANDOFF or RUN stalls.
         * Keep all existing current, speed and travel limits unchanged. */
        const uint32_t reserve = MOTOR_SPEED_STARTUP_TIMEOUT_TICKS +
            MOTOR_SPEED_HANDOFF_TICKS + MOTOR_SPEED_RESTART_RESERVE_TICKS;
        if (speed_probe_restart_count < MOTOR_SPEED_RESTART_MAX_ATTEMPTS &&
            u < MOTOR_SPEED_PI_RUN_TICKS &&
            MOTOR_SPEED_PI_RUN_TICKS - u >= reserve)
        {
          speed_probe_restart_count++;
          speed_pi_diag.restart_count = speed_probe_restart_count;
          speed_pi_diag.restart_start_tick = u;
          speed_probe_phase = MOTOR_SPEED_PHASE_RESTART;
          speed_probe_phase_start_tick = u;
          speed_probe_start_forward_count = forward;
          speed_probe_start_confirm = 0U;
          speed_probe_run_stall_steps = 0U;
          speed_probe_handoff_run_confirm = 0U;
          speed_probe_handoff_stall_steps = 0U;
          speed_probe_handoff_fast_ramp = false;
          speed_profile_segment = 0U;
          speed_profile_stop_steps = 0U;
          speed_probe_start_current_a = clampFloat(
              (float)speed_probe_direction * current_iq_ref,
              0.0f, fminf(MOTOR_SPEED_STARTUP_IQ_A, speed_probe_iq_limit));
          /* PI tracking is reset only when RESTART confirms motion. */
        }
        else
        {
          motorFinishSpeedPiProbe(5U, t, raw);
          return;
        }
      }
    }
    /* After decel, require near-zero measured RPM for 30ms before PWM off.
     * A fixed timeout prevents indefinite motor energization. */
    if (speed_probe_phase == MOTOR_SPEED_PHASE_RUN &&
        speed_profile_segment == 4U)
    {
      const uint32_t profile_ms =
          (u - speed_profile_start_tick) / 20U;
      const uint32_t decel_end = MOTOR_SPEED_PROFILE_ACCEL_MS +
          MOTOR_SPEED_PROFILE_HOLD_MS + MOTOR_SPEED_PROFILE_DECEL_MS;
      if (fabsf(speed_probe_filtered_rpm) <= MOTOR_SPEED_PROFILE_STOP_RPM &&
          fabsf(speed_probe_raw_rpm) <= MOTOR_SPEED_PROFILE_STOP_RPM + 5.0f)
      {
        if (speed_profile_stop_steps < MOTOR_SPEED_PROFILE_STOP_STEPS)
          speed_profile_stop_steps++;
      }
      else
        speed_profile_stop_steps = 0U;
      if (speed_profile_stop_steps >= MOTOR_SPEED_PROFILE_STOP_STEPS)
      {
        motorCaptureSpeedPiSample(u, raw);
        motorFinishSpeedPiProbe(1U, t, raw);
        return;
      }
      if (profile_ms >= decel_end + MOTOR_SPEED_PROFILE_STOP_TIMEOUT_MS)
      {
        motorCaptureSpeedPiSample(u, raw);
        motorFinishSpeedPiProbe(11U, t, raw);
        return;
      }
    }
    current_iq_ref = clampFloat(current_iq_ref,
                                -speed_probe_iq_limit, speed_probe_iq_limit);
    speed_w_ref = speed_probe_target_rpm * (2.0f * PI / 60.0f);
    speed_w_meas = speed_probe_filtered_rpm * (2.0f * PI / 60.0f);
  }

  /* Keep raw RPM, wrong direction, travel, stall and total-time cutoffs. */
  if (fabsf(speed_probe_raw_rpm) >= MOTOR_SPEED_PI_MAX_RPM)
  { motorFinishSpeedPiProbe(3U, t, raw); return; }
  if (forward < -MOTOR_SPEED_PI_REVERSE_COUNTS)
  { motorFinishSpeedPiProbe(4U, t, raw); return; }
  if (u >= MOTOR_SPEED_PI_STALL_TICKS &&
      motorAbsCount(speed_probe_unwrapped) < MOTOR_SPEED_PI_STALL_COUNTS)
  { motorFinishSpeedPiProbe(5U, t, raw); return; }
  if (motorAbsCount(speed_probe_unwrapped) >= MOTOR_SPEED_PI_MAX_TRAVEL_COUNTS)
  { motorFinishSpeedPiProbe(6U, t, raw); return; }
  if (u >= MOTOR_SPEED_PI_RUN_TICKS)
  { motorFinishSpeedPiProbe(11U, t, raw); return; }

  motorCurrentLoop(0.0f, current_iq_ref, theta);
  if (motor_state == MOTOR_STATE_FAULT) return;

  if (speed_pi_diag.sample_count < MOTOR_SPEED_PI_SAMPLE_COUNT &&
      motorSpeedProfileSampleDue(u))
    motorCaptureSpeedPiSample(u, raw);

  closed_loop_ticks++;
}

void motorLowSpeedTask(void)
{
  /* The injected callback normally ends the probe after ~3 seconds.
     Independently cut the power stage if those ADC interrupts stop. */
  /* The previous 5500ms watchdog included the 2s alignment phase and
   * incorrectly cut off the 4.0s portfolio profile during its hold segment.
   * Select a bounded whole-test watchdog for K/L only; preserve the legacy
   * watchdog for other motor tests and retain all fast current/speed guards. */
  const uint32_t timeout_ms = MOTOR_SPEED_PI_WATCHDOG_MS;
  if ((motor_state == MOTOR_STATE_ALIGN ||
       motor_state == MOTOR_STATE_CURRENT_LOOP) &&
      (uint32_t)(HAL_GetTick() - probe_start_ms) > timeout_ms)
  {
    motorSetFault(MOTOR_FAULT_TEST_TIMEOUT);
    return;
  }
  // Keep the first ADC error stage latched after a fault.
  if (motor_state == MOTOR_STATE_FAULT)
  {
    return;
  }
  if (!adcUpdateRegular())
  {
    motorSetFault(MOTOR_FAULT_ADC_REGULAR_FAIL);
    return;
  }
  motor_vbus = adcGetVbusVoltage();
  if (motor_state == MOTOR_STATE_ALIGN || motor_state == MOTOR_STATE_CURRENT_LOOP)
  {
    if (motor_vbus < MOTOR_VBUS_MIN)
    {
      motorSetFault(MOTOR_FAULT_VBUS_LOW);
      return;
    }
    if (motor_vbus > ALIGN_DIAG_VBUS_MAX_V)
    {
      motorSetFault(MOTOR_FAULT_VBUS_HIGH);
      return;
    }
  }
  motor_speed_cmd_raw = adcGetSpeedRaw();
  motor_temp_raw = adcGetTempRaw();
}

void motorCurrentLoop(float id_ref, float iq_ref, float theta_e)
{
  motor_abc_f_t i_abc;
  motor_alphabeta_t i_ab, v_ab;
  motor_dq_t i_dq, v_dq;

  adcGetPhaseCurrent(&i_abc);
  motor_monitor.ia = i_abc.a;
  motor_monitor.ib = i_abc.b;
  motor_monitor.ic = i_abc.c;

  if (!isfinite(theta_e) || !isfinite(i_abc.a) || !isfinite(i_abc.b) || !isfinite(i_abc.c))
  {
    motorSetFault(MOTOR_FAULT_ENCODER_INVALID);
    return;
  }
  if (fabsf(i_abc.a) > CURRENT_TEST_OC_LIMIT_A ||
      fabsf(i_abc.b) > CURRENT_TEST_OC_LIMIT_A ||
      fabsf(i_abc.c) > CURRENT_TEST_OC_LIMIT_A)
  {
    motorSetFault(MOTOR_FAULT_SW_OVERCURRENT);
    return;
  }

  focClarke(i_abc.a, i_abc.b, i_abc.c, &i_ab);
  focPark(i_ab.alpha, i_ab.beta, theta_e, &i_dq);
  motor_monitor.id_ref  = id_ref;
  motor_monitor.id_meas = i_dq.d;
  motor_monitor.iq_ref  = iq_ref;
  motor_monitor.iq_meas = i_dq.q;
  motor_monitor.theta_e = theta_e;

  v_dq.d = piControllerAntiWindup(&pi_id, id_ref, i_dq.d, CUR_DT);
  v_dq.q = piControllerAntiWindup(&pi_iq, iq_ref, i_dq.q, CUR_DT);

  /* Static alignment remains radially limited to 1.8V.
   * Speed trial alone uses a larger SPWM-bounded d/q vector limit. */
  float v_limit = 0.45f * motor_vbus;
  if ((motor_state == MOTOR_STATE_ALIGN ||
       motor_state == MOTOR_STATE_CURRENT_LOOP) &&
      v_limit > ALIGN_DIAG_VECTOR_VOLT_LIMIT_V)
  {
    v_limit = ALIGN_DIAG_VECTOR_VOLT_LIMIT_V;
  }
  if (speed_probe_voltage_enabled && speed_probe_selected &&
      motor_state == MOTOR_STATE_CURRENT_LOOP)
  {
    v_limit = fminf(0.45f * motor_vbus, MOTOR_SPEED_PI_V_LIMIT_V);
  }
  const float vd_pre_limit = v_dq.d;
  const float vq_pre_limit = v_dq.q;
  focSetVoltageLimit(&v_dq, v_limit);
  if (speed_probe_voltage_enabled && speed_probe_selected &&
      motor_state == MOTOR_STATE_CURRENT_LOOP)
  {
    /* Coupled radial anti-windup: reconcile each PI integrator with the
     * actual applied d/q vector after the final magnitude clamp. */
    if (v_dq.d != vd_pre_limit || v_dq.q != vq_pre_limit)
    {
      pi_id.integral = clampFloat(pi_id.integral + 0.25f * (v_dq.d - vd_pre_limit),
                                   pi_id.integral_min, pi_id.integral_max);
      pi_iq.integral = clampFloat(pi_iq.integral + 0.25f * (v_dq.q - vq_pre_limit),
                                   pi_iq.integral_min, pi_iq.integral_max);
    }
    const float applied_mag = sqrtf(v_dq.d*v_dq.d + v_dq.q*v_dq.q);
    if (applied_mag >= v_limit - 0.015f)
      speed_pi_diag.voltage_saturation_ticks++;
    if (applied_mag > speed_pi_diag.max_abs_vmag)
      speed_pi_diag.max_abs_vmag = applied_mag;
  }
  motor_monitor.vd_cmd = v_dq.d;
  motor_monitor.vq_cmd = v_dq.q;

  focInvPark(v_dq.d, v_dq.q, theta_e, &v_ab);
  focGenerateSPWM(v_ab.alpha, v_ab.beta, motor_vbus, &motor_duty);
  motor_monitor.duty_u = motor_duty.u;
  motor_monitor.duty_v = motor_duty.v;
  motor_monitor.duty_w = motor_duty.w;
  pwmSetDuty(motor_duty.u, motor_duty.v, motor_duty.w);
}

void motorControlUpdate(void)
{
  if (motor_state != MOTOR_STATE_CURRENT_LOOP && motor_state != MOTOR_STATE_ALIGN)
    return;
  if (!power_output_armed) return;
  if (pwmIsBreakFault() || (TIM1->SR & TIM_SR_BIF))
  {
    motorSetFault(MOTOR_FAULT_BKIN);
    return;
  }
  if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
  {
    motorSetFault(MOTOR_FAULT_INIT_FAIL);
    return;
  }
  if (motor_state == MOTOR_STATE_ALIGN)
  {
    motorAlign();
    return;
  }
  if (!encoderIsCalibrated())
  {
    motorSetFault(MOTOR_FAULT_ENCODER_INVALID);
    return;
  }
  if (!isfinite(encoderGetElectricalAngle()))
  {
    motorSetFault(MOTOR_FAULT_ENCODER_INVALID);
    return;
  }
  if (!speed_probe_selected)
  {
    motorSetFault(MOTOR_FAULT_INIT_FAIL);
    return;
  }
  motorCommissionSpeedPiProbe();
}


void motorSetFault(motor_fault_t fault)
{
  speed_probe_voltage_enabled = false;
  power_output_armed = false;
  pwmDisableOutput();
  adcInjectedStop();
  pwmStop();
  pidReset(&pi_id); pidReset(&pi_iq); pidReset(&pi_spd);
  current_id_ref = 0.0f; current_iq_ref = 0.0f;
  motor_fault = fault;
  motor_state = MOTOR_STATE_FAULT;
}

void motorClearFault(void)
{
  if (motor_state != MOTOR_STATE_FAULT) return;
  speed_probe_selected = false;
  speed_probe_direction = 0;
  speed_probe_voltage_enabled = false;
  pwmClearBreakFault();
  motor_fault = MOTOR_FAULT_NONE;
  motor_state = MOTOR_STATE_IDLE;
}

bool motorGetMonitor(motor_monitor_t *monitor)
{
  if (monitor == NULL) return false;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  monitor->ia = motor_monitor.ia;
  monitor->ib = motor_monitor.ib;
  monitor->ic = motor_monitor.ic;
  monitor->id_ref = motor_monitor.id_ref;
  monitor->id_meas = motor_monitor.id_meas;
  monitor->iq_ref = motor_monitor.iq_ref;
  monitor->iq_meas = motor_monitor.iq_meas;
  monitor->vd_cmd = motor_monitor.vd_cmd;
  monitor->vq_cmd = motor_monitor.vq_cmd;
  monitor->theta_e = motor_monitor.theta_e;
  monitor->duty_u = motor_monitor.duty_u;
  monitor->duty_v = motor_monitor.duty_v;
  monitor->duty_w = motor_monitor.duty_w;
  monitor->speed_target = (float)speed_probe_direction *
      MOTOR_SPEED_PI_GOAL_RPM * (2.0f * PI / 60.0f);
  monitor->speed_ref = speed_w_ref;
  monitor->speed_meas = speed_w_meas;
  monitor->vbus = motor_vbus;
  monitor->temp_raw = motor_temp_raw;
  monitor->speed_cmd_raw = motor_speed_cmd_raw;
  monitor->state = motor_state;
  monitor->fault = motor_fault;
  __set_PRIMASK(primask);
  return true;
}

motor_state_t motorGetState(void)
{
  return motor_state;
}

motor_fault_t motorGetFault(void)
{
  return motor_fault;
}
