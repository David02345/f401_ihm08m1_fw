/* Minimal encoder-feedback FOC core, refactored from user's motor.c.
 * Hall startup/sweep/trace modes intentionally removed.
 * PWM output stays hardware-OFF while MOTOR_ALLOW_POWER_STAGE == 0.
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

static volatile float speed_w_target = 0.0f;
static float speed_w_ref = 0.0f;
static float speed_w_meas = 0.0f;
static uint16_t speed_loop_divider = 0U;
static volatile bool power_output_armed = false;

static volatile uint32_t align_count = 0U;
static int32_t align_settle_reference_count = 0;
static int32_t align_settle_peak = 0;
static int32_t position_zero_count = 0;
static uint32_t closed_loop_ticks = 0U;
static uint32_t probe_start_ms = 0U;
/* Accepted position for deadbanded travel estimation at 1 kHz. */
static int32_t position_prev_count = 0;
static int32_t position_accumulated_travel = 0;
/* Raw 20 kHz absolute travel retained for diagnostics, NOT trip decision. */
static int32_t position_raw_prev_count = 0;
static int32_t position_raw_accumulated_travel = 0;
static float position_ref_count = 0.0f;
static float position_meas_count = 0.0f;
static float position_error_count = 0.0f;
static float position_iq_cmd = 0.0f;
static float position_shutdown_iq = 0.0f;
/* 1 kHz LPF / derivative damping state. Raw TIM2 is used for safety. */
static float position_filtered_count = 0.0f;
static float position_filter_prev_count = 0.0f;
static float position_vel_counts_s = 0.0f;
static float position_raw_meas_count = 0.0f;
static float position_iq_p_term = 0.0f;
static float position_iq_d_term = 0.0f;
/* Separate 1 kHz variation of zero-Iq stage from active PD stage. */
static int32_t zero_iq_start_count = 0;
static int32_t zero_iq_prev_count = 0;
static int32_t drive_prev_count = 0;
/* 0=running; 1=completed; 2=alignment unstable;
 * 3=move/travel limit; 4=large position error (possible wrong feedback). */
static volatile uint8_t align_gate_status = 0U;
/* Selection is set from UART only while IDLE; sampled in ADC ISR. */
static volatile int8_t torque_probe_direction = 0;
static volatile bool torque_probe_high_current = false;
static volatile motor_torque_probe_diag_t torque_diag;
static volatile bool spin_probe_selected = false;
static volatile int8_t spin_probe_direction = 0;
static volatile motor_spin_probe_diag_t spin_diag;
static int32_t spin_prev_raw_count = 0;
static int32_t spin_unwrapped_count = 0;
static int32_t spin_10ms_prev_count = 0;
static float spin_rpm_10ms = 0.0f;
static volatile bool speed_probe_selected = false;
static volatile int8_t speed_probe_direction = 0;
static volatile motor_speed_pi_diag_t speed_pi_diag;
static int32_t speed_probe_prev_raw = 0;
static int32_t speed_probe_unwrapped = 0;
static int32_t speed_probe_prev_10ms = 0;
static float speed_probe_raw_rpm = 0.0f;
static float speed_probe_filtered_rpm = 0.0f;
static float speed_probe_target_rpm = 0.0f;
static float speed_probe_iq_limit = 0.0f;
static volatile bool speed_probe_voltage_enabled = false;
static motor_speed_phase_t speed_probe_phase = MOTOR_SPEED_PHASE_NONE;
static uint32_t speed_probe_phase_start_tick = 0U;
static uint8_t speed_probe_start_confirm = 0U;
static uint8_t speed_probe_run_stall_steps = 0U;

static volatile bool vector_probe_selected = false;
static volatile motor_vector_probe_diag_t vector_diag;
static volatile bool offset_probe_selected = false;
static volatile int8_t offset_probe_direction = 0;
static volatile int16_t offset_probe_delta_deg = 0;
static volatile motor_offset_probe_diag_t offset_diag;

static int32_t motorAbsCount(int32_t x) { return x < 0 ? -x : x; }

static int32_t motorCountDelta(int32_t a, int32_t b)
{
  int32_t delta = a - b;
  const int32_t n = (int32_t)ENCODER_MAX_COUNT;
  if (delta > n / 2) delta -= n;
  if (delta < -(n / 2)) delta += n;
  return delta;
}

uint8_t motorGetAlignGateStatus(void) { return align_gate_status; }

bool motorSelectTorqueProbe(int8_t direction)
{
  if (motor_state != MOTOR_STATE_IDLE || direction < -1 || direction > 1)
    return false;
  torque_probe_direction = direction;
  torque_probe_high_current = false;
  spin_probe_selected = false;
  spin_probe_direction = 0;
  speed_probe_selected = false;
  speed_probe_direction = 0;
  vector_probe_selected = false;
  offset_probe_selected = false;
  return true;
}

bool motorSelectHighIqProbe(int8_t direction)
{
  if (motor_state != MOTOR_STATE_IDLE || motor_fault != MOTOR_FAULT_NONE ||
      (direction != +1 && direction != -1))
    return false;
  /* Reuse the proven F/R sequence but choose a separate target and limit. */
  if (!motorSelectTorqueProbe(direction)) return false;
  torque_probe_high_current = true;
  return true;
}

bool motorSelectSpinProbe(int8_t direction)
{
  if (motor_state != MOTOR_STATE_IDLE || motor_fault != MOTOR_FAULT_NONE ||
      (direction != +1 && direction != -1)) return false;
  /* Do not touch def.h, ADC limits, PI tuning or existing T/Y/F/R/V tests. */
  torque_probe_direction = 0;
  torque_probe_high_current = false;
  offset_probe_selected = false;
  vector_probe_selected = false;
  spin_probe_direction = direction;
  spin_probe_selected = true;
  speed_probe_selected = false;
  speed_probe_direction = 0;
  return true;
}

bool motorSelectSpeedPiProbe(int8_t direction)
{
  if (motor_state != MOTOR_STATE_IDLE || motor_fault != MOTOR_FAULT_NONE ||
      (direction != +1 && direction != -1)) return false;
  if (CURRENT_TEST_OC_LIMIT_A <= (MOTOR_SPEED_PI_OC_MARGIN_A + 0.5f))
    return false;
  speed_probe_selected = true;
  speed_probe_direction = direction;
  spin_probe_selected = false;
  spin_probe_direction = 0;
  torque_probe_direction = 0;
  torque_probe_high_current = false;
  vector_probe_selected = false;
  offset_probe_selected = false;
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

bool motorIsSpinProbeSelected(void) { return spin_probe_selected; }

bool motorGetSpinProbeDiag(motor_spin_probe_diag_t *result)
{
  if (result == NULL) return false;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *result = spin_diag;
  __set_PRIMASK(primask);
  return result->selected != 0U;
}

bool motorSelectVectorProbe(void)
{
  if (motor_state != MOTOR_STATE_IDLE) return false;
  vector_probe_selected = true;
  spin_probe_selected = false;
  spin_probe_direction = 0;
  speed_probe_selected = false;
  speed_probe_direction = 0;
  torque_probe_direction = 0;
  torque_probe_high_current = false;
  offset_probe_selected = false;
  return true;
}

bool motorSelectOffsetProbe(int8_t direction, uint8_t offset_index)
{
  if (motor_state != MOTOR_STATE_IDLE || motor_fault != MOTOR_FAULT_NONE ||
      (direction != +1 && direction != -1) ||
      offset_index >= OFFSET_PROBE_COUNT)
    return false;
  offset_probe_direction = direction;
  offset_probe_delta_deg = (int16_t)
      (((int32_t)offset_index - 2) * OFFSET_PROBE_STEP_DEG);
  offset_probe_selected = true;
  spin_probe_selected = false;
  spin_probe_direction = 0;
  speed_probe_selected = false;
  speed_probe_direction = 0;
  torque_probe_direction = 0;
  torque_probe_high_current = false;
  vector_probe_selected = false;
  return true;
}

bool motorIsOffsetProbeSelected(void)
{
  return offset_probe_selected;
}

bool motorGetOffsetProbeDiag(motor_offset_probe_diag_t *result)
{
  if (result == NULL) return false;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *result = offset_diag;
  __set_PRIMASK(primask);
  return (result->selected != 0U);
}

bool motorIsVectorProbeSelected(void)
{
  return vector_probe_selected;
}

bool motorGetVectorProbeDiag(motor_vector_probe_diag_t *result)
{
  if (result == NULL) return false;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *result = vector_diag;
  __set_PRIMASK(primask);
  return (result->selected != 0U);
}

int8_t motorGetTorqueProbeDirection(void)
{
  return torque_probe_direction;
}

bool motorGetTorqueProbeDiag(motor_torque_probe_diag_t *result)
{
  if (result == NULL) return false;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *result = torque_diag;
  __set_PRIMASK(primask);
  return (result->direction != 0);
}

static void motorCaptureTorqueSample(uint32_t index, uint32_t t, int32_t now)
{
  if (index >= MOTOR_TORQUE_PROBE_SAMPLE_COUNT) return;
  volatile motor_torque_probe_sample_t *s = &torque_diag.samples[index];
  s->tick = t;
  s->encoder_count = now;
  s->position_counts = motorCountDelta(now, torque_diag.zero_count);
  s->theta_e = motor_monitor.theta_e;
  s->id_ref = motor_monitor.id_ref;
  s->id_meas = motor_monitor.id_meas;
  s->iq_ref = motor_monitor.iq_ref;
  s->iq_meas = motor_monitor.iq_meas;
  s->vq = motor_monitor.vq_cmd;
  torque_diag.sample_mask |= (uint16_t)(1U << index);
}

static void motorCaptureVectorSample(uint32_t index, uint32_t t,
                                     int32_t now, float theta_cmd)
{
  if (index >= MOTOR_VECTOR_PROBE_SAMPLE_COUNT) return;
  volatile motor_vector_probe_sample_t *s = &vector_diag.samples[index];
  s->tick = t;
  s->encoder_count = now;
  s->relative_count = motorCountDelta(now, vector_diag.align_count);
  s->theta_cmd = theta_cmd;
  s->id_ref = motor_monitor.id_ref;
  s->id_meas = motor_monitor.id_meas;
  s->iq_ref = motor_monitor.iq_ref;
  s->iq_meas = motor_monitor.iq_meas;
  s->vd = motor_monitor.vd_cmd;
  s->vq = motor_monitor.vq_cmd;
  s->ia = motor_monitor.ia;
  s->ib = motor_monitor.ib;
  s->ic = motor_monitor.ic;
  s->duty_u = motor_monitor.duty_u;
  s->duty_v = motor_monitor.duty_v;
  s->duty_w = motor_monitor.duty_w;
  vector_diag.sample_mask |= (uint16_t)(1U << index);
}
/* Capture snapshots inside the ADC ISR; print only after PWM OFF. */
static void motorCaptureOffsetSample(uint32_t index, uint32_t t, int32_t now)
{
  if (index >= MOTOR_OFFSET_PROBE_SAMPLE_COUNT) return;
  volatile motor_offset_probe_sample_t *s = &offset_diag.samples[index];
  s->tick = t;
  s->encoder_count = now;
  s->relative_count = motorCountDelta(now, offset_diag.zero_count);
  s->theta_e = motor_monitor.theta_e;
  s->id_ref = motor_monitor.id_ref;
  s->id_meas = motor_monitor.id_meas;
  s->iq_ref = motor_monitor.iq_ref;
  s->iq_meas = motor_monitor.iq_meas;
  s->vd = motor_monitor.vd_cmd;
  s->vq = motor_monitor.vq_cmd;
  s->ia = motor_monitor.ia;
  s->ib = motor_monitor.ib;
  s->ic = motor_monitor.ic;
  offset_diag.sample_mask |= (uint16_t)(1U << index);
}

/* Sampled only at known PWM control ticks. Accessed from main via IRQ lock. */
static volatile motor_align_diag_t align_diag;

static void motorCaptureAlignSample(uint32_t index, uint32_t tick)
{
  if (index >= MOTOR_DIAG_SAMPLE_COUNT) return;
  volatile motor_align_diag_sample_t *s = &align_diag.samples[index];
  s->tick = tick;
  s->encoder_count = (int32_t)encoderGetCount();
  s->theta_e = motor_monitor.theta_e;
  s->id_ref = motor_monitor.id_ref;
  s->id_meas = motor_monitor.id_meas;
  s->iq_ref = motor_monitor.iq_ref;
  s->iq_meas = motor_monitor.iq_meas;
  s->vd = motor_monitor.vd_cmd;
  s->vq = motor_monitor.vq_cmd;
  s->ia = motor_monitor.ia;
  s->ib = motor_monitor.ib;
  s->ic = motor_monitor.ic;
  s->duty_u = motor_monitor.duty_u;
  s->duty_v = motor_monitor.duty_v;
  s->duty_w = motor_monitor.duty_w;
  s->pos_ref_counts = position_ref_count;
  s->pos_meas_counts = position_meas_count;
  s->pos_error_counts = position_error_count;
  s->pos_raw_counts = position_raw_meas_count;
  s->pos_vel_counts_per_s = position_vel_counts_s;
  s->pos_p_iq_a = position_iq_p_term;
  s->pos_d_iq_a = position_iq_d_term;
  s->ccr1 = TIM1->CCR1;
  s->ccr2 = TIM1->CCR2;
  s->ccr3 = TIM1->CCR3;
  s->ccer = TIM1->CCER;
  s->moe = (uint8_t)((TIM1->BDTR & TIM_BDTR_MOE) != 0U);
  align_diag.valid_mask |= (1U << index);
}

bool motorGetAlignDiag(motor_align_diag_t *result)
{
  if (result == NULL) return false;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *result = align_diag;
  __set_PRIMASK(primask);
  return true;
}

volatile uint8_t debug_bkin_stage = 0U;
volatile uint32_t debug_bkin_sr = 0U;
volatile uint32_t debug_bkin_bdtr = 0U;
volatile uint32_t debug_bkin_pa6 = 0U;
volatile uint32_t bdtr_before;
volatile uint32_t sr_before;

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
  if(pidInit(&pi_spd, SPD_KP, SPD_KI, SPD_KD, OUTPUT_IQ_REF_MIN, OUTPUT_IQ_REF_MAX, INTEGRAL_SPD_MIN, INTEGRAL_SPD_MAX) != true)
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
#if MOTOR_ALLOW_POWER_STAGE == 0U

  pwmDisableOutput();
  return;
#else
  if (motor_state != MOTOR_STATE_IDLE)
  {
    return;
  }
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
  /* Avoid inheriting the expanded voltage/PI limits after a speed trial. */
  pi_id.out_min = OUTPUT_VD_REF_MIN;
  pi_id.out_max = OUTPUT_VD_REF_MAX;
  pi_iq.out_min = OUTPUT_VQ_REF_MIN;
  pi_iq.out_max = OUTPUT_VQ_REF_MAX;
  pi_id.integral_min = INTEGRAL_ID_MIN;
  pi_id.integral_max = INTEGRAL_ID_MAX;
  pi_iq.integral_min = INTEGRAL_IQ_MIN;
  pi_iq.integral_max = INTEGRAL_IQ_MAX;
  pi_spd.kp = SPD_KP;
  pi_spd.ki = SPD_KI;
  pi_spd.kd = SPD_KD;
  pi_spd.out_min = OUTPUT_IQ_REF_MIN;
  pi_spd.out_max = OUTPUT_IQ_REF_MAX;
  pi_spd.integral_min = INTEGRAL_SPD_MIN;
  pi_spd.integral_max = INTEGRAL_SPD_MAX;
  speed_probe_voltage_enabled = false;

  align_count = 0U;
  align_settle_reference_count = 0;
  align_settle_peak = 0;
  position_zero_count = 0;
  closed_loop_ticks = 0U;
  position_prev_count = 0;
  position_accumulated_travel = 0;
  position_raw_prev_count = 0;
  position_raw_accumulated_travel = 0;
  position_ref_count = 0.0f;
  position_meas_count = 0.0f;
  position_error_count = 0.0f;
  position_iq_cmd = 0.0f;
  position_shutdown_iq = 0.0f;
  position_filtered_count = 0.0f;
  position_filter_prev_count = 0.0f;
  position_vel_counts_s = 0.0f;
  position_raw_meas_count = 0.0f;
  position_iq_p_term = 0.0f;
  position_iq_d_term = 0.0f;
  zero_iq_start_count = 0;
  zero_iq_prev_count = 0;
  drive_prev_count = 0;
  align_gate_status = 0U;
  memset((void *)&torque_diag, 0, sizeof(torque_diag));
  torque_diag.direction = torque_probe_direction;
  torque_diag.high_current = torque_probe_high_current ? 1U : 0U;
  torque_diag.move_limit_counts = torque_probe_high_current ?
      MOTOR_HIGH_IQ_PROBE_MAX_MOVE_COUNTS : TORQUE_PROBE_MAX_MOVE_COUNTS;
  memset((void *)&spin_diag, 0, sizeof(spin_diag));
  spin_diag.selected = spin_probe_selected ? 1U : 0U;
  spin_diag.direction = spin_probe_direction;
  spin_prev_raw_count = 0;
  spin_unwrapped_count = 0;
  spin_10ms_prev_count = 0;
  spin_rpm_10ms = 0.0f;
  memset((void *)&speed_pi_diag, 0, sizeof(speed_pi_diag));
  speed_pi_diag.selected = speed_probe_selected ? 1U : 0U;
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
  speed_probe_start_confirm = 0U;
  speed_probe_run_stall_steps = 0U;
  memset((void *)&vector_diag, 0, sizeof(vector_diag));
  vector_diag.selected = vector_probe_selected ? 1U : 0U;
  memset((void *)&offset_diag, 0, sizeof(offset_diag));
  offset_diag.selected = offset_probe_selected ? 1U : 0U;
  offset_diag.direction = offset_probe_direction;
  offset_diag.offset_deg = offset_probe_delta_deg;
  memset((void *)&align_diag, 0, sizeof(align_diag));
  align_diag.start_count = (int32_t)encoderGetCount();
  torque_diag.align_count = align_diag.start_count;
  torque_diag.zero_count = align_diag.start_count;
  vector_diag.align_count = align_diag.start_count;
  offset_diag.align_count = align_diag.start_count;
  offset_diag.zero_count = align_diag.start_count;

  encoderClearCalibration();
  encoderResetSpeedTracking();

  current_id_ref = 0.0f;
  current_iq_ref = 0.0f;
  speed_w_ref = 0.0f;
  speed_w_meas = 0.0f;
  speed_loop_divider = 0U;

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
    debug_bkin_stage = 1U;
    debug_bkin_sr = TIM1->SR;
    debug_bkin_bdtr = TIM1->BDTR;
    debug_bkin_pa6 = (GPIOA->IDR >> 6) & 1U;

    motorSetFault(MOTOR_FAULT_BKIN);
    return;
  }

  probe_start_ms = HAL_GetTick();
  motor_state = MOTOR_STATE_ALIGN;

  bdtr_before = TIM1->BDTR;
  sr_before = TIM1->SR;

  pwmEnableOutput();
  /*
  if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
  {
    debug_bkin_stage = 2U;
    debug_bkin_sr = TIM1->SR;
    debug_bkin_bdtr = TIM1->BDTR;
    debug_bkin_pa6 = (GPIOA->IDR >> 6) & 1U;

    motorSetFault(MOTOR_FAULT_BKIN);
    return;
  }
  */
  uint32_t bdtr_after = TIM1->BDTR;
  uint32_t sr_after = TIM1->SR;

  debug_bkin_bdtr = bdtr_after;
  debug_bkin_sr = sr_after;
  debug_bkin_pa6 = (GPIOA->IDR >> 6) & 1U;

  if ((bdtr_after & TIM_BDTR_MOE) == 0U)
  {
    debug_bkin_stage = 2U;
    motorSetFault(MOTOR_FAULT_BKIN);
    return;
  }


  power_output_armed = true;
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
  speed_w_target = 0.0f;
  speed_w_ref = 0.0f;
  speed_w_meas = 0.0f;
  speed_loop_divider = 0U;
  torque_probe_direction = 0;
  torque_probe_high_current = false;
  spin_probe_selected = false;
  spin_probe_direction = 0;
  speed_probe_selected = false;
  speed_probe_direction = 0;
  speed_probe_voltage_enabled = false;
  vector_probe_selected = false;
  offset_probe_selected = false;
  offset_probe_direction = 0;
  motor_state = MOTOR_STATE_IDLE;
}

/* Fixed 0-degree static alignment followed by limited position feedback.
 * The rotor must first settle at a reproducible stator vector.
 * Offset is provisional, so the subsequent position controller is bounded.
 */
void motorAlign(void)
{
  if (motor_state != MOTOR_STATE_ALIGN) return;

  const uint32_t t = align_count;

  /* K/L only: validate the static 0deg alignment with an active vector
     excursion before accepting an encoder offset. Other modes untouched. */
  if (speed_probe_selected && t >= ALIGN_TOTAL_TICKS)
  {
    const uint32_t v = t - ALIGN_TOTAL_TICKS;
    const uint32_t v1 = MOTOR_SPEED_ALIGN_RAMP_TICKS;
    const uint32_t v2 = v1 + MOTOR_SPEED_ALIGN_UP_TICKS;
    const uint32_t v3 = v2 + MOTOR_SPEED_ALIGN_HOLD_TICKS;
    const uint32_t v4 = v3 + MOTOR_SPEED_ALIGN_BACK_TICKS;
    const uint32_t vend = MOTOR_SPEED_ALIGN_TEST_TICKS;
    float id = MOTOR_SPEED_ALIGN_ID_A;
    float theta = 0.0f;
    if (v < v1)
      id = ALIGN_ID + (MOTOR_SPEED_ALIGN_ID_A - ALIGN_ID) *
          ((float)v / (float)v1);
    else if (v < v2)
      theta = (0.5f * PI) * ((float)(v-v1) / (float)MOTOR_SPEED_ALIGN_UP_TICKS);
    else if (v < v3) theta = 0.5f * PI;
    else if (v < v4)
      theta = (0.5f * PI) * (1.0f - (float)(v-v3) /
          (float)MOTOR_SPEED_ALIGN_BACK_TICKS);

    motorCurrentLoop(id, 0.0f, theta);
    if (motor_state == MOTOR_STATE_FAULT) return;
    const int32_t pos = (int32_t)encoderGetCount();
    const int32_t delta = motorCountDelta(pos, speed_pi_diag.align_base_count);
    const int32_t ad = motorAbsCount(delta);
    if (ad > speed_pi_diag.align_peak_delta) speed_pi_diag.align_peak_delta = ad;
    if (ad > MOTOR_SPEED_ALIGN_MAX_EXCURSION_COUNTS)
    {
      speed_pi_diag.result = 9U;
      speed_pi_diag.align_return_count = pos;
      speed_pi_diag.align_return_delta = delta;
      power_output_armed = false;
      pwmDisableOutput();
      motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
      return;
    }
    if (v == v3 - 1U)
    {
      speed_pi_diag.align_plus_count = pos;
      speed_pi_diag.align_plus_delta = delta;
    }
    if (v == v4) align_settle_reference_count = pos;
    if (v >= v4)
    {
      const int32_t settle = motorAbsCount(motorCountDelta(pos, align_settle_reference_count));
      if (settle > align_settle_peak) align_settle_peak = settle;
    }
    align_count++;
    if (v + 1U < vend) return;
    speed_pi_diag.align_return_count = pos;
    speed_pi_diag.align_return_delta = delta;
    if (motorAbsCount(speed_pi_diag.align_plus_delta) < MOTOR_SPEED_ALIGN_MIN_RESPONSE_COUNTS ||
        motorAbsCount(delta) > MOTOR_SPEED_ALIGN_MAX_RESIDUAL_COUNTS ||
        motorAbsCount(motorCountDelta(pos, align_settle_reference_count)) > MOTOR_SPEED_ALIGN_LAST_SETTLE_COUNTS)
    {
      speed_pi_diag.result = 9U;
      power_output_armed = false;
      pwmDisableOutput();
      motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
      return;
    }
    speed_pi_diag.align_vector_pass = 1U;
    /* Continue below to install the encoder electrical offset. */
  }
  else
  {
  float id_ref = ALIGN_ID;
  if (t < ALIGN_RAMP_TICKS)
    id_ref = ALIGN_ID * ((float)(t + 1U) / (float)ALIGN_RAMP_TICKS);

  /* The applied stator current vector direction is fixed at zero. */
  motorCurrentLoop(id_ref, 0.0f, 0.0f);
  if (motor_state == MOTOR_STATE_FAULT) return;

  /* S0..S3: static alignment at 0.35, 0.90, 1.80, 2.00 s. */
  if (t == 7000U)  motorCaptureAlignSample(0U, t);
  if (t == 18000U) motorCaptureAlignSample(1U, t);
  if (t == 36000U) motorCaptureAlignSample(2U, t);
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
  if (t == ALIGN_TOTAL_TICKS - 1U) motorCaptureAlignSample(3U, t);

  align_count++;
  if (align_count < ALIGN_TOTAL_TICKS) return;
  if (speed_probe_selected)
  {
    speed_pi_diag.align_base_count = (int32_t)encoderGetCount();
    speed_pi_diag.align_peak_delta = 0;
    /* Preserve the original 2s settle gate before applying 3.5A. */
    if (align_settle_peak > ALIGN_SETTLE_MAX_COUNTS)
    {
      speed_pi_diag.result = 2U;
      speed_pi_diag.align_count = speed_pi_diag.align_base_count;
      power_output_armed = false;
      pwmDisableOutput();
      motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
    }
    return;
  }
  } /* end original 2s static alignment section */

  const int32_t final_count = (int32_t)encoderGetCount();
  align_diag.align_count = final_count;
  align_diag.settle_peak_counts = align_settle_peak;

  if (!speed_probe_selected && align_settle_peak > ALIGN_SETTLE_MAX_COUNTS)
  {
    align_gate_status = 2U; /* no Iq control attempted */
    if (torque_probe_direction != 0) torque_diag.result = 2U;
    if (speed_probe_selected)
    {
      speed_pi_diag.result = 2U;
      speed_pi_diag.align_count = final_count;
      speed_pi_diag.stop_raw = final_count;
    }
    if (spin_probe_selected)
    {
      spin_diag.result = 2U;
      spin_diag.align_count = final_count;
      spin_diag.end_count = final_count;
    }
    if (offset_probe_selected)
    {
      offset_diag.result = 2U;
      offset_diag.align_count = final_count;
      offset_diag.end_count = final_count;
    }
    if (vector_probe_selected)
    {
      vector_diag.result = 2U;
      vector_diag.align_count = final_count;
      vector_diag.end_count = final_count;
    }
    align_diag.end_count = final_count;
    power_output_armed = false;
    pwmDisableOutput();
    motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
    return;
  }

  /* encoderClearCalibration() was called at motorStart, so this is RAW
     electrical angle. The 0-deg stator field is assumed to align the rotor.
     No guarantee of flux polarity or encoder sign: this is a short probe. */
  const float raw_theta = encoderGetElectricalAngle();
  if (!isfinite(raw_theta))
  {
    motorSetFault(MOTOR_FAULT_ENCODER_INVALID);
    return;
  }
  encoderSetElectricalOffset(-raw_theta);
  if (!encoderIsCalibrated())
  {
    motorSetFault(MOTOR_FAULT_ENCODER_INVALID);
    return;
  }

  /* Store applied offset, NOT the calibrated theta near zero. */
  align_diag.encoder_offset_rad = wrapFloat(-raw_theta, 0.0f, 2.0f * PI);
  if (torque_probe_direction != 0) torque_diag.align_count = final_count;
  if (spin_probe_selected) spin_diag.align_count = final_count;
  if (speed_probe_selected) speed_pi_diag.align_count = final_count;
  if (vector_probe_selected) vector_diag.align_count = final_count;
  if (offset_probe_selected)
  {
    offset_diag.align_count = final_count;
    offset_diag.base_offset_rad = align_diag.encoder_offset_rad;
  }
  closed_loop_ticks = 0U;
  position_zero_count = final_count; /* overwritten after Id release/settle */
  position_prev_count = final_count;
  position_accumulated_travel = 0;
  position_raw_prev_count = final_count;
  position_raw_accumulated_travel = 0;
  position_iq_cmd = 0.0f;
  position_shutdown_iq = 0.0f;
  position_ref_count = 0.0f;
  position_meas_count = 0.0f;
  position_error_count = 0.0f;
  position_filtered_count = 0.0f;
  position_filter_prev_count = 0.0f;
  position_vel_counts_s = 0.0f;
  position_raw_meas_count = 0.0f;
  position_iq_p_term = 0.0f;
  position_iq_d_term = 0.0f;
  zero_iq_start_count = final_count;
  zero_iq_prev_count = final_count;
  drive_prev_count = final_count;

  /* No PI reset here: initial current reference (Id=2A, Iq=0) and
     electrical angle (~0) are continuous across this transition. */
  motor_state = MOTOR_STATE_CURRENT_LOOP;
}

/* Isolated fixed-Iq diagnostic using ENCODER electrical angle feedback.
 * One pulse per run: choose F (+Iq) or R (-Iq). The position controller
 * is bypassed entirely; ADC, Clarke/Park, d/q current PI, inverse Park,
 * PWM, break/overcurrent, supply monitor, and watchdog are unchanged.
 * No UART printing in the 20kHz ADC interrupt.
 */
static void motorCommissionTorqueProbe(void)
{
  const uint32_t t = closed_loop_ticks;
  const int32_t now = (int32_t)encoderGetCount();
  const int32_t from_align = motorCountDelta(now, align_diag.align_count);
  const int32_t move_limit = torque_diag.move_limit_counts;
  const float iq_amplitude = torque_diag.high_current ?
      MOTOR_HIGH_IQ_PROBE_A : TORQUE_PROBE_IQ_A;
  int32_t rel = 0;
  if (motorAbsCount(from_align) > torque_diag.peak_abs_counts)
    torque_diag.peak_abs_counts = motorAbsCount(from_align);

  /* Check absolute excursion around the alignment origin even before
   * the relaxed zero is established; after settling use relaxed zero. */
  if (motorAbsCount(from_align) > move_limit)
  {
    torque_diag.result = 3U;
    torque_diag.cut_tick = t;
    torque_diag.cut_count = now;
    torque_diag.cut_rel_count = from_align;
    torque_diag.end_count = now;
    align_gate_status = 3U;
    power_output_armed = false;
    pwmDisableOutput();
    motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
    return;
  }

  const float theta = encoderGetElectricalAngle();
  if (!isfinite(theta))
  {
    motorSetFault(MOTOR_FAULT_ENCODER_INVALID);
    return;
  }

  float id_ref = 0.0f;
  float iq_ref = 0.0f;
  if (t < POS_T_ID_ZERO)
  {
    /* Identical Id release to the existing position test. */
    id_ref = ALIGN_ID * (1.0f - (float)t / (float)POSITION_ID_DECAY_TICKS);
  }
  else if (t < TORQUE_PROBE_START_TICK)
  {
    /* Id=Iq=0: allow shaft to relax before fixing zero. */
  }
  else
  {
    if (t == TORQUE_PROBE_START_TICK)
    {
      torque_diag.zero_count = now;
      position_zero_count = now;
    }
    rel = motorCountDelta(now, torque_diag.zero_count);
    if (motorAbsCount(rel) > torque_diag.peak_abs_counts)
      torque_diag.peak_abs_counts = motorAbsCount(rel);
    if (motorAbsCount(rel) > move_limit)
    {
      torque_diag.result = 3U;
      torque_diag.cut_tick = t;
      torque_diag.cut_count = now;
      torque_diag.cut_rel_count = rel;
      torque_diag.end_count = now;
      align_gate_status = 3U;
      power_output_armed = false;
      pwmDisableOutput();
      motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
      return;
    }

    const uint32_t u = t - TORQUE_PROBE_START_TICK;
    float level = 0.0f;
    if (u < TORQUE_PROBE_RAMP_TICKS)
      level = (float)(u + 1U) / (float)TORQUE_PROBE_RAMP_TICKS;
    else if (u < TORQUE_PROBE_RAMP_TICKS + TORQUE_PROBE_HOLD_TICKS)
      level = 1.0f;
    else
    {
      const uint32_t decay_tick =
          u - TORQUE_PROBE_RAMP_TICKS - TORQUE_PROBE_HOLD_TICKS;
      level = 1.0f - (float)(decay_tick + 1U) /
                      (float)TORQUE_PROBE_DECAY_TICKS;
      if (level < 0.0f) level = 0.0f;
    }
    iq_ref = (float)torque_probe_direction * iq_amplitude * level;
  }

  motorCurrentLoop(id_ref, iq_ref, theta);
  if (motor_state == MOTOR_STATE_FAULT) return;

  if (t == POS_T_ID_ZERO)
    motorCaptureTorqueSample(0U, t, now);
  if (t == TORQUE_PROBE_START_TICK)
    motorCaptureTorqueSample(1U, t, now);
  if (t == TORQUE_PROBE_START_TICK + TORQUE_PROBE_RAMP_TICKS / 2U)
    motorCaptureTorqueSample(2U, t, now);
  if (t == TORQUE_PROBE_START_TICK + TORQUE_PROBE_RAMP_TICKS)
    motorCaptureTorqueSample(3U, t, now);
  if (t == TORQUE_PROBE_START_TICK + TORQUE_PROBE_RAMP_TICKS + TORQUE_PROBE_HOLD_TICKS / 2U)
    motorCaptureTorqueSample(4U, t, now);
  if (t == TORQUE_PROBE_START_TICK + TORQUE_PROBE_RAMP_TICKS + TORQUE_PROBE_HOLD_TICKS - 1U)
    motorCaptureTorqueSample(5U, t, now);
  if (t == TORQUE_PROBE_START_TICK + TORQUE_PROBE_RAMP_TICKS + TORQUE_PROBE_HOLD_TICKS + TORQUE_PROBE_DECAY_TICKS / 2U)
    motorCaptureTorqueSample(6U, t, now);
  if (t == TORQUE_PROBE_TOTAL_TICKS - 1U)
    motorCaptureTorqueSample(7U, t, now);

  closed_loop_ticks++;
  if (closed_loop_ticks >= TORQUE_PROBE_TOTAL_TICKS)
  {
    torque_diag.result = 1U;
    torque_diag.end_count = (int32_t)encoderGetCount();
    align_gate_status = 1U;
    power_output_armed = false;
    pwmDisableOutput();
    motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
  }
}

/* Bounded ONE-REV encoder-feedback FOC current test (G/H).
 * Every 20kHz control tick integrates the modulo-4000 TIM2 encoder
 * difference, so one turn does NOT look like zero displacement.
 * No speed controller or pre-existing encoder speed tracker is used.
 * The newly measured 10ms mechanical speed is only a guard and log.
 * Moving current is capped by the user's existing measured-current,
 * VBUS, BKIN and voltage-vector safety checks in motorCurrentLoop().
 */
static const uint32_t spin_probe_sample_ticks[MOTOR_SPIN_PROBE_SAMPLE_COUNT] = {
  0U,500U,1000U,1500U,1999U,2999U,3999U,5999U,
  7999U,9999U,11999U,13999U,15999U,17999U,19999U,21999U
};

static void motorCaptureSpinSample(uint32_t u, int32_t raw)
{
  if (spin_diag.sample_count >= MOTOR_SPIN_PROBE_SAMPLE_COUNT) return;
  volatile motor_spin_probe_sample_t *s =
      &spin_diag.samples[spin_diag.sample_count];
  s->tick = u;
  s->raw_count = raw;
  s->unwrapped_count = spin_unwrapped_count;
  s->rpm_meas = spin_rpm_10ms;
  s->theta_e = motor_monitor.theta_e;
  s->id_ref = motor_monitor.id_ref;
  s->id_meas = motor_monitor.id_meas;
  s->iq_ref = motor_monitor.iq_ref;
  s->iq_meas = motor_monitor.iq_meas;
  s->vd = motor_monitor.vd_cmd;
  s->vq = motor_monitor.vq_cmd;
  spin_diag.sample_count++;
}

static void motorFinishSpinProbe(uint8_t result, uint32_t t, int32_t raw)
{
  spin_diag.result = result;
  spin_diag.stop_tick = t;
  spin_diag.stop_raw_count = raw;
  spin_diag.stop_unwrapped_count = spin_unwrapped_count;
  spin_diag.end_count = raw;
  spin_diag.total_counts = spin_unwrapped_count;
  spin_diag.last_rpm = spin_rpm_10ms;
  align_gate_status = result;
  power_output_armed = false;
  pwmDisableOutput();
  motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
}

static void motorCommissionSpinProbe(void)
{
  const uint32_t t = closed_loop_ticks;
  const int32_t raw = (int32_t)encoderGetCount();
  const int32_t from_align = motorCountDelta(raw, spin_diag.align_count);
  const float theta = encoderGetElectricalAngle();
  if (!isfinite(theta))
  {
    motorSetFault(MOTOR_FAULT_ENCODER_INVALID);
    return;
  }

  /* Id release + zero-settle: no acceleration command yet. */
  if (t < MOTOR_SPIN_PROBE_START_TICK)
  {
    if (motorAbsCount(from_align) > MOTOR_HIGH_IQ_PROBE_MAX_MOVE_COUNTS)
    {
      motorFinishSpinProbe(7U, t, raw);
      return;
    }
    float id_ref = 0.0f;
    if (t < POS_T_ID_ZERO)
      id_ref = ALIGN_ID * (1.0f - (float)t / (float)POSITION_ID_DECAY_TICKS);
    motorCurrentLoop(id_ref, 0.0f, theta);
    if (motor_state == MOTOR_STATE_FAULT) return;
    closed_loop_ticks++;
    return;
  }

  const uint32_t u = t - MOTOR_SPIN_PROBE_START_TICK;
  if (u == 0U)
  {
    spin_diag.zero_count = raw;
    spin_prev_raw_count = raw;
    spin_unwrapped_count = 0;
    spin_10ms_prev_count = 0;
    spin_rpm_10ms = 0.0f;
  }
  else
  {
    /* The value is unambiguous at 20kHz unless motion exceeds
       half a mechanical revolution BETWEEN consecutive ADC samples. */
    const int32_t delta = motorCountDelta(raw, spin_prev_raw_count);
    spin_unwrapped_count += delta;
    spin_prev_raw_count = raw;
  }

  const int32_t abs_move = motorAbsCount(spin_unwrapped_count);
  if (abs_move > spin_diag.peak_abs_counts)
    spin_diag.peak_abs_counts = abs_move;
  /* On the verified hardware +Iq causes negative raw TIM2 movement. */
  const int32_t forward = -((int32_t)spin_probe_direction) * spin_unwrapped_count;
  if (forward > spin_diag.maximum_forward_counts)
    spin_diag.maximum_forward_counts = forward;

  if (u != 0U && u % 200U == 0U) /* 10 ms speed estimate */
  {
    const int32_t step = spin_unwrapped_count - spin_10ms_prev_count;
    spin_10ms_prev_count = spin_unwrapped_count;
    /* 20kHz / 200 samples = 100 updates/s; 4000 x4 counts/rev. */
    spin_rpm_10ms = (float)step * (60.0f * 100.0f / (float)ENCODER_MAX_COUNT);
    if (fabsf(spin_rpm_10ms) > spin_diag.peak_abs_rpm)
      spin_diag.peak_abs_rpm = fabsf(spin_rpm_10ms);
  }

  /* All exit paths turn PWM off immediately, even when RPM != 0. */
  if (abs_move >= MOTOR_SPIN_PROBE_MAX_TRAVEL_COUNTS)
  { motorFinishSpinProbe(3U, t, raw); return; }
  if (fabsf(spin_rpm_10ms) >= MOTOR_SPIN_PROBE_MAX_RPM)
  { motorFinishSpinProbe(4U, t, raw); return; }
  if (forward < -MOTOR_SPIN_PROBE_WRONG_COUNTS)
  { motorFinishSpinProbe(5U, t, raw); return; }
  if (u >= MOTOR_SPIN_PROBE_STALL_TICKS && abs_move < MOTOR_SPIN_PROBE_STALL_COUNTS)
  { motorFinishSpinProbe(6U, t, raw); return; }

  float level = 0.0f;
  if (u < MOTOR_SPIN_PROBE_RAMP_TICKS)
    level = (float)(u + 1U) / (float)MOTOR_SPIN_PROBE_RAMP_TICKS;
  else if (u < MOTOR_SPIN_PROBE_RAMP_TICKS + MOTOR_SPIN_PROBE_HOLD_TICKS)
    level = 1.0f;
  else if (u < MOTOR_SPIN_PROBE_RAMP_TICKS + MOTOR_SPIN_PROBE_HOLD_TICKS + MOTOR_SPIN_PROBE_DECAY_TICKS)
  {
    const uint32_t d = u - MOTOR_SPIN_PROBE_RAMP_TICKS - MOTOR_SPIN_PROBE_HOLD_TICKS;
    level = 1.0f - (float)(d + 1U) / (float)MOTOR_SPIN_PROBE_DECAY_TICKS;
    if (level < 0.0f) level = 0.0f;
  }
  else
  { motorFinishSpinProbe(1U, t, raw); return; }

  motorCurrentLoop(0.0f, (float)spin_probe_direction * MOTOR_SPIN_PROBE_IQ_A * level, theta);
  if (motor_state == MOTOR_STATE_FAULT) return;

  if (spin_diag.sample_count < MOTOR_SPIN_PROBE_SAMPLE_COUNT &&
      u == spin_probe_sample_ticks[spin_diag.sample_count])
    motorCaptureSpinSample(u, raw);

  closed_loop_ticks++;
  if (closed_loop_ticks >= MOTOR_SPIN_PROBE_TOTAL_TICKS)
    motorFinishSpinProbe(1U, closed_loop_ticks, (int32_t)encoderGetCount());
}

/* K/L: bounded encoder-feedback speed control, current PI at 20kHz.
 * Distinct STARTUP Iq ramp is independent of the speed PI proportional gain.
 * HANDOFF tracks Iq with a slew limiter and limited integral precharge.
 * RUN uses low-gain speed PI; protections are checked in all phases.
 * The 10ms RPM estimate is independent of the legacy UART speed field.
 */
static const uint32_t speed_probe_sample_ticks[MOTOR_SPEED_PI_SAMPLE_COUNT] = {
    0U, 200U, 400U, 600U, 800U, 1000U, 1200U, 1400U, 1600U,
    1800U, 2000U, 2200U, 2400U, 2600U, 2800U, 3000U,
    3500U, 4000U, 5000U, 6000U, 8000U, 10000U, 12000U,
    14000U, 16000U, 18000U, 20000U, 24000U, 28000U, 29999U
};

static void motorCaptureSpeedPiSample(uint32_t u, int32_t raw)
{
  if (speed_pi_diag.sample_count >= MOTOR_SPEED_PI_SAMPLE_COUNT) return;
  volatile motor_speed_pi_sample_t *s =
      &speed_pi_diag.samples[speed_pi_diag.sample_count];
  s->tick = u;
  s->phase = (uint8_t)speed_probe_phase;
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
  align_gate_status = result;
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
    if (motorAbsCount(from_align) > MOTOR_HIGH_IQ_PROBE_MAX_MOVE_COUNTS)
    {
      motorFinishSpeedPiProbe(7U, t, raw);
      return;
    }
    float id_ref = 0.0f;
    if (t < POS_T_ID_ZERO)
      id_ref = ALIGN_ID * (1.0f - (float)t / (float)POSITION_ID_DECAY_TICKS);
    motorCurrentLoop(id_ref, 0.0f, theta);
    if (motor_state == MOTOR_STATE_FAULT) return;
    closed_loop_ticks++;
    return;
  }

  const uint32_t u = t - MOTOR_SPEED_PI_START_TICK;
  if (u == 0U)
  {
    speed_pi_diag.zero_count = raw;
    speed_probe_prev_raw = raw;
    speed_probe_unwrapped = 0;
    speed_probe_prev_10ms = 0;
    speed_probe_raw_rpm = 0.0f;
    speed_probe_filtered_rpm = 0.0f;
    speed_probe_target_rpm = 0.0f;
    speed_probe_phase = MOTOR_SPEED_PHASE_STARTUP;
    speed_probe_phase_start_tick = 0U;
    speed_probe_start_confirm = 0U;
    speed_probe_run_stall_steps = 0U;
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

    if (speed_probe_phase == MOTOR_SPEED_PHASE_STARTUP)
    {
      const float startup_iq = fminf(MOTOR_SPEED_STARTUP_IQ_A, speed_probe_iq_limit);
      const float ramp = clampFloat((float)u / (float)MOTOR_SPEED_STARTUP_RAMP_TICKS,
                                    0.0f, 1.0f);
      const float measured_along = (float)speed_probe_direction * speed_probe_raw_rpm;
      float desired = (float)speed_probe_direction * startup_iq * ramp;
      /* Avoid holding +3A into a rapidly accelerating rotor. Do not
         apply negative torque in startup. */
      if (measured_along >= MOTOR_SPEED_STARTUP_STOP_RPM) desired = 0.0f;
      else if (measured_along > MOTOR_SPEED_STARTUP_SOFT_RPM)
        desired *= (MOTOR_SPEED_STARTUP_STOP_RPM-measured_along) /
                   (MOTOR_SPEED_STARTUP_STOP_RPM-MOTOR_SPEED_STARTUP_SOFT_RPM);
      const float startup_step = ((float)speed_probe_direction * desired <
          (float)speed_probe_direction * current_iq_ref) ?
          MOTOR_SPEED_IQ_DECEL_SLEW_A_PER_S * 0.010f : iq_step;
      current_iq_ref += clampFloat(desired - current_iq_ref,
                                  -startup_step, startup_step);

      /* Require both physical displacement and two signed 10ms speed reads. */
      if (forward >= MOTOR_SPEED_START_DETECT_COUNTS &&
          (float)speed_probe_direction * speed_probe_raw_rpm >= MOTOR_SPEED_START_DETECT_RPM)
      {
        if (speed_probe_start_confirm < MOTOR_SPEED_START_CONFIRM_STEPS)
          speed_probe_start_confirm++;
      }
      else
        speed_probe_start_confirm = 0U;

      /* A large, correctly signed displacement with meaningful speed is
         already decisive; do not keep boosting solely to get 2 samples. */
      const bool decisive_start =
          (forward >= 100 &&
           (float)speed_probe_direction * speed_probe_filtered_rpm >= 60.0f);
      if (speed_probe_start_confirm >= MOTOR_SPEED_START_CONFIRM_STEPS ||
          decisive_start)
      {
        speed_probe_phase = MOTOR_SPEED_PHASE_HANDOFF;
        speed_probe_phase_start_tick = u;
        speed_pi_diag.startup_end_tick = u;
        speed_pi_diag.rpm_at_startup_end = speed_probe_filtered_rpm;
        speed_pi_diag.iq_at_startup_end = current_iq_ref;

        /* Initial target tracks the speed already reached, within goal. */
        const float speed_abs = clampFloat(
            (float)speed_probe_direction * speed_probe_filtered_rpm,
            MOTOR_SPEED_HANDOFF_MIN_RPM, MOTOR_SPEED_PI_GOAL_RPM);
        speed_probe_target_rpm = (float)speed_probe_direction * speed_abs;
        pidReset(&pi_spd);
        /* Bumpless PI initialization, with a bounded integral estimate.
           A separate Iq slew limit ensures NO step at the phase change. */
        const float e = speed_probe_target_rpm - speed_probe_filtered_rpm;
        pi_spd.integral = clampFloat(current_iq_ref - pi_spd.kp * e,
                                      pi_spd.integral_min, pi_spd.integral_max);
      }
      else if (u >= MOTOR_SPEED_STARTUP_TIMEOUT_TICKS)
      {
        motorFinishSpeedPiProbe(8U, t, raw); /* no sustained start */
        return;
      }
    }
    else /* HANDOFF or RUN: speed PI is separate from the startup torque. */
    {
      const float increment = MOTOR_SPEED_PI_RAMP_RPM_PER_S * 0.010f;
      speed_probe_target_rpm += clampFloat(goal - speed_probe_target_rpm,
                                            -increment, increment);
      const float pi_request = piControllerAntiWindup(&pi_spd,
          speed_probe_target_rpm, speed_probe_filtered_rpm, 0.010f);
      const float bounded_request = clampFloat(pi_request,
                                      -speed_probe_iq_limit, speed_probe_iq_limit);
      const bool removing_drive =
          ((float)speed_probe_direction * bounded_request <
           (float)speed_probe_direction * current_iq_ref);
      const float step = removing_drive ?
          MOTOR_SPEED_IQ_DECEL_SLEW_A_PER_S * 0.010f : iq_step;
      current_iq_ref += clampFloat(bounded_request - current_iq_ref,
                                   -step, step);
      /* Correct integral for a current command not yet reached due to slew. */
      pi_spd.integral = clampFloat(
          pi_spd.integral + MOTOR_SPEED_INTEGRAL_TRACK_GAIN *
          (current_iq_ref - bounded_request),
          pi_spd.integral_min, pi_spd.integral_max);

      if (speed_probe_phase == MOTOR_SPEED_PHASE_HANDOFF &&
          (u - speed_probe_phase_start_tick) >= MOTOR_SPEED_HANDOFF_TICKS &&
          fabsf(bounded_request - current_iq_ref) <= MOTOR_SPEED_HANDOFF_EXIT_IQ_A)
      {
        speed_probe_phase = MOTOR_SPEED_PHASE_RUN;
        speed_pi_diag.handoff_end_tick = u;
      }

      if (speed_probe_phase == MOTOR_SPEED_PHASE_RUN)
      {
        if ((float)speed_probe_direction * speed_probe_filtered_rpm < 5.0f)
        {
          if (speed_probe_run_stall_steps < MOTOR_SPEED_RUN_STALL_STEPS)
            speed_probe_run_stall_steps++;
        }
        else
          speed_probe_run_stall_steps = 0U;
        if (speed_probe_run_stall_steps >= MOTOR_SPEED_RUN_STALL_STEPS)
        {
          motorFinishSpeedPiProbe(5U, t, raw);
          return;
        }
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
  { motorFinishSpeedPiProbe(1U, t, raw); return; }

  motorCurrentLoop(0.0f, current_iq_ref, theta);
  if (motor_state == MOTOR_STATE_FAULT) return;

  if (speed_pi_diag.sample_count < MOTOR_SPEED_PI_SAMPLE_COUNT &&
      u == speed_probe_sample_ticks[speed_pi_diag.sample_count])
    motorCaptureSpeedPiSample(u, raw);

  closed_loop_ticks++;
}

/* O: One selected offset delta and ONE selected +/-Iq pulse per run.
 * Offset is injected into the angle supplied to BOTH Park and inverse Park;
 * encoderSetElectricalOffset() is NOT called here. It preserves the base
 * alignment calibration so no diagnostic result overwrites it.
 * With Id=Iq=0, reset current PI at the start of 150 ms zero-current
 * settling, then change the frame. This avoids switching the d/q frame
 * while the alignment Id current is still actively applied.
 */
static void motorCommissionOffsetProbe(void)
{
  const uint32_t t = closed_loop_ticks;
  const int32_t now = (int32_t)encoderGetCount();
  const int32_t from_align = motorCountDelta(now, offset_diag.align_count);
  int32_t rel = 0;
  const int32_t abs_align = motorAbsCount(from_align);
  if (abs_align > offset_diag.peak_abs_counts)
    offset_diag.peak_abs_counts = abs_align;
  if (abs_align > OFFSET_PROBE_MAX_MOVE_COUNTS)
  {
    offset_diag.result = 3U;
    offset_diag.end_count = now;
    align_gate_status = 3U;
    power_output_armed = false;
    pwmDisableOutput();
    motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
    return;
  }

  float theta = encoderGetElectricalAngle();
  if (!isfinite(theta))
  {
    motorSetFault(MOTOR_FAULT_ENCODER_INVALID);
    return;
  }

  float id_ref = 0.0f;
  float iq_ref = 0.0f;
  if (t < POS_T_ID_ZERO)
  {
    id_ref = ALIGN_ID * (1.0f - (float)t / (float)POSITION_ID_DECAY_TICKS);
  }
  else
  {
    if (t == POS_T_ID_ZERO)
    {
      /* Exactly at the beginning of zero-current settling. */
      pidReset(&pi_id);
      pidReset(&pi_iq);
    }
    theta = wrapFloat(theta +
        (float)offset_probe_delta_deg * (PI / 180.0f), 0.0f, 2.0f * PI);
  }

  if (t >= OFFSET_PROBE_START_TICK)
  {
    if (t == OFFSET_PROBE_START_TICK)
      offset_diag.zero_count = now;
    rel = motorCountDelta(now, offset_diag.zero_count);
    const int32_t abs_rel = motorAbsCount(rel);
    if (abs_rel > offset_diag.peak_abs_counts)
      offset_diag.peak_abs_counts = abs_rel;
    if (abs_rel > OFFSET_PROBE_MAX_MOVE_COUNTS)
    {
      offset_diag.result = 3U;
      offset_diag.end_count = now;
      align_gate_status = 3U;
      power_output_armed = false;
      pwmDisableOutput();
      motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
      return;
    }

    const uint32_t u = t - OFFSET_PROBE_START_TICK;
    float level = 0.0f;
    if (u < TORQUE_PROBE_RAMP_TICKS)
      level = (float)(u + 1U) / (float)TORQUE_PROBE_RAMP_TICKS;
    else if (u < TORQUE_PROBE_RAMP_TICKS + TORQUE_PROBE_HOLD_TICKS)
      level = 1.0f;
    else
    {
      const uint32_t decay_tick =
          u - TORQUE_PROBE_RAMP_TICKS - TORQUE_PROBE_HOLD_TICKS;
      level = 1.0f - (float)(decay_tick + 1U) /
          (float)TORQUE_PROBE_DECAY_TICKS;
      if (level < 0.0f) level = 0.0f;
    }
    iq_ref = (float)offset_probe_direction * OFFSET_PROBE_IQ_A * level;
  }

  motorCurrentLoop(id_ref, iq_ref, theta);
  if (motor_state == MOTOR_STATE_FAULT) return;

  /* Relative count before ZERO is not used to judge the pulse. */
  if      (t == POS_T_ID_ZERO) motorCaptureOffsetSample(0U, t, now);
  else if (t == OFFSET_PROBE_START_TICK - 1U)
    motorCaptureOffsetSample(1U, t, now);
  else if (t == OFFSET_PROBE_START_TICK)
    motorCaptureOffsetSample(2U, t, now);
  else if (t == OFFSET_PROBE_START_TICK + 400U - 1U)
    motorCaptureOffsetSample(3U, t, now);  /* +20 ms */
  else if (t == OFFSET_PROBE_START_TICK + 1000U - 1U)
    motorCaptureOffsetSample(4U, t, now);  /* +50 ms */
  else if (t == OFFSET_PROBE_START_TICK + TORQUE_PROBE_RAMP_TICKS - 1U)
    motorCaptureOffsetSample(5U, t, now);  /* +100 ms */
  else if (t == OFFSET_PROBE_START_TICK + TORQUE_PROBE_RAMP_TICKS + TORQUE_PROBE_HOLD_TICKS / 2U - 1U)
    motorCaptureOffsetSample(6U, t, now);  /* +175 ms */
  else if (t == OFFSET_PROBE_START_TICK + TORQUE_PROBE_RAMP_TICKS + TORQUE_PROBE_HOLD_TICKS - 1U)
    motorCaptureOffsetSample(7U, t, now);  /* +250 ms */
  else if (t == OFFSET_PROBE_START_TICK + TORQUE_PROBE_RAMP_TICKS + TORQUE_PROBE_HOLD_TICKS + TORQUE_PROBE_DECAY_TICKS / 2U - 1U)
    motorCaptureOffsetSample(8U, t, now);  /* +300 ms */
  else if (t == OFFSET_PROBE_TOTAL_TICKS - 1U)
    motorCaptureOffsetSample(9U, t, now);  /* +350 ms */

  closed_loop_ticks++;
  if (closed_loop_ticks >= OFFSET_PROBE_TOTAL_TICKS)
  {
    offset_diag.result = 1U;
    offset_diag.end_count = (int32_t)encoderGetCount();
    align_gate_status = 1U;
    power_output_armed = false;
    pwmDisableOutput();
    motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
  }
}

/* V diagnostic: CURRENT-STEP HYSTERESIS test.
 * The user's def.h settings are kept untouched:
 *   VECTOR_PROBE_ID_A         = 3.5A (already set by user)
 *   VECTOR_PROBE_ANGLE_DEG    = 90deg
 *   VECTOR_PROBE_MAX_MOVE_COUNTS = 250 (already set by user)
 *   VECTOR_PROBE_ANGLE_HOLD_TICKS = 12000 (600ms, already set)
 * The test is strictly OPEN LOOP with respect to rotor position.
 * All stages retain the 20kHz ADC-driven PI and existing safety checks.
 * Extra ~700ms -> 2s alignment + 2.3s test = ~4.3s (<5.5s watchdog).
 */
#define VECTOR_HYST_LOW_ID_A                  2.50f
#define VECTOR_HYST_ID_DOWN_TICKS             2000U    /* 100ms, high -> 2.5A */
#define VECTOR_HYST_LOW_HOLD_TICKS            12000U   /* 600ms at +90deg, 2.5A */
#define VECTOR_HYST_T_ID_DOWN_END             (VECTOR_T_HOLD_END + VECTOR_HYST_ID_DOWN_TICKS)
#define VECTOR_HYST_T_LOW_HOLD_END            (VECTOR_HYST_T_ID_DOWN_END + VECTOR_HYST_LOW_HOLD_TICKS)
#define VECTOR_HYST_T_RETURN_END              (VECTOR_HYST_T_LOW_HOLD_END + VECTOR_PROBE_ANGLE_RAMP_TICKS)
#define VECTOR_HYST_T_HOME_END                (VECTOR_HYST_T_RETURN_END + VECTOR_PROBE_HOME_HOLD_TICKS)
#define VECTOR_HYST_T_TOTAL                   (VECTOR_HYST_T_HOME_END + VECTOR_PROBE_ID_OFF_TICKS)

static void motorCommissionVectorProbe(void)
{
  const uint32_t t = closed_loop_ticks;
  const int32_t now = (int32_t)encoderGetCount();
  const int32_t delta = motorCountDelta(now, vector_diag.align_count);
  const int32_t mag = motorAbsCount(delta);
  if (mag > vector_diag.peak_abs_counts)
    vector_diag.peak_abs_counts = mag;

  const float theta_goal = VECTOR_PROBE_ANGLE_DEG * (PI / 180.0f);
  float theta_cmd = 0.0f;
  float id_ref = VECTOR_PROBE_ID_A;

  if (t < VECTOR_T_UP_START)
  {
    /* 2A alignment -> 3.5A smoothly at fixed stator theta=0. */
    const float u = (float)(t + 1U) / (float)VECTOR_PROBE_ID_RAMP_TICKS;
    id_ref = ALIGN_ID + (VECTOR_PROBE_ID_A - ALIGN_ID) * u;
  }
  else if (t < VECTOR_T_UP_END)
  {
    /* 3.5A: commanded theta 0 -> +90deg. */
    const float u = (float)(t - VECTOR_T_UP_START + 1U) /
                    (float)VECTOR_PROBE_ANGLE_RAMP_TICKS;
    theta_cmd = theta_goal * u;
  }
  else if (t < VECTOR_T_HOLD_END)
  {
    /* 3.5A +90deg: allow rotor response/settling. */
    theta_cmd = theta_goal;
  }
  else if (t < VECTOR_HYST_T_ID_DOWN_END)
  {
    /* Crucial comparison: +90deg is held CONSTANT while Id decreases. */
    const float u = (float)(t - VECTOR_T_HOLD_END + 1U) /
                    (float)VECTOR_HYST_ID_DOWN_TICKS;
    theta_cmd = theta_goal;
    id_ref = VECTOR_PROBE_ID_A +
             (VECTOR_HYST_LOW_ID_A - VECTOR_PROBE_ID_A) * u;
  }
  else if (t < VECTOR_HYST_T_LOW_HOLD_END)
  {
    /* Same +90deg, now at 2.5A for 600ms. */
    theta_cmd = theta_goal;
    id_ref = VECTOR_HYST_LOW_ID_A;
  }
  else if (t < VECTOR_HYST_T_RETURN_END)
  {
    /* Return the stator field to zero while holding 2.5A. */
    const float u = (float)(t - VECTOR_HYST_T_LOW_HOLD_END + 1U) /
                    (float)VECTOR_PROBE_ANGLE_RAMP_TICKS;
    theta_cmd = theta_goal * (1.0f - u);
    id_ref = VECTOR_HYST_LOW_ID_A;
  }
  else if (t < VECTOR_HYST_T_HOME_END)
  {
    theta_cmd = 0.0f;
    id_ref = VECTOR_HYST_LOW_ID_A;
  }
  else
  {
    /* Zero-angle 2.5A -> 0A; automatic PWM OFF at the end. */
    const float u = (float)(t - VECTOR_HYST_T_HOME_END + 1U) /
                    (float)VECTOR_PROBE_ID_OFF_TICKS;
    id_ref = VECTOR_HYST_LOW_ID_A * (1.0f - u);
    if (id_ref < 0.0f) id_ref = 0.0f;
  }

  /* Raw TIM2 excursion check EVERY ISR, before issuing any new PWM command.
   * This preserves the user's VECTOR_PROBE_MAX_MOVE_COUNTS=250 limit.
   * At a cutoff record the planned theta command, not a fictitious result. */
  if (mag > VECTOR_PROBE_MAX_MOVE_COUNTS)
  {
    vector_diag.result = 3U;
    vector_diag.end_count = now;
    vector_diag.cut_tick = t;
    vector_diag.cut_theta_cmd = theta_cmd;
    vector_diag.cut_encoder_count = now;
    align_gate_status = 3U;
    power_output_armed = false;
    pwmDisableOutput();
    motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
    return;
  }

  /* Encoder electrical angle is NOT used for Park or inverse-Park here. */
  motorCurrentLoop(id_ref, 0.0f, theta_cmd);
  if (motor_state == MOTOR_STATE_FAULT) return;

  /* Precisely 16 samples: 16-bit mask fits including index 15. */
  if      (t == 0U)
    motorCaptureVectorSample(0U, t, now, theta_cmd);
  else if (t == VECTOR_T_UP_START - 1U)
    motorCaptureVectorSample(1U, t, now, theta_cmd);
  else if (t == VECTOR_T_UP_START + VECTOR_PROBE_ANGLE_RAMP_TICKS / 2U - 1U)
    motorCaptureVectorSample(2U, t, now, theta_cmd);
  else if (t == VECTOR_T_UP_END - 1U)
    motorCaptureVectorSample(3U, t, now, theta_cmd);
  else if (t == VECTOR_T_UP_END + VECTOR_PROBE_ANGLE_HOLD_TICKS / 2U - 1U)
    motorCaptureVectorSample(4U, t, now, theta_cmd);
  else if (t == VECTOR_T_HOLD_END - 1U)
    motorCaptureVectorSample(5U, t, now, theta_cmd);
  else if (t == VECTOR_T_HOLD_END + VECTOR_HYST_ID_DOWN_TICKS / 2U - 1U)
    motorCaptureVectorSample(6U, t, now, theta_cmd);
  else if (t == VECTOR_HYST_T_ID_DOWN_END - 1U)
    motorCaptureVectorSample(7U, t, now, theta_cmd);
  else if (t == VECTOR_HYST_T_ID_DOWN_END + VECTOR_HYST_LOW_HOLD_TICKS / 4U - 1U)
    motorCaptureVectorSample(8U, t, now, theta_cmd);
  else if (t == VECTOR_HYST_T_ID_DOWN_END + VECTOR_HYST_LOW_HOLD_TICKS / 2U - 1U)
    motorCaptureVectorSample(9U, t, now, theta_cmd);
  else if (t == VECTOR_HYST_T_LOW_HOLD_END - 1U)
    motorCaptureVectorSample(10U, t, now, theta_cmd);
  else if (t == VECTOR_HYST_T_LOW_HOLD_END + VECTOR_PROBE_ANGLE_RAMP_TICKS / 2U - 1U)
    motorCaptureVectorSample(11U, t, now, theta_cmd);
  else if (t == VECTOR_HYST_T_RETURN_END - 1U)
    motorCaptureVectorSample(12U, t, now, theta_cmd);
  else if (t == VECTOR_HYST_T_HOME_END - 1U)
    motorCaptureVectorSample(13U, t, now, theta_cmd);
  else if (t == VECTOR_HYST_T_HOME_END + VECTOR_PROBE_ID_OFF_TICKS / 2U - 1U)
    motorCaptureVectorSample(14U, t, now, theta_cmd);
  else if (t == VECTOR_HYST_T_TOTAL - 1U)
    motorCaptureVectorSample(15U, t, now, theta_cmd);

  closed_loop_ticks++;
  if (closed_loop_ticks >= VECTOR_HYST_T_TOTAL)
  {
    vector_diag.result = 1U; /* Sequence completed, NOT calibrated. */
    vector_diag.end_count = (int32_t)encoderGetCount();
    align_gate_status = 1U;
    power_output_armed = false;
    pwmDisableOutput();
    motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
  }
}

/* A single position-feedback commissioning sequence, scheduled at 20 kHz.
 * After zero-current settling, +8 -> 0 -> -8 -> 0 raw encoder counts.
 * Position PD updates at 1 kHz; current PI stays at 20 kHz.
 * Critical safety checks use unfiltered TIM2 counts at 20 kHz.
 * Controlled signed raw counts are relative to the actual relaxed position.
 * NOTE: polarity is deliberately inverted based on measured torque response:
 * +Iq caused negative encoder count (confirmed on two consecutive runs).
 */
static float motorPositionSegmentRef(uint32_t t)
{
  const uint32_t seg = (t - POS_T_PLUS_START) / POSITION_SEGMENT_TICKS;
  const uint32_t local_t = (t - POS_T_PLUS_START) % POSITION_SEGMENT_TICKS;
  float u = (float)local_t / (float)POSITION_TARGET_RAMP_TICKS;
  if (u > 1.0f) u = 1.0f;
  switch (seg)
  {
    case 0U: return POSITION_TARGET_COUNTS * u;
    case 1U: return POSITION_TARGET_COUNTS * (1.0f - u);
    case 2U: return -POSITION_TARGET_COUNTS * u;
    default: return -POSITION_TARGET_COUNTS * (1.0f - u);
  }
}

static void motorCommissionPositionTest(void)
{
  const uint32_t t = closed_loop_ticks;
  const int32_t now = (int32_t)encoderGetCount();
  /* Keep the old unfiltered path as telemetry: a 0->1->0 dither adds 2.
   * It is not a reliable mechanical travel / trip criterion at 20 kHz. */
  const int32_t raw_step = motorCountDelta(now, position_raw_prev_count);
  position_raw_prev_count = now;
  position_raw_accumulated_travel += motorAbsCount(raw_step);
  align_diag.raw_accumulated_travel_counts = position_raw_accumulated_travel;

  /* Diagnostic 1 kHz travel, accumulated without ever tripping on it. */
  if ((t % POSITION_CTRL_DIVIDER_TICKS) == 0U)
  {
    const int32_t step = motorCountDelta(now, position_prev_count);
    const int32_t magnitude = motorAbsCount(step);
    if (magnitude >= POSITION_TRAVEL_DEADBAND_COUNTS)
    {
      position_accumulated_travel += magnitude;
      position_prev_count = now;
    }

    /* Is the raw encoder noisy even at Id=Iq=0 and MOE still enabled?
     * This stage lasts 0.15s, before the first position target.
     * It does NOT prove whether motion is mechanical or electrical. */
    if (t == POS_T_ID_ZERO)
    {
      zero_iq_start_count = now;
      zero_iq_prev_count = now;
    }
    if (t >= POS_T_ID_ZERO && t < POS_T_PLUS_START)
    {
      const int32_t zstep = motorAbsCount(motorCountDelta(now, zero_iq_prev_count));
      align_diag.zero_iq_variation_1khz += zstep;
      if (zstep > align_diag.zero_iq_peak_1ms_step)
        align_diag.zero_iq_peak_1ms_step = zstep;
      zero_iq_prev_count = now;
      align_diag.zero_iq_net_count = motorCountDelta(now, zero_iq_start_count);
    }
    else if (t >= POS_T_PLUS_START && t < POS_T_HOME2_END)
    {
      if (t == POS_T_PLUS_START) drive_prev_count = now;
      const int32_t dstep = motorAbsCount(motorCountDelta(now, drive_prev_count));
      align_diag.drive_variation_1khz += dstep;
      if (dstep > align_diag.drive_peak_1ms_step)
        align_diag.drive_peak_1ms_step = dstep;
      drive_prev_count = now;
    }
  }
  align_diag.accumulated_travel_counts = position_accumulated_travel;

  /* Safety: check instantaneous excursion on EVERY 20 kHz update.
   * Do NOT trip on total variation (sum(abs(delta))): encoder dithering
   * or genuine small servo oscillation can drive that sum arbitrarily
   * high despite remaining inside the tightly bounded position envelope.
   * The 1 kHz and raw total-variation counters are TELEMETRY ONLY.
   * Absolute position error is separately checked below at 20 kHz,
   * and overcurrent, VBUS, BKIN and time watchdog remain independent.
   */
  if (motorAbsCount(motorCountDelta(now, align_diag.align_count)) >
          POSITION_MAX_MOVE_COUNTS)
  {
    align_gate_status = 3U;
    align_diag.end_count = now;
    power_output_armed = false;
    pwmDisableOutput();
    motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
    return;
  }

  const float theta = encoderGetElectricalAngle();
  if (!isfinite(theta))
  {
    motorSetFault(MOTOR_FAULT_ENCODER_INVALID);
    return;
  }

  float id_ref = 0.0f;
  float iq_ref = 0.0f;
  if (t < POS_T_ID_ZERO)
  {
    const float u = (float)t / (float)POSITION_ID_DECAY_TICKS;
    id_ref = ALIGN_ID * (1.0f - u);
  }
  else if (t < POS_T_PLUS_START)
  {
    /* No field/torque while current-free resting position is established. */
  }
  else if (t < POS_T_HOME2_END)
  {
    /* Establish position zero once, after Id decays and Iq=0 settling. */
    if (t == POS_T_PLUS_START)
    {
      position_zero_count = now;
      align_diag.position_zero_count = now;
      position_prev_count = now;
      drive_prev_count = now;
      position_filtered_count = 0.0f;
      position_filter_prev_count = 0.0f;
      position_vel_counts_s = 0.0f;
      position_raw_meas_count = 0.0f;
      position_iq_p_term = 0.0f;
      position_iq_d_term = 0.0f;
    }
    position_raw_meas_count = (float)motorCountDelta(now, position_zero_count);

    /* Raw position/error ALWAYS determine emergency trip decisions.
       Filtered control feedback is never used to waive protection. */
    const float raw_error = position_ref_count - position_raw_meas_count;
    const int32_t abs_raw_error = motorAbsCount((int32_t)lroundf(raw_error));
    if (abs_raw_error > align_diag.peak_abs_raw_error_counts)
      align_diag.peak_abs_raw_error_counts = abs_raw_error;
    if (fabsf(raw_error) > POSITION_MAX_ERROR_COUNTS)
    {
      align_gate_status = 4U;
      align_diag.end_count = now;
      power_output_armed = false;
      pwmDisableOutput();
      motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
      return;
    }

    if (((t - POS_T_PLUS_START) % POSITION_CTRL_DIVIDER_TICKS) == 0U)
    {
      /* 1 kHz position low-pass, then damped differentiated velocity.
         This is intentionally a filtered velocity estimate (counts/s),
         NOT encoderUpdateSpeed(), which isn't serviced in this mode. */
      position_filtered_count += POSITION_POS_LPF_ALPHA *
          (position_raw_meas_count - position_filtered_count);
      const float unfiltered_velocity =
          (position_filtered_count - position_filter_prev_count) * 1000.0f;
      position_filter_prev_count = position_filtered_count;
      position_vel_counts_s += POSITION_VEL_LPF_ALPHA *
          (unfiltered_velocity - position_vel_counts_s);

      position_ref_count = motorPositionSegmentRef(t);
      position_meas_count = position_filtered_count;
      position_error_count = position_ref_count - position_meas_count;

      /* Continuous +/-1-count dead zone (no sudden jump at edge).
         For raw count +velocity, +Iq supplies braking on this motor. */
      float active_error = 0.0f;
      if (position_error_count > POSITION_ERROR_DEADBAND_COUNTS)
        active_error = position_error_count - POSITION_ERROR_DEADBAND_COUNTS;
      else if (position_error_count < -POSITION_ERROR_DEADBAND_COUNTS)
        active_error = position_error_count + POSITION_ERROR_DEADBAND_COUNTS;
      position_iq_p_term = -POSITION_KP_A_PER_COUNT * active_error;
      position_iq_d_term = clampFloat(
          POSITION_KD_A_PER_COUNT_PER_S * position_vel_counts_s,
          -POSITION_D_IQ_LIMIT_A, POSITION_D_IQ_LIMIT_A);
      const float desired_iq = clampFloat(
          position_iq_p_term + position_iq_d_term,
          -POSITION_IQ_LIMIT_A, POSITION_IQ_LIMIT_A);
      const float increment = clampFloat(desired_iq - position_iq_cmd,
          -POSITION_IQ_SLEW_A_PER_MS, POSITION_IQ_SLEW_A_PER_MS);
      position_iq_cmd += increment;

      const float gap = fabsf(position_raw_meas_count - position_filtered_count);
      const int32_t gap_mc = (int32_t)(gap * 1000.0f);
      if (gap_mc > align_diag.peak_raw_filter_gap_millicounts)
        align_diag.peak_raw_filter_gap_millicounts = gap_mc;
    }
    iq_ref = position_iq_cmd;
  }
  else
  {
    /* Reduce Iq smoothly to zero, then disable PWM automatically. */
    if (t == POS_T_HOME2_END)
      position_shutdown_iq = position_iq_cmd;
    const float u = (float)(t - POS_T_HOME2_END) /
                    (float)POSITION_FINAL_ZERO_TICKS;
    iq_ref = position_shutdown_iq * (1.0f - u);
    position_raw_meas_count = (float)motorCountDelta(now, position_zero_count);
    position_ref_count = 0.0f;
    if ((t % POSITION_CTRL_DIVIDER_TICKS) == 0U)
    {
      position_filtered_count += POSITION_POS_LPF_ALPHA *
          (position_raw_meas_count - position_filtered_count);
      position_meas_count = position_filtered_count;
      position_error_count = -position_meas_count;
    }
  }

  motorCurrentLoop(id_ref, iq_ref, theta);
  if (motor_state == MOTOR_STATE_FAULT) return;

  /* Boundaries logged after current loop updates; no UART calls in ISR. */
  if (t == POS_T_ID_ZERO)
  {
    align_diag.id_zero_count = now;
    align_diag.completed_stages |= 0x01U;
    motorCaptureAlignSample(4U, align_count + t);
  }
  if (t == POS_T_PLUS_START)
    motorCaptureAlignSample(5U, align_count + t);
  if (t == POS_T_PLUS_START + POSITION_TARGET_RAMP_TICKS)
    motorCaptureAlignSample(6U, align_count + t);
  if (t == POS_T_PLUS_END - 1U)
  {
    align_diag.plus_end_count = now;
    align_diag.completed_stages |= 0x02U;
    motorCaptureAlignSample(7U, align_count + t);
  }
  if (t == POS_T_HOME1_END - 1U)
  {
    align_diag.home1_end_count = now;
    align_diag.completed_stages |= 0x04U;
    motorCaptureAlignSample(8U, align_count + t);
  }
  if (t == POS_T_MINUS_END - 1U)
  {
    align_diag.minus_end_count = now;
    align_diag.completed_stages |= 0x08U;
    motorCaptureAlignSample(9U, align_count + t);
  }
  if (t == POS_T_HOME2_END - 1U)
  {
    align_diag.home2_end_count = now;
    align_diag.completed_stages |= 0x10U;
    motorCaptureAlignSample(10U, align_count + t);
  }
  if (t == POS_TOTAL_TICKS - 1U)
    motorCaptureAlignSample(11U, align_count + t);

  const int32_t abs_error = motorAbsCount((int32_t)lroundf(position_error_count));
  if (abs_error > align_diag.peak_abs_error_counts)
    align_diag.peak_abs_error_counts = abs_error;

  closed_loop_ticks++;
  if (closed_loop_ticks >= POS_TOTAL_TICKS)
  {
    align_gate_status = 1U;
    align_diag.end_count = (int32_t)encoderGetCount();
    power_output_armed = false;
    pwmDisableOutput();
    motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
  }
}

void motorLowSpeedTask(void)
{
  /* The injected callback normally ends the probe after ~3 seconds.
     Independently cut the power stage if those ADC interrupts stop. */
  if ((motor_state == MOTOR_STATE_ALIGN ||
       motor_state == MOTOR_STATE_CURRENT_LOOP) &&
      (uint32_t)(HAL_GetTick() - probe_start_ms) > POSITION_WATCHDOG_MS)
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

  /* Legacy alignment/commissioning remains radially limited to 1.8V.
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

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_SPEED)
static void motorSpeedLoop(void)
{
  if (motor_state != MOTOR_STATE_SPEED_LOOP)
  {
    return;
  }
  speed_w_meas = encoderGetMechanicalSpeed();

  if (!isfinite(speed_w_meas))
  {
    motorSetFault(MOTOR_FAULT_ENCODER_INVALID);
    return;
  }
  motor_monitor.speed_meas = speed_w_meas;

  float delta = speed_w_target - speed_w_ref;
  float step;

  if(delta >= 0.0f)
  {
    step = SPEED_RAMP_UP * SPD_DT;
  }
  else
  {
    step = SPEED_RAMP_DOWN * SPD_DT;
  }
  delta = clampFloat(delta, -step, step);
  speed_w_ref = clampFloat(speed_w_ref + delta, 0.0f, OUTPUT_SPD_REF_MAX);
  motor_monitor.speed_ref = speed_w_ref;
  current_id_ref = 0.0f;
  current_iq_ref = piControllerAntiWindup(&pi_spd, speed_w_ref, speed_w_meas, SPD_DT);
}
#endif

void motorControlUpdate(void)
{
  if (motor_state != MOTOR_STATE_CURRENT_LOOP &&
      motor_state != MOTOR_STATE_SPEED_LOOP &&
      motor_state != MOTOR_STATE_ALIGN)
  {
    return;
  }
  if (!power_output_armed)
  {
    return;
  }
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

#if _USE_ENCODER
  if (!encoderIsCalibrated())
  {
    motorSetFault(MOTOR_FAULT_ENCODER_INVALID);
    return;
  }

  const float theta_e = encoderGetElectricalAngle();

  if (!isfinite(theta_e))
  {
    motorSetFault(MOTOR_FAULT_ENCODER_INVALID);
    return;
  }
#endif

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_SPEED
  if (motor_state == MOTOR_STATE_SPEED_LOOP)
  {
    speed_loop_divider++;
    if (speed_loop_divider >= SPEED_LOOP_DIVIDER)
    {
      speed_loop_divider = 0U;
      encoderUpdateSpeed(SPD_DT);
      motorSpeedLoop();

      if (motor_state == MOTOR_STATE_FAULT)
      {
        return;
      }
    }
  }
#endif

  if (motor_state == MOTOR_STATE_CURRENT_LOOP)
  {
    if (offset_probe_selected)
      motorCommissionOffsetProbe();
    else if (vector_probe_selected)
      motorCommissionVectorProbe();
    else if (speed_probe_selected)
      motorCommissionSpeedPiProbe();
    else if (spin_probe_selected)
      motorCommissionSpinProbe();
    else if (torque_probe_direction != 0)
      motorCommissionTorqueProbe();
    else
      motorCommissionPositionTest();
    return;
  }
  motorCurrentLoop(current_id_ref, current_iq_ref, theta_e);
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
  monitor->speed_target = speed_w_target;
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

void motorSetCurrentReference(float id_ref, float iq_ref)
{
  if (!isfinite(id_ref) || !isfinite(iq_ref))
  {
    return;
  }

  current_id_ref = clampFloat(id_ref, OUTPUT_ID_REF_MIN, OUTPUT_ID_REF_MAX);
  current_iq_ref = clampFloat(iq_ref, OUTPUT_IQ_REF_MIN, OUTPUT_IQ_REF_MAX);
}

void motorSetSpeedReference(float w_ref)
{
  if (!isfinite(w_ref))
  {
    return;
  }

  speed_w_target = clampFloat(w_ref, 0.0f, OUTPUT_SPD_REF_MAX);
}

motor_state_t motorGetState(void)
{
  return motor_state;
}

motor_fault_t motorGetFault(void)
{
  return motor_fault;
}
