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
static int32_t align_start_count = 0;
/* Sampled only at known PWM control ticks. Accessed from main via IRQ lock. */
static volatile motor_align_diag_t align_diag;

static void motorCaptureAlignSample(uint32_t index)
{
  if (index >= 3U) return;
  volatile motor_align_diag_sample_t *s = &align_diag.samples[index];
  s->tick = align_count;
  s->encoder_count = (int32_t)encoderGetCount();
  s->theta_e = motor_monitor.theta_e;
  s->id_ref = motor_monitor.id_ref;
  s->id_meas = motor_monitor.id_meas;
  s->iq_meas = motor_monitor.iq_meas;
  s->vd = motor_monitor.vd_cmd;
  s->vq = motor_monitor.vq_cmd;
  s->ia = motor_monitor.ia;
  s->ib = motor_monitor.ib;
  s->ic = motor_monitor.ic;
  s->duty_u = motor_monitor.duty_u;
  s->duty_v = motor_monitor.duty_v;
  s->duty_w = motor_monitor.duty_w;
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

  motor_speed_cmd_raw = adcGetSpeedRaw();
  motor_temp_raw = adcGetTempRaw();

  pidReset(&pi_id);
  pidReset(&pi_iq);
  pidReset(&pi_spd);

  align_count = 0U;
  align_start_count = 0;
  memset((void *)&align_diag, 0, sizeof(align_diag));
  align_diag.start_count = (int32_t)encoderGetCount();

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
  motor_state = MOTOR_STATE_IDLE;
}

void motorAlign(void)
{
  if(motor_state != MOTOR_STATE_ALIGN)
  {
    return;
  }
  float id_align_ref;

  if (align_count < ALIGN_RAMP_TICKS)
  {
    id_align_ref = ALIGN_ID * ((float)(align_count + 1U) / (float)ALIGN_RAMP_TICKS);
  }
  else
  {
    id_align_ref = ALIGN_ID;
  }

  float theta_align = 0.0f;

  // 0.5초 ~ 1.5초: 자기장 0° → 90° 천천히 이동
  if (align_count >= 10000U && align_count < 30000U)
  {
    theta_align = ALIGN_THETA * ((float)(align_count - 10000U) / 20000.0f);
  }
  // 1.5초 이후: 90° 고정
  else if (align_count >= 30000U)
  {
    theta_align = ALIGN_THETA;
  }

  motorCurrentLoop(id_align_ref, 0.0f, theta_align);
  if (motor_state == MOTOR_STATE_FAULT)
  {
    return;
  }

  /* Take exactly three snapshots at 0, 45 and 90 electrical degrees.
   * The 0-deg sample is taken after the Id ramp is complete.
   * Never print UART from the 20 kHz ADC interrupt. */
  if (align_count == 9500U)
  {
    motorCaptureAlignSample(0U);
  }
  else if (align_count == 20000U)
  {
    motorCaptureAlignSample(1U);
  }
  else if (align_count == 35000U)
  {
    motorCaptureAlignSample(2U);
  }

  const uint32_t total_ticks = ALIGN_RAMP_TICKS + ALIGN_HOLD_TICKS;
  align_count++;

  if (align_count == (total_ticks - ALIGN_SETTLE_TICKS))
  {
    align_start_count = (int32_t)encoderGetCount();
  }

  if (align_count < total_ticks)
  {
    return;
  }

  /* Diagnostic endpoint: do NOT claim rotor alignment merely because the
   * encoder stopped moving, and do NOT assign a fictitious electrical offset.
   * Hardware output is disabled here immediately; main loop later stops ADC. */
  align_diag.end_count = (int32_t)encoderGetCount();
  power_output_armed = false;
  pwmDisableOutput();
  motor_state = MOTOR_STATE_ALIGN_DIAG_DONE;
}

void motorLowSpeedTask(void)
{
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

  focSetVoltageLimit(&v_dq, 0.45f * motor_vbus);
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

  motorCurrentLoop(current_id_ref, current_iq_ref, theta_e);
}

void motorSetFault(motor_fault_t fault)
{
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
