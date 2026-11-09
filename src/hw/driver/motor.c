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
static float speed_w_target = 0.0f;
static float speed_w_ref = 0.0f;
static float speed_w_meas = 0.0f;
static uint16_t speed_loop_divider = 0U;
static volatile bool power_output_armed = false;

bool motorInit(void)
{
  bool ret = true;

  motor_state = MOTOR_STATE_IDLE;
  motor_fault = MOTOR_FAULT_NONE;

  if(pidInit(&pi_id, CUR_KP, CUR_KI, CUR_KD, OUTPUT_ID_REF_MIN, OUTPUT_ID_REF_MAX, INTEGRAL_ID_MIN, INTEGRAL_ID_MAX) != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
  if(pidInit(&pi_iq, CUR_KP, CUR_KI, CUR_KD, OUTPUT_IQ_REF_MIN, OUTPUT_IQ_REF_MAX, INTEGRAL_IQ_MIN, INTEGRAL_IQ_MAX) != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
  if(pidInit(&pi_spd, SPD_KP, SPD_KI, SPD_KD, OUTPUT_SPD_REF_MIN, OUTPUT_SPD_REF_MAX, INTEGRAL_SPD_MIN, INTEGRAL_SPD_MAX) != true)
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
  if (ret != true)
  {
    motorSetFault(MOTOR_FAULT_INIT_FAIL);
  }
#endif

  return ret;
}

void motorStart(void)
{
#if MOTOR_ALLOW_POWER_STAGE == 0U
  /* Encoder drivers currently return zero / lack true angle calibration. */
  pwmDisableOutput();
  return;
#else
  if (motor_state != MOTOR_STATE_IDLE) return;
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
  pidReset(&pi_id); pidReset(&pi_iq); pidReset(&pi_spd);
  current_id_ref = 0.0f; current_iq_ref = 0.0f;
  speed_w_ref = 0.0f; speed_w_meas = 0.0f; speed_w_target = 0.0f;
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
    motorSetFault(MOTOR_FAULT_BKIN);
    return;
  }
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT
  motor_state = MOTOR_STATE_CURRENT_LOOP;
#elif MOTOR_CONTROL_MODE == MOTOR_CONTROL_SPEED
  motor_state = MOTOR_STATE_SPEED_LOOP;
#else
#error "Use CURRENT or SPEED for encoder FOC"
#endif
  /* Avoid treating an ADC callback between state-change and MOE as a fault. */
  pwmEnableOutput();
  if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
  {
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
  pidReset(&pi_id); pidReset(&pi_iq); pidReset(&pi_spd);
  current_id_ref = 0.0f; current_iq_ref = 0.0f;
  speed_w_target = 0.0f; speed_w_ref = 0.0f; speed_w_meas = 0.0f;
  speed_loop_divider = 0U;
  motor_state = MOTOR_STATE_IDLE;
}

void motorLowSpeedTask(void)
{
  if (motor_state != MOTOR_STATE_IDLE) return;
  /* Permit PWM-off encoder testing with motor VBUS disconnected. */
  if (!adcUpdateRegular())
  {
    motorSetFault(MOTOR_FAULT_ADC_REGULAR_FAIL);
    return;
  }
  motor_vbus = adcGetVbusVoltage();
  motor_speed_cmd_raw = adcGetSpeedRaw();
  motor_temp_raw = adcGetTempRaw();
}


static void motorCurrentLoop(float id_ref, float iq_ref, float theta_e)
{
  motor_abc_f_t i_abc;
  motor_alphabeta_t i_ab, v_ab;
  motor_dq_t i_dq, v_dq;

  /* PWM-ADC injected sampling is unchanged from the original project. */
  adcGetPhaseCurrent(&i_abc);
  motor_monitor.ia = i_abc.a;
  motor_monitor.ib = i_abc.b;
  motor_monitor.ic = i_abc.c;

  if (!isfinite(theta_e) || !isfinite(i_abc.a) || !isfinite(i_abc.b) ||
      !isfinite(i_abc.c))
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

  /* The original conditional anti-windup PI implementation is preserved. */
  v_dq.d = piControllerAntiWindup(&pi_id, id_ref, i_dq.d, CUR_DT);
  v_dq.q = piControllerAntiWindup(&pi_iq, iq_ref, i_dq.q, CUR_DT);
  /* SPWM voltage-vector limit based on measured VBUS, leaving a margin. */
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
  if (motor_state != MOTOR_STATE_SPEED_LOOP) return;
  speed_w_meas = encoderGetMechanicalSpeed();  /* rad/s, motor shaft */
  if (!isfinite(speed_w_meas))
  {
    motorSetFault(MOTOR_FAULT_ENCODER_INVALID);
    return;
  }
  motor_monitor.speed_meas = speed_w_meas;

  if (motor_speed_cmd_raw <= SPEED_CMD_DEADBAND_RAW)
    speed_w_target = 0.0f;
  else
  {
    float ratio = (float)(motor_speed_cmd_raw - SPEED_CMD_DEADBAND_RAW) /
                  (float)(4095U - SPEED_CMD_DEADBAND_RAW);
    speed_w_target = clampFloat(ratio, 0.0f, 1.0f) * OUTPUT_SPD_REF_MAX;
  }
  float delta = speed_w_target - speed_w_ref;
  const float step = ((delta >= 0.0f) ? SPEED_RAMP_UP : SPEED_RAMP_DOWN) * SPD_DT;
  delta = clampFloat(delta, -step, step);
  speed_w_ref = clampFloat(speed_w_ref + delta, 0.0f, OUTPUT_SPD_REF_MAX);
  motor_monitor.speed_ref = speed_w_ref;
  current_id_ref = 0.0f;
  current_iq_ref = piController(&pi_spd, speed_w_ref, speed_w_meas, SPD_DT);
}
#endif

void motorControlUpdate(void)
{
  if (motor_state != MOTOR_STATE_CURRENT_LOOP &&
      motor_state != MOTOR_STATE_SPEED_LOOP) return;
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
  const float theta_e = encoderGetElectricalAngle();
  if (!isfinite(theta_e))
  {
    motorSetFault(MOTOR_FAULT_ENCODER_INVALID);
    return;
  }
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_SPEED
  speed_loop_divider++;
  if (speed_loop_divider >= SPEED_LOOP_DIVIDER)
  {
    speed_loop_divider = 0U;
    motorSpeedLoop();
    if (motor_state == MOTOR_STATE_FAULT) return;
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
  if (!isfinite(id_ref) || !isfinite(iq_ref)) return;
  current_id_ref = clampFloat(id_ref, OUTPUT_ID_REF_MIN, OUTPUT_ID_REF_MAX);
  current_iq_ref = clampFloat(iq_ref, OUTPUT_IQ_REF_MIN, OUTPUT_IQ_REF_MAX);
}

motor_state_t motorGetState(void) { return motor_state; }
motor_fault_t motorGetFault(void) { return motor_fault; }
