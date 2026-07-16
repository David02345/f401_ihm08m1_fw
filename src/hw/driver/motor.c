/*
 * motor.c
 *
 *  Created on: 2026. 7. 4.
 *      Author: YDG
 */

#include "motor.h"
#include "motor_types.h"
#include "util.h"


static volatile motor_state_t motor_state;
static volatile motor_fault_t motor_fault = MOTOR_FAULT_NONE;
static motor_duty_t motor_duty;

static pid_ctrl_t pi_id;
static pid_ctrl_t pi_iq;
static pid_ctrl_t pi_spd;


static volatile float motor_vbus    = 0.0f;
static uint16_t motor_speed_cmd_raw = 0;
static uint16_t motor_temp_raw      = 0;



#if _USE_MOTOR_SPEED_LOOP
static float speed_w_ref  = 0.0f;
static float speed_w_meas = 0.0f;

static uint16_t speed_loop_divider = 0;
#endif

#if _USE_MOTOR_CURRENT_LOOP
static float current_id_ref = 0.0f;
static float current_iq_ref = 0.0f;
#endif

#if _USE_MOTOR_OPENLOOP
static float open_loop_theta_e = 0.0f;
static float open_loop_speed_e = 0.0f;
static float open_loop_vd = 0.0f;
static float open_loop_vq = 0.0f;
#endif

bool motorInit(void)
{
  bool ret = true;

  motor_state = MOTOR_STATE_IDLE;
  motor_fault = MOTOR_FAULT_NONE;

  if(pidInit(&pi_id, CUR_KP, CUR_KI, CUR_KD, OUTPUT_ID_MIN, OUTPUT_ID_MAX) != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
  if(pidInit(&pi_iq, CUR_KP, CUR_KI, CUR_KD, OUTPUT_IQ_MIN, OUTPUT_IQ_MAX) != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
  if(pidInit(&pi_spd, VEL_KP, VEL_KI, VEL_KD, OUTPUT_VEL_MIN, OUTPUT_VEL_MAX) != true)
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
  if(hallInit() != true)
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

  if(motor_vbus < MOTOR_VBUS_MIN)
  {
    motorSetFault(MOTOR_FAULT_VBUS_LOW);
    return;
  }

  adcInjectedStart();
  pwmStart();

  if(adcCalibrateCurrentOffset() != true)
  {
    motorSetFault(MOTOR_FAULT_ADC_OFFSET_FAIL);
    return;
  }

#if _USE_MOTOR_SPEED_LOOP
    speed_w_ref    = 0.0f;

    motor_state = MOTOR_STATE_SPEED_LOOP;
#elif _USE_MOTOR_CURRENT_LOOP
    current_id_ref = 0.0f;
    current_iq_ref = 0.0f;

    motor_state = MOTOR_STATE_CURRENT_LOOP;
#elif _USE_MOTOR_OPENLOOP
    open_loop_theta_e = 0.0f;
    open_loop_speed_e = 0.5f;
    open_loop_vd = 0.0f;
    open_loop_vq = OPEN_LOOP_TARGET_VQ;
    pwmSetDuty(0.5, 0.5, 0.5);

    motor_state = MOTOR_STATE_OPEN_LOOP;
#endif
}

void motorStop(void)
{
  pwmDisableOutput();
  adcInjectedStop();
  pwmStop();

  motor_state = MOTOR_STATE_IDLE;
}

#if _USE_MOTOR_SPEED_LOOP
static void motorSpeedLoop(void)
{
  if (motor_state != MOTOR_STATE_SPEED_LOOP)
  {
    return;
  }
  speed_w_meas = hallGetMechanicalSpeed();

  current_id_ref = 0;
  current_iq_ref = piController(&pi_spd, speed_w_ref, speed_w_meas, SPD_DT);
}
#endif

#if _USE_MOTOR_CURRENT_LOOP
static void motorCurrentLoop(float id_ref, float iq_ref, float theta_e)
{
  if (motor_state != MOTOR_STATE_CURRENT_LOOP)
  {
    return;
  }
  motor_abc_f_t i_abc;
  motor_alphabeta_t i_ab;
  motor_dq_t i_dq;
  motor_dq_t v_dq;
  motor_alphabeta_t v_ab;

  adcGetPhaseCurrent(&i_abc);

  focClarke(i_abc.a, i_abc.b, i_abc.c, &i_ab);
  focPark(i_ab.alpha, i_ab.beta, theta_e, &i_dq);

  v_dq.d = piController(&pi_id, id_ref, i_dq.d, CUR_DT);
  v_dq.q = piController(&pi_iq, iq_ref, i_dq.q, CUR_DT);

  focSetVoltageLimit(&v_dq, MOTOR_VLIMIT);

  focInvPark(v_dq.d, v_dq.q, theta_e, &v_ab);
  focGenerateSPWM(v_ab.alpha, v_ab.beta, motor_vbus, &motor_duty);

  pwmSetDuty(motor_duty.u, motor_duty.v, motor_duty.w);
}
#endif


#if _USE_MOTOR_OPENLOOP

static void motorOpenLoop(void)
{
  if(motor_state == MOTOR_STATE_OPEN_LOOP)
  {
    open_loop_speed_e += OPEN_LOOP_ACCEL_E * OPEN_DT;
    open_loop_speed_e = clampFloat(open_loop_speed_e, 0.5f, OPEN_LOOP_TARGET_SPEED);
    open_loop_theta_e += (open_loop_speed_e * OPEN_DT);
    open_loop_theta_e = wrapFloat(open_loop_theta_e, 0.0f, 2.0f * PI);
    focRunOpenLoopVoltage(open_loop_vd, open_loop_vq, open_loop_theta_e, motor_vbus, &motor_duty);
    pwmSetDuty(motor_duty.u, motor_duty.v, motor_duty.w);
  }
}
static void motorOpenLoop
#endif

void motorControlUpdate(void)
{
  float theta_e;

  if (motor_state != MOTOR_STATE_CURRENT_LOOP &&
      motor_state != MOTOR_STATE_SPEED_LOOP)
  {
      return;
  }

#if _USE_HALL_SENSOR
  theta_e = hallGetElectricalAngle();
#else
  theta_e = 0.0f;
#endif

#if _USE_MOTOR_SPEED_LOOP
  if (motor_state == MOTOR_STATE_SPEED_LOOP)
  {
    speed_loop_divider++;

    if (speed_loop_divider >= SPEED_LOOP_DIVIDER)
    {
      speed_loop_divider = 0;
      motorSpeedLoop();       // iq_ref만 갱신
    }
  }
#endif
#if _USE_MOTOR_CURRENT_LOOP
   motorCurrentLoop(current_id_ref, current_iq_ref, theta_e);
#endif
#if _USE_MOTOR_OPENLOOP
  motorOpenLoop();
#endif
}

void motorLowSpeedTask(void)
{
  if(motor_state == MOTOR_STATE_FAULT)
  {
    return;
  }

  if (pwmIsBreakFault())
   {
     motorSetFault(MOTOR_FAULT_BKIN);
     return;
   }

  if(adcUpdateRegular() != true)
  {
    motorSetFault(MOTOR_FAULT_ADC_REGULAR_FAIL);
    return;
  }
  motor_vbus = adcGetVbusVoltage();
  motor_speed_cmd_raw = adcGetSpeedRaw();
  motor_temp_raw = adcGetTempRaw();

  if(motor_vbus < MOTOR_VBUS_MIN)
  {
    motorSetFault(MOTOR_FAULT_VBUS_LOW);
    return;
  }
}

void motorSetFault(motor_fault_t fault)
{
  pwmDisableOutput();

  adcInjectedStop();
  pwmStop();

  pidReset(&pi_id);
  pidReset(&pi_iq);
  pidReset(&pi_spd);

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

motor_state_t motorGetState(void)
{
  return motor_state;
}

motor_fault_t motorGetFault(void)
{
  return motor_fault;
}

float motorGetVbus(void)
{
  return motor_vbus;
}


