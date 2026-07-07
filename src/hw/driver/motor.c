/*
 * motor.c
 *
 *  Created on: 2026. 7. 4.
 *      Author: YDG
 */

#include "motor.h"
#include "motor_types.h"
#include "util.h"



static motor_state_t motor_state;
static motor_fault_t motor_fault = MOTOR_FAULT_NONE;
static motor_duty_t motor_duty;

static pid_ctrl_t pi_id;
static pid_ctrl_t pi_iq;
static pid_ctrl_t pi_vel;
static pid_ctrl_t pd_pos;

static float motor_vbus = 0.0f;
static uint16_t motor_speed_cmd_raw = 0;
static uint16_t motor_temp_raw = 0;
static float open_loop_theta_e = 0.0f;
static float open_loop_speed_e = 0.0f;
static float open_loop_vd = 0.0f;
static float open_loop_vq = 0.0f;



static void motorSetFault(motor_fault_t fault);



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
  if(pidInit(&pi_vel, VEL_KP, VEL_KI, VEL_KD, OUTPUT_VEL_MIN, OUTPUT_VEL_MAX) != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
  if(pidInit(&pd_pos, POS_KP, POS_KI, POS_KD, OUTPUT_POS_MIN, OUTPUT_POS_MAX) != true)
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

  pwmStart();
  adcInjectedStart();

  if(adcCalibrateCurrentOffset() != true)
  {
    motorSetFault(MOTOR_FAULT_ADC_OFFSET_FAIL);
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

  motor_state = MOTOR_STATE_READY;
}

void motorStop(void)
{
  pwmDisableOutput();
  adcInjectedStop();
  pwmStop();

  motor_state = MOTOR_STATE_IDLE;
}
void motorOpenLoopStart(void)
{
  if (motor_state != MOTOR_STATE_READY)
  {
    return;
  }
  open_loop_theta_e = 0.0f;
  open_loop_speed_e = 0.5f;
  open_loop_vd = 0.0f;
  open_loop_vq = OPEN_LOOP_TARGET_VQ;
  pwmSetDuty(0.5, 0.5, 0.5);

  pwmEnableOutput();
  motor_state = MOTOR_STATE_OPEN_LOOP;
}

void motorControlUpdate(void)
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

void motorLowSpeedTask(void)
{
  if(motor_state == MOTOR_STATE_FAULT)
  {
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

static void motorSetFault(motor_fault_t fault)
{
  pwmDisableOutput();

  adcInjectedStop();
  pwmStop();

  motor_fault = fault;
  motor_state = MOTOR_STATE_FAULT;
}
