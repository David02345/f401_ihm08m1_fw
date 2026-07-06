/*
 * motor.c
 *
 *  Created on: 2026. 7. 4.
 *      Author: YDG
 */

#include "motor.h"
#include "motor_types.h"
#include "util.h"

typedef enum
{
  MOTOR_STATE_IDLE,
  MOTOR_STATE_READY,
  MOTOR_STATE_OPEN_LOOP,
  MOTOR_STATE_FAULT,
}motor_state_t;

static motor_state_t motor_state;
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


bool motorInit(void)
{
  bool ret = true;

  motor_state = MOTOR_STATE_IDLE;

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

  return ret;
}
void motorStart(void)
{
  if(motor_state == MOTOR_STATE_IDLE)
  {
    pwmStart();
    adcInjectedStart();
    if(adcCalibrateCurrentOffset() == true)
    {
      motor_state = MOTOR_STATE_READY;
    }
    else
    {
      pwmDisableOutput();
      adcInjectedStop();
      pwmStop();

      motor_state = MOTOR_STATE_FAULT;
    }
  }
  else if(motor_state == MOTOR_STATE_FAULT)
  {
    pwmDisableOutput();
    adcInjectedStop();
    pwmStop();

    return;
  }
  else if(motor_state == MOTOR_STATE_OPEN_LOOP)
  {
    pwmDisableOutput();
    adcInjectedStop();
    pwmStop();
    motor_state = MOTOR_STATE_FAULT;
    return;
  }
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
  open_loop_vq = 0.5f;
  pwmSetDuty(0.5, 0.5, 0.5);

  pwmEnableOutput();
  motor_state = MOTOR_STATE_OPEN_LOOP;
}
void motorControlUpdate(void)
{
  if(motor_state == MOTOR_STATE_OPEN_LOOP)
  {
    open_loop_theta_e += (open_loop_speed_e * OPEN_DT);
    open_loop_theta_e = wrapFloat(open_loop_theta_e, 0.0f, 2.0f * PI);
    focRunOpenLoopVoltage(open_loop_vd, open_loop_vq, open_loop_theta_e, motor_vbus, &motor_duty);
    pwmSetDuty(motor_duty.u, motor_duty.v, motor_duty.w);
  }
}
void motorLowSpeedTask(void)
{
  if(adcUpdateRegular())
  {
    motor_vbus = adcGetVbusVoltage();
    motor_speed_cmd_raw = adcGetSpeedRaw();
    motor_temp_raw = adcGetTempRaw();
  }
  else
  {
    motor_state = MOTOR_STATE_FAULT;
  }

  if (motor_state == MOTOR_STATE_FAULT)
  {
    pwmDisableOutput();
    return;
  }
}
