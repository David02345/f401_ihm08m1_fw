/*
 * pid.c
 *
 *  Created on: 2026. 7. 3.
 *      Author: YDG
 */

#include "pid.h"
#include "util.h"

bool pidInit(pid_ctrl_t *pid, float kp, float ki, float kd, float out_min, float out_max, float int_min, float int_max)
{
  bool ret = true;

  if (pid == NULL)
  {
    return false;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;

  pid->integral = 0.0f;
  pid->integral_max = int_max;
  pid->integral_min = int_min;
  pid->prev_error = 0.0f;
  pid->output = 0.0f;

  pid->out_max = out_max;
  pid->out_min = out_min;

  return ret;
}

float piController(pid_ctrl_t *pi, float ref, float feedback, float dt)
{
  if (pi == NULL || dt <= 0.0f)
  {
    return 0.0f;
  }

  float error = ref - feedback;

  pi->integral += pi->ki * error * dt;
  pi->integral = clampFloat(pi->integral, pi->integral_min, pi->integral_max);

  pi->output = pi->kp * error + pi->integral;
  pi->output = clampFloat(pi->output, pi->out_min, pi->out_max);

  return pi->output;
}

float pdController(pid_ctrl_t *pd, float ref, float feedback, float dt)
{
  if (pd == NULL || dt <= 0.0f)
  {
    return 0.0f;
  }

  float error = ref - feedback;
  float derivative = (error - pd->prev_error)/dt;

  pd->output = (pd->kp * error) + (pd->kd * derivative);

  pd->prev_error = error;

  pd->output = clampFloat(pd->output, pd->out_min, pd->out_max);

  return pd->output;
}

void pidReset(pid_ctrl_t *pid)
{
  if (pid == NULL)
  {
    return;
  }

  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->output = 0.0f;
}
