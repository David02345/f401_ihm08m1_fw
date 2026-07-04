/*
 * pid.c
 *
 *  Created on: 2026. 7. 3.
 *      Author: YDG
 */

#include "pid.h"
#include "util.h"

bool pidInit(pid_ctrl_t *pid, float kp, float ki, float kd, float min, float max)
{
  bool ret = true;

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;

  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->output = 0.0f;

  pid->out_max = max;
  pid->out_min = min;

  return ret;
}

float piController(pid_ctrl_t *pi, float ref, float feedback, float dt)
{
  if (pi == NULL || dt <= 0.0f)
  {
    return 0.0f;
  }

  float error = ref - feedback;

  pi->integral += error * dt;
  clampFloat(pi->integral, -100.0f, 100.0f);

  pi->output = (pi->kp * error) + (pi->ki * pi->integral);
  clampFloat(pi->output, pi->out_min, pi->out_max);

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

  clampFloat(pd->output, pd->out_min, pd->out_max);

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
