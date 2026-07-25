/*
 * pid.h
 *
 *  Created on: 2026. 7. 3.
 *      Author: YDG
 */

#ifndef SRC_COMMON_HW_INCLUDE_PID_H_
#define SRC_COMMON_HW_INCLUDE_PID_H_

#include "hw_def.h"



typedef struct
{
  float kp;
  float ki;
  float kd;

  float integral;
  float prev_error;

  float out_min;
  float out_max;
  float output;

  float integral_min;
  float integral_max;
} pid_ctrl_t;


bool pidInit(pid_ctrl_t *pid, float kp, float ki, float kd, float out_min, float out_max, float int_min, float int_max);
float piController(pid_ctrl_t *pi, float ref, float feedback, float dt);
float pdController(pid_ctrl_t *pd, float ref, float feedback, float dt);
void pidReset(pid_ctrl_t *pi);

#endif /* SRC_COMMON_HW_INCLUDE_PID_H_ */
