/*
 * motor_types.h
 *
 *  Created on: 2026. 7. 2.
 *      Author: YDG
 */

#ifndef SRC_COMMON_HW_INCLUDE_MOTOR_TYPES_H_
#define SRC_COMMON_HW_INCLUDE_MOTOR_TYPES_H_


#include "hw_def.h"


typedef struct
{
  float a;
  float b;
  float c;
} motor_abc_f_t;

typedef struct
{
  uint16_t a;
  uint16_t b;
  uint16_t c;
} motor_abc_u16_t;

typedef struct
{
  int32_t a;
  int32_t b;
  int32_t c;
} motor_abc_s32_t;

typedef struct
{
  float alpha;
  float beta;
} motor_alphabeta_t;

typedef struct
{
  float d;
  float q;
} motor_dq_t;

typedef struct
{
  float u;
  float v;
  float w;
} motor_duty_t;



#endif /* SRC_COMMON_HW_INCLUDE_MOTOR_TYPES_H_ */
