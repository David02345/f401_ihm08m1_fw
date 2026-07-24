/*
 * motor_types.h
 *
 *  Created on: 2026. 7. 2.
 *      Author: YDG
 */

#ifndef SRC_COMMON_HW_INCLUDE_MOTOR_TYPES_H_
#define SRC_COMMON_HW_INCLUDE_MOTOR_TYPES_H_


#include "hw_def.h"

typedef enum
{
  MOTOR_STATE_IDLE             = 0x00U,
  MOTOR_STATE_READY            = 0x01U,
  MOTOR_STATE_OPEN_LOOP        = 0x02U,
  MOTOR_STATE_CURRENT_LOOP     = 0x03U,
  MOTOR_STATE_SPEED_LOOP       = 0x04U,
  MOTOR_STATE_POSITION_LOOP    = 0x05U,
  MOTOR_STATE_FAULT            = 0x06U,
} motor_state_t;

typedef enum
{
  MOTOR_FAULT_NONE             = 0x00U,
  MOTOR_FAULT_INIT_FAIL        = 0x01U,
  MOTOR_FAULT_ADC_OFFSET_FAIL  = 0x02U,
  MOTOR_FAULT_VBUS_LOW         = 0x03U,
  MOTOR_FAULT_ADC_REGULAR_FAIL = 0x04U,
  MOTOR_FAULT_OVERCURRENT      = 0x05U,
  MOTOR_FAULT_OPEN_LOOP_FAIL   = 0x06U,
  MOTOR_FAULT_BKIN             = 0x07U,
} motor_fault_t;


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

typedef struct
{
  float ia;
  float ib;
  float ic;

  float id_ref;
  float id_meas;

  float iq_ref;
  float iq_meas;

  float vd_cmd;
  float vq_cmd;

  float theta_e;

  float speed_target;
  float speed_ref;
  float speed_meas;

  float duty_u;
  float duty_v;
  float duty_w;

  float vbus;
  float temp_raw;
  uint16_t speed_cmd_raw;

  motor_state_t state;
  motor_fault_t fault;
} motor_monitor_t;

#endif /* SRC_COMMON_HW_INCLUDE_MOTOR_TYPES_H_ */
