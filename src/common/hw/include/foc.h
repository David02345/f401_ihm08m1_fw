/*
 * foc.h
 *
 *  Created on: 2026. 7. 1.
 *      Author: YDG
 */

#ifndef SRC_COMMON_HW_INCLUDE_FOC_H_
#define SRC_COMMON_HW_INCLUDE_FOC_H_

#include "hw_def.h"

#ifdef _USE_MOTOR_FOC

typedef struct
{
  float a;
  float b;
  float c;
} foc_abc_t;

typedef struct
{
  float alpha;
  float beta;
} foc_alphabeta_t;

typedef struct
{
  float d;
  float q;
} foc_dq_t;

typedef struct
{
  float u;
  float v;
  float w;
} foc_duty_t;


bool focInit(void);

void focClarke(float ia, float ib, float ic, foc_alphabeta_t *out);
void focPark(float alpha, float beta, float theta_e, foc_dq_t *out);

void focInvPark(float vd, float vq, float theta_e, foc_alphabeta_t *out);
void focInvClarke(float alpha, float beta, foc_abc_t *out);

void focSetVoltageLimit(foc_dq_t *v_dq, float v_limit);
void focGenerateSPWM(float valpha, float vbeta, float vbus, foc_duty_t *duty);
void focRunOpenLoopVoltage(float vd, float vq, float theta_e, float vbus, foc_duty_t *duty);

#endif

#endif /* SRC_COMMON_HW_INCLUDE_FOC_H_ */
