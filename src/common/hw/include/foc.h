/*
 * foc.h
 *
 *  Created on: 2026. 7. 1.
 *      Author: YDG
 */

#ifndef SRC_COMMON_HW_INCLUDE_FOC_H_
#define SRC_COMMON_HW_INCLUDE_FOC_H_

#include "hw_def.h"
#include "motor_types.h"


#ifdef _USE_MOTOR_FOC


bool focInit(void);

void focClarke(float ia, float ib, float ic, motor_alphabeta_t *out);
void focPark(float alpha, float beta, float theta_e, motor_dq_t *out);

void focInvPark(float vd, float vq, float theta_e, motor_alphabeta_t *out);
void focInvClarke(float alpha, float beta, motor_abc_f_t *out);

void focSetVoltageLimit(motor_dq_t *v_dq, float v_limit);
void focGenerateSPWM(float valpha, float vbeta, float vbus, motor_duty_t *duty);
void focRunOpenLoopVoltage(float vd, float vq, float theta_e, float vbus, motor_duty_t *duty);

#endif

#endif /* SRC_COMMON_HW_INCLUDE_FOC_H_ */
