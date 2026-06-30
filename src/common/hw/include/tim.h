/*
 * tim.h
 *
 *  Created on: 2026. 6. 30.
 *      Author: YDG
 */

#ifndef SRC_COMMON_HW_INCLUDE_TIM_H_
#define SRC_COMMON_HW_INCLUDE_TIM_H_

#include "hw_def.h"

bool timInit(void);
void pwmStart(void);
void pwmStop(void);
void pwmEnableOutput(void);
void pwmDisableOutput(void);
void pwmSetDuty(float duty_u, float duty_v, float duty_w);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle);

#endif /* SRC_COMMON_HW_INCLUDE_TIM_H_ */
