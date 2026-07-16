/*
 * hall.h
 *
 *  Created on: 2026. 7. 14.
 *      Author: YDG
 */

#ifndef SRC_COMMON_HW_INCLUDE_HALL_H_
#define SRC_COMMON_HW_INCLUDE_HALL_H_

#include "hw_def.h"
#include "util.h"

#if _USE_HALL_SENSOR

bool hallInit(void);
void hallUpdate(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void hallUpdateTimeout(void);

uint8_t hallGetState(void);
int8_t hallGetSectorIndex(void);
bool hallIsValid(void);

float hallGetElectricalSpeed(void);
float hallGetMechanicalSpeed(void);

float hallGetElectricalAngle(void);
float hallGetMechanicalAngle(void);
#endif

#endif /* SRC_COMMON_HW_INCLUDE_HALL_H_ */
