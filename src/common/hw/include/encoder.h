/*
 * encoder.h
 *
 *  Created on: 2026. 7. 28.
 *      Author: YDG
 */

#ifndef SRC_COMMON_HW_INCLUDE_ENCODER_H_
#define SRC_COMMON_HW_INCLUDE_ENCODER_H_

#include "hw_def.h"



bool encoderInit(void);
uint32_t encoderGetCount(void);
float encoderGetElectricalSpeed(void);
float encoderGetMechanicalSpeed(void);
float encoderGetElectricalAngle(void);
float encoderGetMechanicalAngle(void);


#endif /* SRC_COMMON_HW_INCLUDE_ENCODER_H_ */
