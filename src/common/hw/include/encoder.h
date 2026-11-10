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
void encoderUpdateSpeed(float dt);
float encoderGetElectricalSpeed(void);
float encoderGetMechanicalSpeed(void);
float encoderGetElectricalAngle(void);
float encoderGetMechanicalAngle(void);
float encoderGetGearedAngle(void);
void encoderClearCalibration(void);
bool encoderIsCalibrated(void);
void encoderResetSpeedTracking(void);
void encoderSetElectricalOffset(float offset);


#endif /* SRC_COMMON_HW_INCLUDE_ENCODER_H_ */
