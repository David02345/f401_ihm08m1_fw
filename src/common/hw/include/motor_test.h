/*
 * motor_test.h
 *
 *  Created on: 2026. 10. 27.
 *      Author: YDG
 */

#ifndef SRC_COMMON_HW_INCLUDE_MOTOR_TEST_H_
#define SRC_COMMON_HW_INCLUDE_MOTOR_TEST_H_


#include "hw_def.h"
#include "pid.h"
#include "pwm.h"
#include "adc.h"
#include "foc.h"
#include "hall.h"
#include "motor_types.h"

typedef struct
{
    float id_ref;
    float iq_ref;

    float id_meas;
    float iq_meas;

    float vd_cmd;
    float vq_cmd;

    float theta_e;
} motor_test_sample_t;

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)



bool motorTestDiagIsDone(void);
uint32_t motorTestDiagGetSampleCount(void);
void motorTestDiagGetCurrentMinMax(motor_abc_f_t *i_min, motor_abc_f_t *i_max);

uint32_t motorLTestGetCount(void);
bool motorLTestGetSample(uint32_t index, float *id, float *iq, float *vd, float *vq, float *theta_e);




#endif

#endif /* SRC_COMMON_HW_INCLUDE_MOTOR_TEST_H_ */
