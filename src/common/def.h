/*
 * def.h
 *
 *  Created on: 2026. 5. 26.
 *      Author: YDG
 */

#ifndef SRC_COMMON_DEF_H_
#define SRC_COMMON_DEF_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

//R_UV ≈ R_VW ≈ R_WU ≈ 0.157 Ω

/** @defgroup Math Utils
  * @{
  */
#define PI                                3.14159265358979323846
/**
  * @}
  */

/** @defgroup PID Gains for Current, Speed, Position Control
  * @{
  */
#define CUR_KP                            0.2f
#define CUR_KI                            50.0f
#define CUR_KD                            0.0f
#define SPD_KP                            0.05f
#define SPD_KI                            1.0f
#define SPD_KD                            0.0f
#define POS_KP                            2.0f
#define POS_KI                            0.0f
#define POS_KD                            0.1f
/**
  * @}
  */

/** @defgroup Delta Time for Open-Loop, Current Loop, Speed Loop, Position Loop
  * @{
  */
#define CUR_DT                            0.00005f
#define SPD_DT                            0.001f
#define POS_DT                            0.01f
/**
  * @}
  */

/** @defgroup CubeMX PWM Value Setting
  * @{
  */
#define PWM_ARR                           2099
#define PWM_DEADTIME                      80
#define PWM_ADC_BLANK_TICKS               150
#define PWM_ADC_TRIG_PULSE                (PWM_ARR - PWM_ADC_BLANK_TICKS)
#define PWM_ADC_INIT_PULSE                1049
/**
  * @}
  */

/** @defgroup Speed Loop Parameters
  * @{
  */
#define SPEED_LOOP_DIVIDER                20
#define SPEED_CMD_DEADBAND_RAW            30
#define SPEED_RAMP_UP                     5.0f       // rad/s2
#define SPEED_RAMP_DOWN                   10.0f      // rad/s2
/**
  * @}
  */


/** @defgroup Integral & Output Minimum, Maximum Values
  * @{
  */
#define INTEGRAL_ID_MAX                   0.5f
#define INTEGRAL_ID_MIN                   -0.5f
#define INTEGRAL_IQ_MAX                   0.5f
#define INTEGRAL_IQ_MIN                   -0.5f
#define INTEGRAL_SPD_MAX                  1.0f
#define INTEGRAL_SPD_MIN                  -1.0f

#define OUTPUT_VD_REF_MAX                 0.5f
#define OUTPUT_VD_REF_MIN                 -0.5f
#define OUTPUT_VQ_REF_MAX                 0.5f
#define OUTPUT_VQ_REF_MIN                 -0.5f
#define OUTPUT_ID_REF_MAX                 0.5f
#define OUTPUT_ID_REF_MIN                 -0.5f
#define OUTPUT_IQ_REF_MAX                 1.0f
#define OUTPUT_IQ_REF_MIN                 -1.0f
#define OUTPUT_SPD_REF_MAX                10.0f
#define OUTPUT_SPD_REF_MIN                -10.0f
/**
  * @}
  */

/** @defgroup Motor Software Specifications
  * @{
  */
#define MOTOR_VBUS                        24
#define MOTOR_VBUS_MIN                    12
#define MOTOR_VLIMIT_SPWM                 MOTOR_VBUS * 0.5f
#define MOTOR_VLIMIT_SVPWM                MOTOR_VBUS * 0.577350269f
/**
  * @}
  */

/** @defgroup Motor Hardware Specifications
  * @{
  */
#define GEAR_RATIO                        25
#define GEAR_EFFICIENCY                   0.94
#define MOTOR_POLE_PAIRS                  4
/**
  * @}
  */

/** @defgroup Hall Sensor Parameters
  * @{
  */
#define HALL_SECTOR_ANGLE_E               (PI / 3.0f)
#define HALL_SPEED_LPF_ALPHA              1.0f
#define HALL_STOP_TIMEOUT_US              3000000U
#define HALL_ELEC_OFFSET                  5.295f

#define HALL_CAL_ALIGN_COUNT              20000U
#define HALL_CAL_ALIGN_VD                 0.40f
#define HALL_CAL_ROTATE_VD                0.45f
#define HALL_CAL_SPEED_E                  0.40f
#define HALL_CAL_DIRECTION                (-1.0f)

#define HALL_CAL_TEST_TIME_MS             24000U
/**
  * @}
  */

/** @defgroup Open-Loop Define
  * @{
  */
#define OPEN_DT                           0.00005f

#define OPEN_LOOP_ALIGN_COUNT             10000U
#define OPEN_LOOP_ALIGN_THETA_E           0.0f
#define OPEN_LOOP_ALIGN_VQ                0.3f

#define OPEN_LOOP_START_SPEED_E           0.0f
#define OPEN_LOOP_ACCEL_E                 0.2f
#define OPEN_LOOP_TARGET_SPEED            0.7f
#define OPEN_LOOP_TARGET_VQ               0.4f

#define OPEN_LOOP_TEST_TIME_MS            8000U
/**
  * @}
  */

/** @defgroup Current Loop Define
  * @{
  */
#define CURRENT_NEUTRAL_DIAG_ENABLE       1U

#define CURRENT_LOOP_TEST_TIME_MS         30U
#define CURRENT_LOOP_PERIOD_US            50U

// [수정] 30 ms × 20 kHz = 600 samples
#define CURRENT_LOOP_TEST_SAMPLE_COUNT    \
        ((CURRENT_LOOP_TEST_TIME_MS * 1000U) / CURRENT_LOOP_PERIOD_US)

#define CURRENT_TEST_OC_LIMIT_A           0.8f

// [수정] Fixed alpha test command
#define CURRENT_TEST_V_ALPHA              0.04f
#define CURRENT_TEST_V_BETA               0.00f

#define CURRENT_NEUTRAL_DUTY              0.5f
/**
  * @}
  */

/** @defgroup HW Channel Define
  * @{
  */
#define _DEF_LED1                         0
#define _DEF_LED2                         1
#define _DEF_LED3                         2
#define _DEF_LED4                         3

#define _DEF_UART1                        0
#define _DEF_UART2                        1
#define _DEF_UART3                        2
#define _DEF_UART4                        3
/**
  * @}
  */

#endif /* SRC_COMMON_DEF_H_ */
