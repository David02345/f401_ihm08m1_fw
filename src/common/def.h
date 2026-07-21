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
/**
  * @}
  */


/** @defgroup Integral & Output Minimum, Maximum Values
  * @{
  */
#define INTEGRAL_ID_MAX                   12.0f
#define INTEGRAL_ID_MIN                   -12.0f
#define INTEGRAL_IQ_MAX                   12.0f
#define INTEGRAL_IQ_MIN                   -12.0f
#define INTEGRAL_VEL_MAX                  1.0f
#define INTEGRAL_VEL_MIN                  -1.0f

#define OUTPUT_VD_REF_MAX                 12.0f
#define OUTPUT_VD_REF_MIN                 -12.0f
#define OUTPUT_VQ_REF_MAX                 12.0f
#define OUTPUT_VQ_REF_MIN                 -12.0f
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
#define MOTOR_VLIMIT_SPWM                 12
#define MOTOR_VLIMIT_SVPWM                13.86
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
#define HALL_SPEED_LPF_ALPHA              0.2f
#define HALL_STOP_TIMEOUT_US              200000U
#define HALL_ELEC_OFFSET                  0.0f
/**
  * @}
  */

/** @defgroup Open-Loop Define
  * @{
  */
#define OPEN_DT                           0.00005f
#define OPEN_LOOP_ACCEL_E                 20.0f
#define OPEN_LOOP_TARGET_SPEED            (20.0f * PI)
#define OPEN_LOOP_TARGET_VQ               1.5f
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
