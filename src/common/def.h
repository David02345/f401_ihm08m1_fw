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
#define CUR_KP                            0.30f  //0.2f
#define CUR_KI                            246.0f   //50.0f
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
#define INTEGRAL_ID_MIN                  -0.5f
#define INTEGRAL_IQ_MAX                   0.5f
#define INTEGRAL_IQ_MIN                  -0.5f
#define INTEGRAL_SPD_MAX                  1.0f
#define INTEGRAL_SPD_MIN                 -1.0f
/*
#define OUTPUT_VD_REF_MAX                 0.5f
#define OUTPUT_VD_REF_MIN                 -0.5f
#define OUTPUT_VQ_REF_MAX                 0.5f
#define OUTPUT_VQ_REF_MIN                 -0.5f
*/
#define OUTPUT_VD_REF_MAX                 0.10f
#define OUTPUT_VD_REF_MIN                -0.10f
#define OUTPUT_VQ_REF_MAX                 0.35f
#define OUTPUT_VQ_REF_MIN                -0.35f
#define OUTPUT_ID_REF_MAX                 0.5f
#define OUTPUT_ID_REF_MIN                -0.5f
#define OUTPUT_IQ_REF_MAX                 1.0f
#define OUTPUT_IQ_REF_MIN                -1.0f
#define OUTPUT_SPD_REF_MAX                10.0f
#define OUTPUT_SPD_REF_MIN               -10.0f
/**
  * @}
  */

/** @defgroup Motor Software Specifications
  * @{
  */
#define MOTOR_VBUS                        14.0f   // 현재 시험 전압
#define MOTOR_VBUS_MIN                    12.0f
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
#define MOTOR_PHASE_R                     0.0784f     // Ohm
#define MOTOR_LQ                          0.00020f    // H
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
#define CURRENT_SW_BREAK_TEST_ENABLE      0U
#define CURRENT_SW_BREAK_TRIGGER_SAMPLE   20U

#define CURRENT_ADC_NOISE_TEST_ENABLE     0U
#define CURRENT_ADC_PWM_OFF_TEST_ENABLE   0U
#define CURRENT_ADC_SEQUENCE_TEST_ENABLE  0U
#define CURRENT_ADC_RAW_SCAN_ENABLE       0U  // ADC Raw 스파이크 검사 (MOE OFF, FOC OFF)
#define CURRENT_ADC_RAW_SPIKE_COUNTS      16U // Offset에서 16 Count 이상 벗어나면 이상 샘플로 기록

#define CURRENT_ADC_UART_SILENT_TEST      0U  // UART OFF 상태에서 10초 동안 Raw Scan 실행
#define CURRENT_ADC_UART_SILENT_MS        CURRENT_ADC_PWM_ON_SCAN_MS
// A-3: 측정 중 500ms마다 UART 송신
// 0 = A-2, 1 = A-3
#define CURRENT_ADC_TEST_UART_ENABLE      0U
#define CURRENT_ADC_SCAN_OC_LIMIT_A       0.8f
// Raw Scan 중 PWM ON 허용
#define CURRENT_ADC_PWM_ON_SCAN_ENABLE    0U
// 처음에는 100ms, 이후 1000ms -> 10000ms
#define CURRENT_ADC_PWM_ON_SCAN_MS        100U


// 신규 Iq Step Response 시험
#define CURRENT_IQ_STEP_TEST_ENABLE       0U
#define CURRENT_IQ_STEP_STAGE_SAMPLES     100U
#define CURRENT_IQ_STEP_REF_A             0.05f

// 신규 Hall FOC 회전 진단
#define CURRENT_ROTATION_TEST_ENABLE      1U
#define CURRENT_ROTATION_REF_A            0.55f
#define CURRENT_ROTATION_START_SAMPLE     2000U
#define CURRENT_ROTATION_END_SAMPLE       22000U
#define CURRENT_ROTATION_LOG_DIVIDER      200U

#define CURRENT_ROTATION_ANGLE_SWEEP_ENABLE  1U
#define CURRENT_ROTATION_SWEEP_SPEED_E       1.5f

#define CURRENT_FOC_BENCH_ENABLE          0U // PWM 출력 없이 FOC 계산 시간 측정
#define CURRENT_FOC_BENCH_DIAG_LOAD       0U // FOC 벤치마크에 Hall 검사 및 로그 저장 부하 추가


#if (CURRENT_IQ_STEP_TEST_ENABLE == 1U) && \
    ((CURRENT_NEUTRAL_DIAG_ENABLE != 1U) || \
     (CURRENT_ADC_NOISE_TEST_ENABLE != 0U) || \
     (CURRENT_ADC_SEQUENCE_TEST_ENABLE != 0U) || \
     (CURRENT_ADC_PWM_OFF_TEST_ENABLE != 0U))
#error "Iq step test requires current PI diagnostic mode"
#endif

#if (CURRENT_ROTATION_TEST_ENABLE == 1U)

#define CURRENT_LOOP_TEST_TIME_MS         1200U

#elif (CURRENT_ADC_SEQUENCE_TEST_ENABLE == 1U)

#define CURRENT_LOOP_TEST_TIME_MS         20U

#elif (CURRENT_IQ_STEP_TEST_ENABLE == 1U)

#define CURRENT_LOOP_TEST_TIME_MS         15U

#elif (CURRENT_ADC_NOISE_TEST_ENABLE == 1U)

#define CURRENT_LOOP_TEST_TIME_MS         10U

#else

#define CURRENT_LOOP_TEST_TIME_MS         10U

#endif
#define CURRENT_LOOP_PERIOD_US            50U

#define CURRENT_LOOP_TEST_SAMPLE_COUNT    ((CURRENT_LOOP_TEST_TIME_MS * 1000U) / CURRENT_LOOP_PERIOD_US)

#define CURRENT_TEST_OC_LIMIT_A           0.8f

#define CURRENT_TEST_V_ALPHA              0.00f
#define CURRENT_TEST_V_BETA               0.04f

#define CURRENT_NEUTRAL_DUTY              0.5f

#if (CURRENT_ROTATION_TEST_ENABLE == 1U)

#define CURRENT_L_TEST_SAMPLE_COUNT ((CURRENT_LOOP_TEST_SAMPLE_COUNT \
      + CURRENT_ROTATION_LOG_DIVIDER - 1U) / CURRENT_ROTATION_LOG_DIVIDER)

// Noise Test 비활성 상태에서 큰 미사용 배열 할당 방지
#define CURRENT_NOISE_TEST_SAMPLE_COUNT   1U

#else

#define CURRENT_L_TEST_SAMPLE_COUNT       CURRENT_LOOP_TEST_SAMPLE_COUNT

#define CURRENT_NOISE_TEST_SAMPLE_COUNT   CURRENT_LOOP_TEST_SAMPLE_COUNT

#endif

#if (CURRENT_ROTATION_TEST_ENABLE == 1U) && \
    ((CURRENT_NEUTRAL_DIAG_ENABLE != 1U) || \
     (CURRENT_ADC_NOISE_TEST_ENABLE != 0U) || \
     (CURRENT_ADC_PWM_OFF_TEST_ENABLE != 0U) || \
     (CURRENT_ADC_SEQUENCE_TEST_ENABLE != 0U) || \
     (CURRENT_IQ_STEP_TEST_ENABLE != 0U))
#error "Rotation test mode configuration error"
#endif

#if (CURRENT_ROTATION_TEST_ENABLE == 1U) && \
    (CURRENT_ROTATION_END_SAMPLE >= CURRENT_LOOP_TEST_SAMPLE_COUNT)
#error "Rotation end must precede test completion"

#endif

#if (CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U) && \
    ((CURRENT_ADC_RAW_SCAN_ENABLE != 1U) || \
     (CURRENT_ADC_UART_SILENT_TEST != 1U) || \
     (CURRENT_ADC_NOISE_TEST_ENABLE != 0U) || \
     (CURRENT_ROTATION_TEST_ENABLE != 0U) || \
     (CURRENT_FOC_BENCH_ENABLE != 0U))
#error "PWM ON scan requires exclusive Raw Scan mode"
#endif

#if CURRENT_ROTATION_TEST_ENABLE
#define CURRENT_DIAG_TAIL_SAMPLES         10U
#else
#define CURRENT_DIAG_TAIL_SAMPLES         20U
#endif

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
