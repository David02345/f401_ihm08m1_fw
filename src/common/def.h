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
#define PI                                3.14159265358979323846f
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
#define PWM_DEADTIME                      64 // 80
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
#define INTEGRAL_ID_MAX                   1.2f
#define INTEGRAL_ID_MIN                  -1.2f
#define INTEGRAL_IQ_MAX                   1.2f
#define INTEGRAL_IQ_MIN                  -1.2f
#define INTEGRAL_SPD_MAX                  1.0f
#define INTEGRAL_SPD_MIN                 -1.0f
#define OUTPUT_VD_REF_MAX                 1.8f
#define OUTPUT_VD_REF_MIN                -1.8f
#define OUTPUT_VQ_REF_MAX                 1.8f
#define OUTPUT_VQ_REF_MIN                -1.8f
#define OUTPUT_ID_REF_MAX                 2.0f
#define OUTPUT_ID_REF_MIN                -2.0f
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
#define GEAR_RATIO                        1.0f
#define GEAR_EFFICIENCY                   1.0f
#define COUNT_TO_ANGLE                    (2.0f * PI / (float)ENCODER_MAX_COUNT)
#define MOTOR_POLE_PAIRS                  4
#define MOTOR_PHASE_R                     0.0784f     // Ohm
#define MOTOR_LQ                          0.00020f    // H

#define ENCODER_MAX_COUNT                 4000U
#define ENCODER_ELEC_SIGN                 (-1.0f)
/**
  * @}
  */

/** @defgroup Hall Sensor Parameters
  * @{
  */
#define HALL_SECTOR_ANGLE_E               (PI / 3.0f)
#define HALL_SPEED_LPF_ALPHA              1.0f
#define HALL_STOP_TIMEOUT_US              3000000U
#define HALL_ELEC_OFFSET                  5.295f /* baseline H: Hall 010 -> 110 once */

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
#define CURRENT_TEST_OC_LIMIT_A           4.00f
/**
  * @}
  */

/** @defgroup Encoder FOC Position Closed-loop Commissioning
  * Static alignment at theta_s=0 for 2s, Id decay and zero-current settle.
  * Relative position feedback: +8 -> 0 -> -8 -> 0 encoder counts.
  * This is a deliberately small-angle position-controller commissioning run,
  * NOT an unlimited position servo. All Iq is clamped to +/-0.50 A.
  * Position feedback direction is inferred from measured +/-Iq pulses:
  * +Iq moved raw encoder COUNT NEGATIVE, so positive encoder position
  * error requires a NEGATIVE Iq command. Mechanical angle is raw TIM2 x4.
  * 20 kHz current loop; 1 kHz position-filter / PD update.
 * Both PD and snapshots use filtered position; raw counts are used for
 * excursion and tracking-error safety checks. No integral windup.
  * @{
  */
#define ALIGN_ID                           2.00f
#define ALIGN_RAMP_TICKS                   8000U
#define ALIGN_TOTAL_TICKS                  40000U
#define ALIGN_SETTLE_TICKS                 2000U
#define ALIGN_SETTLE_MAX_COUNTS            8
#define ALIGN_DIAG_VECTOR_VOLT_LIMIT_V     1.8f
#define ALIGN_DIAG_VBUS_MAX_V              16.0f

#define POSITION_TARGET_COUNTS             8.0f    /* +/-0.72 deg mechanical */
#define POSITION_KP_A_PER_COUNT            0.075f
/* Speed is raw encoder counts/second (not electrical radians/s).
 * Positive raw count velocity requires positive Iq to brake, since
 * +Iq physically moves raw counts negative on this hardware. */
#define POSITION_KD_A_PER_COUNT_PER_S      0.0008f
#define POSITION_D_IQ_LIMIT_A              0.18f
#define POSITION_POS_LPF_ALPHA             0.10f   /* 1ms update, ~9.5ms tau */
#define POSITION_VEL_LPF_ALPHA             0.035f  /* 1ms update, ~28ms tau */
#define POSITION_ERROR_DEADBAND_COUNTS     1.0f    /* continuous dead zone */
#define POSITION_IQ_LIMIT_A                0.50f
#define POSITION_IQ_SLEW_A_PER_MS          0.020f
#define POSITION_CTRL_DIVIDER_TICKS        20U      /* 1 kHz */
#define POSITION_ID_DECAY_TICKS            2000U    /* 0.10s */
#define POSITION_INITIAL_ZERO_TICKS        3000U    /* 0.15s */
#define POSITION_SEGMENT_TICKS             10000U   /* 0.50s per setpoint */
#define POSITION_TARGET_RAMP_TICKS         4000U    /* 0.20s target slew */
#define POSITION_FINAL_ZERO_TICKS          2000U    /* 0.10s command fade */

#define POS_T_ID_ZERO                      (POSITION_ID_DECAY_TICKS)
#define POS_T_PLUS_START                   (POS_T_ID_ZERO + POSITION_INITIAL_ZERO_TICKS)
#define POS_T_PLUS_END                     (POS_T_PLUS_START + POSITION_SEGMENT_TICKS)
#define POS_T_HOME1_END                    (POS_T_PLUS_END + POSITION_SEGMENT_TICKS)
#define POS_T_MINUS_END                    (POS_T_HOME1_END + POSITION_SEGMENT_TICKS)
#define POS_T_HOME2_END                    (POS_T_MINUS_END + POSITION_SEGMENT_TICKS)
#define POS_TOTAL_TICKS                    (POS_T_HOME2_END + POSITION_FINAL_ZERO_TICKS)

#define POSITION_MAX_MOVE_COUNTS           70
#define POSITION_MAX_TRAVEL_COUNTS         150 /* telemetry reference only, NOT a trip threshold */
#define POSITION_TRAVEL_DEADBAND_COUNTS      2      /* ignore 1-count dithering in 1 kHz travel accumulator */
#define POSITION_MAX_ERROR_COUNTS          18.0f
#define POSITION_WATCHDOG_MS               5500U

/* Standalone encoder-FOC torque-direction probe, F=+Iq / R=-Iq.
 * Position PD is BYPASSED, and the existing current PI remains active.
 * Runs after the SAME 2 s static alignment and Id-release settling.
 * Total time after alignment: 0.25 s settle + 0.35 s pulse = 0.60 s.
 * All values are bounded; no arbitrary user-supplied current accepted. */
#define TORQUE_PROBE_IQ_A                   0.30f
#define TORQUE_PROBE_RAMP_TICKS             2000U  /* 100ms */
#define TORQUE_PROBE_HOLD_TICKS             3000U  /* 150ms */
#define TORQUE_PROBE_DECAY_TICKS            2000U  /* 100ms */
#define TORQUE_PROBE_START_TICK             POS_T_PLUS_START
#define TORQUE_PROBE_TOTAL_TICKS            (TORQUE_PROBE_START_TICK + TORQUE_PROBE_RAMP_TICKS + TORQUE_PROBE_HOLD_TICKS + TORQUE_PROBE_DECAY_TICKS)
#define TORQUE_PROBE_MAX_MOVE_COUNTS        70     /* absolute angle excursion limit */

/* O: encoder-FOC phase offset diagnostic. One pulse per fresh alignment.
 * Use 0.30 A and same timing as the existing F/R test, not a high-current
 * 90-degree alignment experiment. Do not change stored electrical offset. */
#define OFFSET_PROBE_COUNT                  5U
#define OFFSET_PROBE_STEP_DEG               30
#define OFFSET_PROBE_IQ_A                   TORQUE_PROBE_IQ_A
#define OFFSET_PROBE_MAX_MOVE_COUNTS        TORQUE_PROBE_MAX_MOVE_COUNTS
#define OFFSET_PROBE_START_TICK             TORQUE_PROBE_START_TICK
#define OFFSET_PROBE_TOTAL_TICKS            TORQUE_PROBE_TOTAL_TICKS


/* V: stator-current-vector tracking diagnostic (NO encoder-angle feedback).
 * Keep the original 2A static align, then smoothly reduce Id to 1.5A.
 * Command theta=0 -> +90 electric deg -> 0 with Iq=0, finish at Id=0.
 * No position controller, no motor-angle-dependent Park/InvPark.
 * ISR still reads TIM2 for excursion safety and timestamped telemetry.
 * 2s alignment + 1.2s vector test < 5.5s watchdog.
 */
#define VECTOR_PROBE_ID_A                    3.50f
#define VECTOR_PROBE_ANGLE_DEG               90.0f
#define VECTOR_PROBE_MAX_MOVE_COUNTS         250
#define VECTOR_PROBE_ID_RAMP_TICKS           2000U   /* 100ms, 2A -> 1.5A */
#define VECTOR_PROBE_ANGLE_RAMP_TICKS        6000U   /* 300ms each way */
#define VECTOR_PROBE_ANGLE_HOLD_TICKS        12000U   /* 200ms at commanded angle */
#define VECTOR_PROBE_HOME_HOLD_TICKS         4000U   /* 200ms at 0deg */
#define VECTOR_PROBE_ID_OFF_TICKS            2000U   /* 100ms, 1.5A -> 0 */
#define VECTOR_T_UP_START                    (VECTOR_PROBE_ID_RAMP_TICKS)
#define VECTOR_T_UP_END                      (VECTOR_T_UP_START + VECTOR_PROBE_ANGLE_RAMP_TICKS)
#define VECTOR_T_HOLD_END                    (VECTOR_T_UP_END + VECTOR_PROBE_ANGLE_HOLD_TICKS)
#define VECTOR_T_DOWN_END                    (VECTOR_T_HOLD_END + VECTOR_PROBE_ANGLE_RAMP_TICKS)
#define VECTOR_T_HOME_END                    (VECTOR_T_DOWN_END + VECTOR_PROBE_HOME_HOLD_TICKS)
#define VECTOR_T_TOTAL                       (VECTOR_T_HOME_END + VECTOR_PROBE_ID_OFF_TICKS)
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
