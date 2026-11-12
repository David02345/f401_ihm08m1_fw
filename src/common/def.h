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

/** Current PI gains (20kHz). Speed PI gains are in K/L section. */
#define CUR_KP                            0.30f
#define CUR_KI                            246.0f
#define CUR_KD                            0.0f
#define CUR_DT                            0.00005f
#define SPD_DT                            0.001f /* used by legacy hall driver */

/** PWM and ADC sampling configuration. */
#define PWM_ARR                           2099
#define PWM_DEADTIME                      64
#define PWM_ADC_BLANK_TICKS               150
#define PWM_ADC_TRIG_PULSE                (PWM_ARR - PWM_ADC_BLANK_TICKS)
#define PWM_ADC_INIT_PULSE                1049

/** Current PI startup limits; speed mode expands voltage limits after Id release. */
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
#define OUTPUT_IQ_REF_MAX                 1.0f
#define OUTPUT_IQ_REF_MIN                -1.0f

#define MOTOR_VBUS                        14.0f
#define MOTOR_VBUS_MIN                    12.0f
#define MOTOR_POLE_PAIRS                  4
#define ENCODER_MAX_COUNT                 4000U
#define ENCODER_ELEC_SIGN                 (-1.0f)
#define COUNT_TO_ANGLE                    (2.0f * PI / (float)ENCODER_MAX_COUNT)

/* The Hall driver is retained in the BSP but disabled in the active build. */
#define HALL_SECTOR_ANGLE_E               (PI / 3.0f)
#define HALL_SPEED_LPF_ALPHA              1.0f
#define HALL_STOP_TIMEOUT_US              3000000U
#define HALL_ELEC_OFFSET                  5.295f

#define CURRENT_TEST_OC_LIMIT_A           4.00f

/* Static 2A electrical alignment: 2s total, 0.1s final settle gate. */
#define ALIGN_ID                          2.00f
#define ALIGN_RAMP_TICKS                  8000U
#define ALIGN_TOTAL_TICKS                 40000U
#define ALIGN_SETTLE_TICKS                2000U
#define ALIGN_SETTLE_MAX_COUNTS           8
#define ALIGN_DIAG_VECTOR_VOLT_LIMIT_V    1.8f
#define ALIGN_DIAG_VBUS_MAX_V             18.0f

/* Retained values from the proven speed-only Id release:
 * originally POSITION_ID_DECAY_TICKS=2000 and POS_T_PLUS_START=5000.
 * The 250-count cap was previously shared with the high-Iq probe. */
#define MOTOR_SPEED_ID_DECAY_TICKS        2000U
#define MOTOR_SPEED_ID_SETTLE_TICKS       3000U
#define MOTOR_SPEED_ALIGN_RELEASE_MAX_MOVE_COUNTS 250

/** @defgroup FOC Speed Control - K/L
  * @{
  */
#define MOTOR_SPEED_PI_GOAL_RPM            180.0f
#define MOTOR_SPEED_PI_RAMP_RPM_PER_S      400.0f
#define MOTOR_SPEED_PI_KP_A_PER_RPM        0.006f
#define MOTOR_SPEED_PI_KI_A_PER_RPM_S      0.015f
#define MOTOR_SPEED_PI_IQ_MAX_A            3.20f
#define MOTOR_SPEED_PI_OC_MARGIN_A         0.25f
#define MOTOR_SPEED_PI_V_LIMIT_V           4.50f
#define MOTOR_SPEED_PI_TICKS_PER_STEP      200U    /* 10 ms at 20 kHz */
#define MOTOR_SPEED_PI_RUN_TICKS           110000U /* 5.5s absolute safety cutoff after Id zero */
/* Whole K/L sequence watchdog: static align 2s + Id release 0.25s +
 * at most 5.5s speed-profile controller + 0.5s scheduling margin.
 */
#define MOTOR_SPEED_PI_WATCHDOG_MS \
  ((ALIGN_TOTAL_TICKS + MOTOR_SPEED_PI_START_TICK + MOTOR_SPEED_PI_RUN_TICKS) / 20U + 500U)
#define MOTOR_SPEED_PI_START_TICK          (MOTOR_SPEED_ID_DECAY_TICKS + MOTOR_SPEED_ID_SETTLE_TICKS)
#define MOTOR_SPEED_PI_MAX_RPM             500.0f
#define MOTOR_SPEED_PI_MAX_TRAVEL_COUNTS  65000   /* 16.25 mechanical rev; emergency travel cut */
#define MOTOR_SPEED_PI_REVERSE_COUNTS      400
#define MOTOR_SPEED_PI_STALL_TICKS         6000U   /* 300 ms */
#define MOTOR_SPEED_PI_STALL_COUNTS        20
#define MOTOR_SPEED_PI_FILTER_ALPHA        0.60f
#define MOTOR_SPEED_PI_SAMPLE_COUNT       150U
/**
  * @}
  */

/** @defgroup FOC Portfolio Speed Profile (starts on RUN entry)
  * 500ms ramp from handoff RPM to 180RPM, 3000ms hold, 500ms ramp to 0.
  * 500ms maximum zero-speed settling; automatic PWM off on success/failure.
  * The ramp starts from the nonzero HANDOFF speed, NOT from standstill.
  * @{
  */
#define MOTOR_SPEED_PROFILE_ACCEL_MS         500U
#define MOTOR_SPEED_PROFILE_HOLD_MS         3000U
#define MOTOR_SPEED_PROFILE_DECEL_MS         500U
#define MOTOR_SPEED_PROFILE_STOP_TIMEOUT_MS  500U
#define MOTOR_SPEED_PROFILE_STOP_RPM          10.0f
#define MOTOR_SPEED_PROFILE_STOP_STEPS        3U  /* 3 x 10ms */
#define MOTOR_SPEED_PROFILE_BRAKE_IQ_A        1.20f /* limited braking at/near zero */
/**
  * @}
  */

/** @defgroup FOC Speed Startup, Handoff and Restart - K/L
  * @{
  */
#define MOTOR_SPEED_STARTUP_IQ_A           2.50f
#define MOTOR_SPEED_STARTUP_RAMP_TICKS     1600U   /* 80 ms */
#define MOTOR_SPEED_STARTUP_TIMEOUT_TICKS  3400U   /* 170 ms */
#define MOTOR_SPEED_START_DETECT_RPM       20.0f
#define MOTOR_SPEED_START_DETECT_COUNTS    25
#define MOTOR_SPEED_START_CONFIRM_STEPS    2U      /* 2 successive 10ms readings */
#define MOTOR_SPEED_HANDOFF_TICKS          2400U   /* 120 ms */
#define MOTOR_SPEED_HANDOFF_MIN_RPM        40.0f
#define MOTOR_SPEED_IQ_SLEW_A_PER_S        40.0f
#define MOTOR_SPEED_INTEGRAL_TRACK_GAIN    0.15f
#define MOTOR_SPEED_INTEGRAL_LIMIT_A       0.80f
/* HANDOFF overspeed guard (100 Hz; no change to RUN speed PI). */
#define MOTOR_SPEED_HANDOFF_OVERSPEED_MARGIN_RPM 30.0f
#define MOTOR_SPEED_HANDOFF_I_BLEED_A_PER_STEP  0.20f
#define MOTOR_SPEED_HANDOFF_BRAKE_MAX_A         0.35f
#define MOTOR_SPEED_RUN_STALL_STEPS        25U     /* 250ms below 5RPM */
#define MOTOR_SPEED_RESTART_MAX_ATTEMPTS   1U
#define MOTOR_SPEED_RESTART_RESERVE_TICKS  2000U  /* 100ms buffer */
#define MOTOR_SPEED_IQ_DECEL_SLEW_A_PER_S  180.0f
#define MOTOR_SPEED_STARTUP_SOFT_RPM       90.0f
#define MOTOR_SPEED_STARTUP_STOP_RPM       130.0f
#define MOTOR_SPEED_HANDOFF_EXIT_IQ_A       0.35f
/* HANDOFF: use a faster reference ramp only when startup ends below 60RPM.
 * Latched per handoff, so K startup can catch up while L retains 400RPM/s. */
#define MOTOR_SPEED_HANDOFF_FAST_THRESHOLD_RPM  60.0f
#define MOTOR_SPEED_HANDOFF_FAST_RAMP_RPM_PER_S    2000.0f
/* Handoff must establish real motion before entering RUN. */
#define MOTOR_SPEED_HANDOFF_RUN_MIN_RPM         30.0f
#define MOTOR_SPEED_HANDOFF_RUN_CONFIRM_STEPS   3U
/* After normal handoff dwell, allow one retry for a genuine near-zero stall.
 * Bound HANDOFF dwell to avoid TIME COMPLETE while stuck in transition. */
#define MOTOR_SPEED_HANDOFF_STALL_ABS_RPM        5.0f
#define MOTOR_SPEED_HANDOFF_STALL_STEPS          4U
#define MOTOR_SPEED_HANDOFF_MAX_TICKS         6000U  /* 300ms */
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
