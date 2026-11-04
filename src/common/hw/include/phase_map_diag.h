/*
 * motor.h
 *
 *  Created on: 2026. 11. 5.
 *      Author: YDG
 */

#ifndef SRC_COMMON_HW_INCLUDE_PHASE_MAP_DIAG_H_
#define SRC_COMMON_HW_INCLUDE_PHASE_MAP_DIAG_H_

#include "hw_def.h"

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP) && (OPEN_LOOP_DIAG_ENABLE == 1U)

// SAFE STARTING POINT ONLY.  Actual winding voltage can differ due to deadtime.
// Runs 6 short pulses (U+, U-, V+, V-, W+, W-) at 20 kHz.
#define PHASE_MAP_VOLTAGE_V             0.30f
#define PHASE_MAP_OC_LIMIT_A            0.80f
#define PHASE_MAP_BASELINE_SAMPLES      120U  // 6.0 ms, all duties 50%
#define PHASE_MAP_PULSE_SAMPLES         8U    // 0.4 ms, one phase differential
#define PHASE_MAP_RELAX_SAMPLES         300U  // 15 ms, all duties 50%
#define PHASE_MAP_TEST_COUNT            6U
#define PHASE_MAP_FRAME_SAMPLES         (PHASE_MAP_BASELINE_SAMPLES + PHASE_MAP_PULSE_SAMPLES + PHASE_MAP_RELAX_SAMPLES)
#define PHASE_MAP_TOTAL_SAMPLES         (PHASE_MAP_TEST_COUNT * PHASE_MAP_FRAME_SAMPLES)
#define PHASE_MAP_TEST_TIMEOUT_MS       250U

#if PHASE_MAP_PULSE_SAMPLES < 6U
#error "Need at least six ADC samples per phase-map pulse"
#endif

typedef struct
{
  float base_a, base_b, base_c;
  float pulse_a, pulse_b, pulse_c;
  uint32_t base_count, pulse_count;
} phase_map_measure_t;

typedef struct
{
  uint32_t samples;
  uint8_t complete;
  uint8_t hall_moved;
  uint8_t oc_trip;
  uint8_t bkin_trip;
  uint8_t moe_lost;
  uint8_t hall_start;
  float max_abs_current;
  float vbus_at_start;
  phase_map_measure_t measures[PHASE_MAP_TEST_COUNT];
} phase_map_diag_result_t;

void motorPhaseMapDiagSelect(void);  // Only while IDLE, before motorStart()
bool motorPhaseMapDiagIsSelected(void);
bool motorPhaseMapDiagIsDone(void);
void motorPhaseMapDiagGet(phase_map_diag_result_t *out);

#endif
#endif
