/*
 * fixed_id_diag.h
 *
 *  Created on: 2026. 11. 5.
 *      Author: YDG
 */

#ifndef FIXED_ID_DIAG_H_
#define FIXED_ID_DIAG_H_

/* Values carried by existing motorOpenLoopDiagSetFixedOnly(uint8_t).
 * 0: S/R sweep; 1: F/N fixed Iq; 2: A fixed Id at 0; 3: B fixed Id at pi/2.
 * No change to the existing motor_ol_diag_t layout / motor.h is needed.
 */
#define OL_DIAG_MODE_SWEEP             0U
#define OL_DIAG_MODE_FIXED_IQ          1U
#define OL_DIAG_MODE_FIXED_ID_0        2U
#define OL_DIAG_MODE_FIXED_ID_90       3U

#define OL_FIXED_ID_REF_A              0.40f
#define OL_FIXED_ID_OC_LIMIT_A         0.80f
#define OL_FIXED_ID_HOLD_MS            100U
#define OL_FIXED_ID_HOLD_SAMPLES       ((OL_FIXED_ID_HOLD_MS * 1000U) / 50U)
#define OL_FIXED_ID_END_SAMPLES        (OPEN_LOOP_DIAG_SETTLE_SAMPLES + OPEN_LOOP_DIAG_RAMP_SAMPLES + OL_FIXED_ID_HOLD_SAMPLES)

#endif /* FIXED_ID_DIAG_H_ */
