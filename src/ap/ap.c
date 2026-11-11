/*
 * ap.c
 *
 *  Created on: 2026. 5. 26.
 *      Author: YDG
 */
#include "ap.h"
#include "util.h"

static float speed_target = 0.0f;

/* Keep the diagnostic snapshot in BSS rather than on the small MCU stack. */
static motor_align_diag_t completed_diag;
static motor_torque_probe_diag_t completed_torque_diag;
static motor_spin_probe_diag_t completed_spin_diag;
static motor_speed_pi_diag_t completed_speed_pi_diag;
static motor_vector_probe_diag_t completed_vector_diag;
static motor_offset_probe_diag_t completed_offset_diag;
/* UART selection: 1=-60, 2=-30, 3=0, 4=+30, 5=+60 electrical deg. */
static uint8_t offset_probe_choice = 2U; /* default 0 deg */

/* Specific ALIGN reason, printed only after PWM is disabled. */
static const char *apSpeedAlignFailReason(uint8_t mask)
{
  if (mask & 1U) return "MAX_EXCURSION";
  if (mask & 2U) return "MIN_RESPONSE";
  if (mask & 4U) return "RETURN_RESIDUAL";
  if (mask & 8U) return "FINAL_SETTLE";
  if (mask & 16U) return "STATIC_SETTLE";
  return "NONE";
}

static const char *apSpeedAlignFailStage(uint8_t stage)
{
  static const char * const names[] = {
      "STATIC", "RAMP", "UP_0_TO_90", "HOLD_90", "BACK_90_TO_0", "SETTLE_0"};
  return (stage < sizeof(names) / sizeof(names[0])) ? names[stage] : "UNKNOWN";
}

static int32_t apEncoderDelta(int32_t to, int32_t from)
{
  int32_t d = to - from;
  const int32_t n = (int32_t)ENCODER_MAX_COUNT;
  if (d > n / 2) d -= n;
  if (d < -(n / 2)) d += n;
  return d;
}

void apInit(void)
{
  uartOpen(_DEF_UART1, 57600);
}

void apMain(void)
{
  uint32_t slow_ms = millis();
  uint32_t print_ms = millis();
  uint32_t last_count = encoderGetCount();
#if MOTOR_ALLOW_POWER_STAGE == 0U
  uint32_t speed_prev_ms = millis();
#endif
  pwmDisableOutput();
  uartPrintf(_DEF_UART1, "\r\n=== ENCODER FOC CLEAN PROJECT ===\r\n");
  uartPrintf(_DEF_UART1, "TIM2 Encoder Mode initialized.\r\n");
#if MOTOR_ALLOW_POWER_STAGE == 0U
  uartPrintf(_DEF_UART1, "Motor PWM output LOCKED: MOTOR_ALLOW_POWER_STAGE=0.\r\n");
#else
  uartPrintf(_DEF_UART1, "WARNING: Power stage ENABLED; S can energize motor.\r\n");
#endif
  uartPrintf(_DEF_UART1, "Encoder monitor every 250ms; P=print, ?=help.\r\n");
  uartPrintf(_DEF_UART1, "POSITION FOC: 2A align (2s) -> Id=0 + settle -> +8/0/-8/0 count targets.\r\n");
  uartPrintf(_DEF_UART1, "POSITION FOC: filtered PD @1kHz, Iq <= 0.5A, auto PWM OFF, raw safety checks.\r\n");
  uartPrintf(_DEF_UART1, "F/R=FOC +/-0.30A; T/Y=FOC +/-3.50A 250cnt; G/H=FOC +/-3.50A 1rev; K/L=STARTUP+SPEED PI +/-180RPM; V=vector; S=PD.\r\n");
  uartPrintf(_DEF_UART1, "T/Y: 2A align -> Id=0 settle -> Iq 0->+/-3.5A->0 (100/150/100ms); move limit=250 counts.\r\n");
  uartPrintf(_DEF_UART1, "K/L: 2A align -> startup Iq ramp -> smooth handoff -> +/-180RPM speed PI (1.5s).\r\n");
  uartPrintf(_DEF_UART1,
      "X=stop, C=clear fault. V high Id=%ldmA -> low 2500mA at fixed +90deg, Iq=0.\r\n",
      (long)(VECTOR_PROBE_ID_A * 1000.0f));
  uartPrintf(_DEF_UART1, "OFFSET DIAG: 1=-60 2=-30 3=0 4=+30 5=+60 deg; P=+Iq, N=-Iq (300mA per separate run).\r\n");
  uartPrintf(_DEF_UART1,
      "DIAG: supply 12..16V, PWM 20kHz, DT=64 ticks, SW peak current limit=%ldmA.\r\n",
      (long)(CURRENT_TEST_OC_LIMIT_A * 1000.0f));
  uartPrintf(_DEF_UART1, "HARDWARE: verify JP1/JP2 CLOSED, J5/J6=3Sh, C3/C5/C7 removed for FOC.\r\n");

  while (1)
  {
    const uint32_t now = millis();
    float ratio = 0.0f;

#if MOTOR_ALLOW_POWER_STAGE == 0U
    uint32_t elapsed_ms = now - speed_prev_ms;
    if (elapsed_ms >= 10U)
    {
      speed_prev_ms = now;
      encoderUpdateSpeed((float)elapsed_ms * 0.001f);
    }
#endif
    if (now - slow_ms >= 100U)
    {
      slow_ms = now;
      motorLowSpeedTask();

      uint16_t raw = adcGetSpeedRaw();

      if (raw > SPEED_CMD_DEADBAND_RAW)
      {
          ratio = (float)(raw - SPEED_CMD_DEADBAND_RAW)
                       /(4095.0f - SPEED_CMD_DEADBAND_RAW);
      }

      speed_target = clampFloat(ratio, 0.0f, 1.0f) * OUTPUT_SPD_REF_MAX;

      motorSetSpeedReference(speed_target);
    }
    /* Capture summary only AFTER the ISR has disabled MOE. No UART in ISR. */
    if (motorGetState() == MOTOR_STATE_ALIGN_DIAG_DONE)
    {
      motor_align_diag_t * const diag_ptr = &completed_diag;
      if (motorIsOffsetProbeSelected())
      {
        motor_offset_probe_diag_t * const q = &completed_offset_diag;
        if (motorGetOffsetProbeDiag(q))
        {
          static const char * const names[MOTOR_OFFSET_PROBE_SAMPLE_COUNT] = {
            "ID_ZERO", "ZERO_SETTLED", "PULSE_START", "RAMP_20MS",
            "RAMP_50MS", "HOLD_START", "HOLD_MID", "HOLD_END",
            "DECAY_HALF", "FINAL"
          };
          uartPrintf(_DEF_UART1,
              "\r\n=== ENCODER ELECTRICAL OFFSET SWEEP (PWM OFF) ===\r\n");
          uartPrintf(_DEF_UART1,
              "extra_offset=%+d deg sign=%+d Iq_peak=%ldmA base_offset=%ldmrad "
              "align=%ld zero=%ld end=%ld moved=%+ld peak_abs=%ld "
              "result=%u fault=%u\r\n",
              (int)q->offset_deg, (int)q->direction,
              (long)(OFFSET_PROBE_IQ_A * 1000.0f),
              (long)(q->base_offset_rad * 1000.0f),
              (long)q->align_count, (long)q->zero_count,
              (long)q->end_count,
              (long)apEncoderDelta(q->end_count, q->zero_count),
              (long)q->peak_abs_counts,
              (unsigned int)q->result, (unsigned int)motorGetFault());
          uartPrintf(_DEF_UART1,
              "t_ms,stage,count,rel_count,theta_mrad,"
              "id_ref_mA,id_meas_mA,iq_ref_mA,iq_meas_mA,"
              "vd_mV,vq_mV,ia_mA,ib_mA,ic_mA,sum_mA\r\n");
          for (uint32_t i = 0U; i < MOTOR_OFFSET_PROBE_SAMPLE_COUNT; ++i)
          {
            if ((q->sample_mask & (1U << i)) == 0U) continue;
            const motor_offset_probe_sample_t *s = &q->samples[i];
            uartPrintf(_DEF_UART1,
                "%lu,%s,%ld,%+ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                (unsigned long)(s->tick / 20U), names[i],
                (long)s->encoder_count, (long)s->relative_count,
                (long)(s->theta_e * 1000.0f),
                (long)(s->id_ref * 1000.0f), (long)(s->id_meas * 1000.0f),
                (long)(s->iq_ref * 1000.0f), (long)(s->iq_meas * 1000.0f),
                (long)(s->vd * 1000.0f), (long)(s->vq * 1000.0f),
                (long)(s->ia * 1000.0f), (long)(s->ib * 1000.0f),
                (long)(s->ic * 1000.0f),
                (long)((s->ia + s->ib + s->ic) * 1000.0f));
          }
          uartPrintf(_DEF_UART1,
              "NOTE: extra_offset is diagnostic only, not saved. "
              "RESULT=1 means completed, not validated flux calibration.\r\n");
        }
      }
      else if (motorIsVectorProbeSelected())
      {
        motor_vector_probe_diag_t * const v = &completed_vector_diag;
        if (motorGetVectorProbeDiag(v))
        {
          static const char * const labels[MOTOR_VECTOR_PROBE_SAMPLE_COUNT] = {
              "ALIGN_END", "ID_HIGH_END", "UP_HALF", "PLUS_END",
              "HIGH_HOLD_MID", "HIGH_HOLD_END", "ID_DOWN_HALF",
              "ID_DOWN_END", "LOW_HOLD_150", "LOW_HOLD_MID",
              "LOW_HOLD_END", "RETURN_HALF", "BACK_ZERO",
              "ZERO_HOLD_END", "ID_OFF_HALF", "FINAL"
          };
          const float cmd_goal_mrad = VECTOR_PROBE_ANGLE_DEG * (PI / 180.0f) * 1000.0f;
          const float expected_count =
              -VECTOR_PROBE_ANGLE_DEG / 360.0f * (float)ENCODER_MAX_COUNT /
              (float)MOTOR_POLE_PAIRS; /* ENCODER_ELEC_SIGN=-1 */
          uartPrintf(_DEF_UART1,
              "\r\n=== STATOR VECTOR CURRENT HYSTERESIS DIAG (PWM OFF) ===\r\n");
          uartPrintf(_DEF_UART1,
              "Id_high=%ldmA, Id_low=2500mA, angle=0->+%ld->0mrad; "
              "HIGH_HOLD=%lums, LOW_HOLD=600ms; NO ENCODER ANGLE FEEDBACK\r\n",
              (long)(VECTOR_PROBE_ID_A * 1000.0f), (long)cmd_goal_mrad,
              (unsigned long)(VECTOR_PROBE_ANGLE_HOLD_TICKS / 20U));
          uartPrintf(_DEF_UART1,
              "align=%ld end=%ld final_delta=%+ld peak_abs=%ld counts expected_at_target=%ld milli-counts (ideal follow, sign=-1) result=%u fault=%u\r\n",
              (long)v->align_count, (long)v->end_count,
              (long)apEncoderDelta(v->end_count, v->align_count),
              (long)v->peak_abs_counts,
              (long)(expected_count * 1000.0f), /* milli-counts */
              (unsigned int)v->result, (unsigned int)motorGetFault());
          if (v->result == 3U)
          {
            uartPrintf(_DEF_UART1,
                "EARLY_EXCURSION_CUT: t_ms=%lu cmd_mrad=%ld cnt=%ld from_align=%+ld limit=%ld counts\r\n",
                (unsigned long)(v->cut_tick / 20U),
                (long)(v->cut_theta_cmd * 1000.0f),
                (long)v->cut_encoder_count,
                (long)apEncoderDelta(v->cut_encoder_count, v->align_count),
                (long)VECTOR_PROBE_MAX_MOVE_COUNTS);
          }
          uartPrintf(_DEF_UART1,
              "t_ms,stage,cmd_mrad,enc_count,rel_count,enc_delta_mrad,"
              "idref_mA,id2_mA,iqref_mA,iq2_mA,vd_mV,vq_mV,"
              "ia_mA,ib_mA,ic_mA,sum_mA,"
              "alpha2_mA,beta2_mA,alpha3_mA,beta3_mA,"
              "id3_mA,iq3_mA,ia_ideal_mA,ib_ideal_mA,ic_ideal_mA,"
              "dutyU_permille,dutyV_permille,dutyW_permille\r\n");
          for (uint32_t i = 0U; i < MOTOR_VECTOR_PROBE_SAMPLE_COUNT; ++i)
          {
            if ((v->sample_mask & (1U << i)) == 0U) continue;
            const motor_vector_probe_sample_t *s = &v->samples[i];
            const float theta_enc_delta = (float)s->relative_count *
                ENCODER_ELEC_SIGN * (2.0f * PI * (float)MOTOR_POLE_PAIRS) /
                (float)ENCODER_MAX_COUNT;
            /* Calculation is MAIN-LOOP ONLY, from the SAME captured ADC currents.
             * This has no effect on the original PI/Clarke feedback. */
            const float root3 = 1.7320508075688772f;
            const float a2 = s->ia;
            const float b2 = (s->ia + 2.0f * s->ib) / root3;
            const float a3 = (2.0f * s->ia - s->ib - s->ic) / 3.0f;
            const float b3 = (s->ib - s->ic) / root3;
            const float ct = cosf(s->theta_cmd);
            const float st = sinf(s->theta_cmd);
            const float id3 = a3 * ct + b3 * st;
            const float iq3 = -a3 * st + b3 * ct;
            /* Ideal balanced current is calculated using the commanded d/q
             * references; the measured d/q may deviate transiently. */
            const float ia_ideal = s->id_ref * ct - s->iq_ref * st;
            const float ib_ideal = -0.5f * ia_ideal +
                (root3 / 2.0f) * (s->id_ref * st + s->iq_ref * ct);
            const float ic_ideal = -ia_ideal - ib_ideal;
            uartPrintf(_DEF_UART1,
                "%lu,%s,%ld,%ld,%+ld,%+ld,"
                "%ld,%ld,%ld,%ld,%ld,%ld,"
                "%ld,%ld,%ld,%ld,"
                "%ld,%ld,%ld,%ld,"
                "%ld,%ld,%ld,%ld,%ld,"
                "%ld,%ld,%ld\r\n",
                (unsigned long)(s->tick / 20U), labels[i],
                (long)(s->theta_cmd * 1000.0f),
                (long)s->encoder_count, (long)s->relative_count,
                (long)(theta_enc_delta * 1000.0f),
                (long)(s->id_ref * 1000.0f), (long)(s->id_meas * 1000.0f),
                (long)(s->iq_ref * 1000.0f), (long)(s->iq_meas * 1000.0f),
                (long)(s->vd * 1000.0f), (long)(s->vq * 1000.0f),
                (long)(s->ia * 1000.0f), (long)(s->ib * 1000.0f),
                (long)(s->ic * 1000.0f),
                (long)((s->ia + s->ib + s->ic) * 1000.0f),
                (long)(a2 * 1000.0f), (long)(b2 * 1000.0f),
                (long)(a3 * 1000.0f), (long)(b3 * 1000.0f),
                (long)(id3 * 1000.0f), (long)(iq3 * 1000.0f),
                (long)(ia_ideal * 1000.0f),
                (long)(ib_ideal * 1000.0f),
                (long)(ic_ideal * 1000.0f),
                (long)(s->duty_u * 1000.0f),
                (long)(s->duty_v * 1000.0f),
                (long)(s->duty_w * 1000.0f));
          }
          /* Compare the same 2.5A at +90deg against the PRIOR standalone
           * 2.5A run. The alignment reference is the SAME for both stages
           * within this experiment, enabling an unambiguous shift reading. */
          if ((v->sample_mask & (1U << 5U)) &&
              (v->sample_mask & (1U << 10U)))
          {
            const int32_t high_end = v->samples[5].relative_count;
            const int32_t low_end  = v->samples[10].relative_count;
            const int32_t change  = low_end - high_end;
            uartPrintf(_DEF_UART1,
                "HYSTERESIS_RESULT: high_hold_rel=%+ld low_hold_rel=%+ld "
                "change_at_fixed_90deg=%+ld counts (positive = return toward zero if high_rel negative)\r\n",
                (long)high_end, (long)low_end, (long)change);
          }
          else
          {
            uartPrintf(_DEF_UART1,
                "HYSTERESIS_RESULT: incomplete snapshots (likely early cutoff); "
                "do NOT infer 2.5A settled position.\r\n");
          }
          uartPrintf(_DEF_UART1,
              "NOTE: 2sh is LIVE PI feedback; 3sh is offline diagnostic only. "
              "Ideal phases use commanded Id/Iq. No claim of physical phase mapping.\r\n");
          uartPrintf(_DEF_UART1,
              "RESULT: 1=sequence complete (NOT proof of field tracking), 2=align unstable, 3=excursion cut.\r\n");
        }
      }
      else if (motorIsSpeedPiProbeSelected())
      {
        motor_speed_pi_diag_t * const q = &completed_speed_pi_diag;
        if (motorGetSpeedPiProbeDiag(q))
        {
          static const char * const results[] = {
              "UNFINISHED", "TIME COMPLETE", "ALIGN UNSTABLE",
              "OVERSPEED CUT", "WRONG DIRECTION CUT", "STALL CUT",
              "TRAVEL CUT", "PRE-START EXCURSION CUT", "STARTUP TIMEOUT",
              "ALIGN VECTOR CHECK FAILED", "RESTART TIMEOUT"
          };
          const uint8_t rc = q->result;
          uartPrintf(_DEF_UART1,
              "\r\n=== ENCODER FOC SPEED PI (PWM OFF) ===\r\n");
          uartPrintf(_DEF_UART1,
              "direction=%+d target_signed_rpm=%ld iq_cap=%ldmA "
              "v_cap=%ldmV (speed only), align=%ld zero=%ld stop=%ld "
              "unwrapped=%+ld counts forward_max=%ld "
              "peak_raw_rpm=%ld stop_filtered_rpm=%ld "
              "voltage_sat_ms=%lu max_voltage_mV=%ld "
              "result=%u (%s) fault=%u\r\n",
              (int)q->direction,
              (long)((float)q->direction * MOTOR_SPEED_PI_GOAL_RPM),
              (long)(q->iq_limit_used * 1000.0f),
              (long)(MOTOR_SPEED_PI_V_LIMIT_V * 1000.0f),
              (long)q->align_count, (long)q->zero_count, (long)q->stop_raw,
              (long)q->total_counts, (long)q->maximum_forward_counts,
              (long)q->peak_abs_rpm, (long)q->stop_rpm,
              (unsigned long)(q->voltage_saturation_ticks / 20U),
              (long)(q->max_abs_vmag * 1000.0f),
              (unsigned int)rc,
              (rc < sizeof(results)/sizeof(results[0])) ? results[rc] : "UNKNOWN",
              (unsigned int)motorGetFault());
          uartPrintf(_DEF_UART1,
              "ALIGN_CHECK: pass=%u reason=%s mask=0x%02X stage=%s v_ms=%lu\r\n",
              (unsigned int)q->align_vector_pass,
              apSpeedAlignFailReason(q->align_fail_mask),
              (unsigned int)q->align_fail_mask,
              apSpeedAlignFailStage(q->align_fail_stage),
              (unsigned long)(q->align_fail_v_tick / 20U));
          uartPrintf(_DEF_UART1,
              "ALIGN_COUNTS: base=%ld plus=%ld plus_valid=%u plus_delta=%+ld\r\n",
              (long)q->align_base_count, (long)q->align_plus_count,
              (unsigned int)q->align_plus_valid, (long)q->align_plus_delta);
          uartPrintf(_DEF_UART1,
              "ALIGN_COUNTS: end_or_cut=%ld returned=%u residual=%+ld peak=%ld\r\n",
              (long)q->align_return_count, (unsigned int)q->align_return_valid,
              (long)q->align_return_delta, (long)q->align_peak_delta);
          uartPrintf(_DEF_UART1,
              "ALIGN_SETTLE: static_peak=%ld settle_base=%ld final_delta=%+ld settle_peak=%ld\r\n",
              (long)q->align_static_settle_peak_counts,
              (long)q->align_settle_base_count,
              (long)q->align_settle_final_delta,
              (long)q->align_settle_peak_counts);
          uartPrintf(_DEF_UART1,
              "ALIGN_LIMITS: response_min=%ld excursion_max=%ld residual_max=%ld final_settle_max=%ld\r\n",
              (long)MOTOR_SPEED_ALIGN_MIN_RESPONSE_COUNTS,
              (long)MOTOR_SPEED_ALIGN_MAX_EXCURSION_COUNTS,
              (long)MOTOR_SPEED_ALIGN_MAX_RESIDUAL_COUNTS,
              (long)MOTOR_SPEED_ALIGN_LAST_SETTLE_COUNTS);
          if (q->align_fail_mask != 0U)
            uartPrintf(_DEF_UART1,
                "ALIGN_FAIL_FLAGS: excursion=%u response=%u residual=%u settle=%u static=%u\r\n",
                (unsigned int)((q->align_fail_mask & 1U) != 0U),
                (unsigned int)((q->align_fail_mask & 2U) != 0U),
                (unsigned int)((q->align_fail_mask & 4U) != 0U),
                (unsigned int)((q->align_fail_mask & 8U) != 0U),
                (unsigned int)((q->align_fail_mask & 16U) != 0U));
          uartPrintf(_DEF_UART1,
              "STARTUP_HANDOFF: startup_end_ms=%lu handoff_end_ms=%lu "
              "start_rpm=%ld start_iq_mA=%ld stop_phase=%u "
              "(1=STARTUP,2=HANDOFF,3=RUN,4=RESTART)\r\n",
              (unsigned long)(q->startup_end_tick / 20U),
              (unsigned long)(q->handoff_end_tick / 20U),
              (long)q->rpm_at_startup_end,
              (long)(q->iq_at_startup_end * 1000.0f),
              (unsigned int)q->last_phase);
          uartPrintf(_DEF_UART1,
              "RESTART_INFO: attempts=%u restart_start_ms=%lu "
              "restart_recovered_ms=%lu (0=not attempted/recovered)\r\n",
              (unsigned int)q->restart_count,
              (unsigned long)(q->restart_start_tick / 20U),
              (unsigned long)(q->restart_recovered_tick / 20U));
          uartPrintf(_DEF_UART1,
              "t_ms,phase,raw_count,unwrapped_count,rpm_raw,rpm_lpf,rpm_ref,"
              "iq_ref_mA,iq_meas_mA,id_meas_mA,vd_mV,vq_mV,vmag_mV\r\n");
          for (uint32_t i=0U; i<q->sample_count && i<MOTOR_SPEED_PI_SAMPLE_COUNT; ++i)
          {
            const motor_speed_pi_sample_t *s = &q->samples[i];
            uartPrintf(_DEF_UART1,
                "%lu,%u,%ld,%+ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                (unsigned long)(s->tick / 20U), (unsigned int)s->phase,
                (long)s->raw_count,
                (long)s->unwrapped_count, (long)s->rpm_raw,
                (long)s->rpm_filtered, (long)s->rpm_ref,
                (long)(s->iq_ref * 1000.0f), (long)(s->iq_meas * 1000.0f),
                (long)(s->id_meas * 1000.0f),
                (long)(s->vd * 1000.0f), (long)(s->vq * 1000.0f),
                (long)(s->vmag * 1000.0f));
          }
          uartPrintf(_DEF_UART1,
              "NOTE: raw TIM2 counts invert the reported RPM sign; "
              "+Iq -> negative raw count -> positive RPM. "
              "No UART in 20kHz ISR. PWM OFF may coast.\r\n");
        }
      }
      else if (motorIsSpinProbeSelected())
      {
        motor_spin_probe_diag_t * const q = &completed_spin_diag;
        if (motorGetSpinProbeDiag(q))
        {
          static const char * const results[] = {
              "UNFINISHED", "TIME COMPLETE", "ALIGN UNSTABLE",
              "ONE REV CUT", "RPM CUT", "WRONG DIRECTION CUT",
              "STALL CUT", "PRE-START EXCURSION CUT"};
          const uint8_t rc = q->result;
          uartPrintf(_DEF_UART1,
              "\r\n=== ONE-REV ENCODER FOC SPIN TEST (PWM OFF) ===\r\n");
          uartPrintf(_DEF_UART1,
              "sign=%+d Iq_target=%ldmA align=%ld zero=%ld stop_raw=%ld "
              "unwrapped=%+ld counts mech_revs_milli=%+ld "
              "max_forward=%ld peak_abs=%ld peak_rpm=%ld "
              "result=%u (%s) fault=%u\r\n",
              (int)q->direction, (long)(MOTOR_SPIN_PROBE_IQ_A * 1000.0f),
              (long)q->align_count, (long)q->zero_count,
              (long)q->stop_raw_count, (long)q->total_counts,
              (long)((q->total_counts * 1000L) / (int32_t)ENCODER_MAX_COUNT),
              (long)q->maximum_forward_counts, (long)q->peak_abs_counts,
              (long)q->peak_abs_rpm, (unsigned int)rc,
              (rc < sizeof(results)/sizeof(results[0])) ? results[rc] : "UNKNOWN",
              (unsigned int)motorGetFault());
          uartPrintf(_DEF_UART1,
              "stop_ms_after_align=%lu stop_raw=%ld stop_unwrapped=%+ld stop_rpm=%ld\r\n",
              (unsigned long)(q->stop_tick / 20U), (long)q->stop_raw_count,
              (long)q->stop_unwrapped_count, (long)q->last_rpm);
          uartPrintf(_DEF_UART1,
              "t_ms_from_Iq,raw_count,unwrapped_count,rpm_10ms,theta_e_mrad,Id_ref_mA,Id_meas_mA,Iq_ref_mA,Iq_meas_mA,Vd_mV,Vq_mV\r\n");
          for (uint32_t i=0U; i<q->sample_count && i<MOTOR_SPIN_PROBE_SAMPLE_COUNT; ++i)
          {
            const motor_spin_probe_sample_t *s = &q->samples[i];
            uartPrintf(_DEF_UART1,
                "%lu,%ld,%+ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                (unsigned long)(s->tick / 20U),
                (long)s->raw_count, (long)s->unwrapped_count,
                (long)s->rpm_meas, (long)(s->theta_e * 1000.0f),
                (long)(s->id_ref * 1000.0f), (long)(s->id_meas * 1000.0f),
                (long)(s->iq_ref * 1000.0f), (long)(s->iq_meas * 1000.0f),
                (long)(s->vd * 1000.0f), (long)(s->vq * 1000.0f));
          }
          uartPrintf(_DEF_UART1,
              "NOTE: RPM is an independent 10ms encoder derivative, NOT speed PI; "
              "1rev cut uses signed unwrapped counts. PWM OFF != shaft stopped.\r\n");
        }
      }
      else if (motorGetTorqueProbeDirection() != 0)
      {
        motor_torque_probe_diag_t * const q = &completed_torque_diag;
        if (motorGetTorqueProbeDiag(q))
        {
          static const char * const labels[MOTOR_TORQUE_PROBE_SAMPLE_COUNT] = {
            "ID_ZERO", "TORQUE_START", "RAMP_HALF", "HOLD_START",
            "HOLD_MID", "HOLD_END", "DECAY_HALF", "FINAL"
          };
          uartPrintf(_DEF_UART1,
              "\r\n=== FIXED IQ ENCODER FOC PROBE (PWM OFF) ===\r\n");
          uartPrintf(_DEF_UART1,
              "mode=%s sign=%+d Iq_peak=%ldmA limit=%ld counts align=%ld zero=%ld end=%ld moved=%+ld peak_abs=%ld counts result=%u fault=%u\r\n",
              q->high_current ? "HIGH_ENCODER_FOC" : "LOW_ENCODER_FOC",
              (int)q->direction,
              (long)((q->high_current ? MOTOR_HIGH_IQ_PROBE_A : TORQUE_PROBE_IQ_A) * 1000.0f),
              (long)q->move_limit_counts,
              (long)q->align_count, (long)q->zero_count, (long)q->end_count,
              (long)apEncoderDelta(q->end_count, q->zero_count),
              (long)q->peak_abs_counts, (unsigned int)q->result,
              (unsigned int)motorGetFault());
          if (q->result == 3U)
          {
            uartPrintf(_DEF_UART1,
                "EARLY_EXCURSION_CUT: t_ms=%lu count=%ld rel=%+ld limit=%ld (Iq diagnostic)\r\n",
                (unsigned long)(q->cut_tick / 20U), (long)q->cut_count,
                (long)q->cut_rel_count, (long)q->move_limit_counts);
          }
          uartPrintf(_DEF_UART1,
              "t_ms,stage,count,relative_count,theta_e_mrad,Id_ref_mA,Id_meas_mA,Iq_ref_mA,Iq_meas_mA,Vq_mV\r\n");
          for (uint32_t i = 0U; i < MOTOR_TORQUE_PROBE_SAMPLE_COUNT; ++i)
          {
            if ((q->sample_mask & (1U << i)) == 0U) continue;
            const motor_torque_probe_sample_t *s = &q->samples[i];
            uartPrintf(_DEF_UART1,
                "%lu,%s,%ld,%+ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                (unsigned long)(s->tick / 20U), labels[i],
                (long)s->encoder_count, (long)s->position_counts,
                (long)(s->theta_e * 1000.0f),
                (long)(s->id_ref * 1000.0f), (long)(s->id_meas * 1000.0f),
                (long)(s->iq_ref * 1000.0f), (long)(s->iq_meas * 1000.0f),
                (long)(s->vq * 1000.0f));
          }
          uartPrintf(_DEF_UART1,
              "RESULT: 1=sequence complete, 2=alignment unstable, 3=excursion cut (not a pass/fail torque calibration).\r\n");
        }
      }
      else if (motorGetAlignDiag(diag_ptr))
      {
        /* The summary is printed after the PWM has been switched off. */
        uartPrintf(_DEF_UART1,
            "\r\n=== STATIC ALIGN + ENCODER POSITION FOC (PWM OFF) ===\r\n");
        uartPrintf(_DEF_UART1,
            "START=%ld ALIGNED=%ld POS_ZERO=%ld END=%ld mask=0x%lX stages=0x%X\r\n",
            (long)diag_ptr->start_count, (long)diag_ptr->align_count,
            (long)diag_ptr->position_zero_count, (long)diag_ptr->end_count,
            (unsigned long)diag_ptr->valid_mask,
            (unsigned int)diag_ptr->completed_stages);
        uartPrintf(_DEF_UART1,
            "ALIGN_SETTLE=%ld counts (max=%d) offset=%ldmrad SIGN=%ld\r\n",
            (long)diag_ptr->settle_peak_counts, (int)ALIGN_SETTLE_MAX_COUNTS,
            (long)(diag_ptr->encoder_offset_rad * 1000.0f),
            (long)ENCODER_ELEC_SIGN);
        static const char * const sample_names[MOTOR_DIAG_SAMPLE_COUNT] = {
          "ALIGN_RAMP", "ALIGN_HOLD", "ALIGN_STABLE", "ALIGN_END",
          "ID_ZERO", "POS_ZERO", "PLUS_RAMP_END", "PLUS_TARGET_END",
          "HOME1_END", "MINUS_TARGET_END", "HOME2_END", "FINAL_ZERO"
        };
        for (uint32_t i = 0U; i < MOTOR_DIAG_SAMPLE_COUNT; ++i)
        {
          if ((diag_ptr->valid_mask & (1U << i)) == 0U) continue;
          const motor_align_diag_sample_t *s = &diag_ptr->samples[i];
          uartPrintf(_DEF_UART1,
              "S%lu %s count=%ld tick=%lu theta=%ldmrad Id=%ld/%ldmA Iq=%ld/%ldmA\r\n",
              (unsigned long)i, sample_names[i], (long)s->encoder_count,
              (unsigned long)s->tick, (long)(s->theta_e * 1000.0f),
              (long)(s->id_ref * 1000.0f), (long)(s->id_meas * 1000.0f),
              (long)(s->iq_ref * 1000.0f), (long)(s->iq_meas * 1000.0f));
          uartPrintf(_DEF_UART1,
              "S%lu POS target=%ld measured=%ld err=%ld milli-counts Vd/Vq=%ld/%ldmV\r\n",
              (unsigned long)i,
              (long)(s->pos_ref_counts * 1000.0f),
              (long)(s->pos_meas_counts * 1000.0f),
              (long)(s->pos_error_counts * 1000.0f),
              (long)(s->vd * 1000.0f), (long)(s->vq * 1000.0f));
          uartPrintf(_DEF_UART1,
              "S%lu PD raw=%ld filt=%ld vel=%ldmc/s P/D=%ld/%ldmA\r\n",
              (unsigned long)i, (long)(s->pos_raw_counts * 1000.0f),
              (long)(s->pos_meas_counts * 1000.0f),
              (long)(s->pos_vel_counts_per_s * 1000.0f),
              (long)(s->pos_p_iq_a * 1000.0f),
              (long)(s->pos_d_iq_a * 1000.0f));
          uartPrintf(_DEF_UART1,
              "S%lu Ia/Ib/Ic=%ld/%ld/%ldmA SUM=%ldmA duty=%ld/%ld/%ld CCER=0x%08lX MOE=%u\r\n",
              (unsigned long)i,
              (long)(s->ia * 1000.0f), (long)(s->ib * 1000.0f),
              (long)(s->ic * 1000.0f),
              (long)((s->ia + s->ib + s->ic) * 1000.0f),
              (long)(s->duty_u * 1000.0f),
              (long)(s->duty_v * 1000.0f),
              (long)(s->duty_w * 1000.0f),
              (unsigned long)s->ccer, (unsigned int)s->moe);
        }
        if ((diag_ptr->valid_mask & (1U << 10U)) != 0U)
        {
          uartPrintf(_DEF_UART1,
              "POS endpoints raw: zero=%ld plus=%ld home1=%ld minus=%ld home2=%ld\r\n",
              (long)diag_ptr->position_zero_count,
              (long)diag_ptr->plus_end_count, (long)diag_ptr->home1_end_count,
              (long)diag_ptr->minus_end_count, (long)diag_ptr->home2_end_count);
          uartPrintf(_DEF_UART1,
              "POS endpoints relative: +goal=%+ld home1=%+ld -goal=%+ld home2=%+ld counts\r\n",
              (long)apEncoderDelta(diag_ptr->plus_end_count, diag_ptr->position_zero_count),
              (long)apEncoderDelta(diag_ptr->home1_end_count, diag_ptr->position_zero_count),
              (long)apEncoderDelta(diag_ptr->minus_end_count, diag_ptr->position_zero_count),
              (long)apEncoderDelta(diag_ptr->home2_end_count, diag_ptr->position_zero_count));
        }
        uartPrintf(_DEF_UART1,
            "TRAVEL_DIAG_FILTERED_1KHZ=%ld (reference=%d, NO TRIP, deadband=%d); RAW_TRAVEL_20KHZ=%ld counts\r\n",
            (long)diag_ptr->accumulated_travel_counts,
            (int)POSITION_MAX_TRAVEL_COUNTS,
            (int)POSITION_TRAVEL_DEADBAND_COUNTS,
            (long)diag_ptr->raw_accumulated_travel_counts);
        uartPrintf(_DEF_UART1,
            "ZERO_IQ_1KHZ variation=%ld net=%+ld peak_step_1ms=%ld counts (Id=Iq=0, PWM ON)\r\n",
            (long)diag_ptr->zero_iq_variation_1khz,
            (long)diag_ptr->zero_iq_net_count,
            (long)diag_ptr->zero_iq_peak_1ms_step);
        uartPrintf(_DEF_UART1,
            "DRIVE_1KHZ variation=%ld peak_step_1ms=%ld counts; RAW_FILTER_PEAK=%ld milli-counts\r\n",
            (long)diag_ptr->drive_variation_1khz,
            (long)diag_ptr->drive_peak_1ms_step,
            (long)diag_ptr->peak_raw_filter_gap_millicounts);
        uartPrintf(_DEF_UART1,
            "POSITION peak_filtered_err=%ld peak_raw_err=%ld counts; instant_movement_limit=%d counts\r\n",
            (long)diag_ptr->peak_abs_error_counts,
            (long)diag_ptr->peak_abs_raw_error_counts,
            (int)POSITION_MAX_MOVE_COUNTS);
        uartPrintf(_DEF_UART1,
            "RESULT=%u (1=SEQUENCE COMPLETE, 2=ALIGN UNSTABLE, 3=INSTANT EXCURSION CUT, 4=POSITION ERROR CUT)\r\n",
            (unsigned int)motorGetAlignGateStatus());
        uartPrintf(_DEF_UART1,
            "NOTE: COMPLETE does not prove accurate position tracking. Check signed endpoint error and actual movement.\r\n");
      }
      motorStop();
      uartPrintf(_DEF_UART1, "COMMISSIONING FINISHED; PWM=OFF, state=IDLE.\r\n\r\n");
    }
    if (uartAvailable(_DEF_UART1))
    {
      uint8_t key = uartRead(_DEF_UART1);
      uartPrintf(_DEF_UART1, "RX KEY = %c\r\n", key);

      if (key == 'S')
      {
        if (motorGetState() == MOTOR_STATE_IDLE && motorGetFault() == MOTOR_FAULT_NONE)
        {
          (void)motorSelectTorqueProbe(0);
          uartPrintf(_DEF_UART1, "START POSITION COMMAND\r\n");
          motorStart();
        }
      }
      else if (key == 'V')
      {
        if (motorGetState() == MOTOR_STATE_IDLE && motorGetFault() == MOTOR_FAULT_NONE)
        {
          if (motorSelectVectorProbe())
          {
            uartPrintf(_DEF_UART1,
                "START STATOR-VECTOR HYSTERESIS 3.5A(+90 HOLD)->2.5A(+90 HOLD)->0deg; no encoder angle feedback\r\n");
            motorStart();
          }
        }
      }
      else if (key == 'K' || key == 'L')
      {
        if (motorGetState() == MOTOR_STATE_IDLE && motorGetFault() == MOTOR_FAULT_NONE)
        {
          const int8_t sign = (key == 'K') ? +1 : -1;
          if (motorSelectSpeedPiProbe(sign))
          {
            uartPrintf(_DEF_UART1,
                "START ENCODER FOC STARTUP+SPEED PI sign=%+d goal=%ldRPM "
                "Iq_limit=%ldmA voltage_limit=%ldmV (speed mode only) "
                "startup target=3000mA, handoff, run=1500ms; auto PWM OFF\r\n",
                (int)sign, (long)((float)sign * MOTOR_SPEED_PI_GOAL_RPM),
                (long)(fminf(MOTOR_SPEED_PI_IQ_MAX_A,
                    CURRENT_TEST_OC_LIMIT_A - MOTOR_SPEED_PI_OC_MARGIN_A)*1000.0f),
                (long)(MOTOR_SPEED_PI_V_LIMIT_V*1000.0f));
            motorStart();
          }
          else
          {
            uartPrintf(_DEF_UART1,
                "SPEED PI selection refused: idle/fault/OC config.\r\n");
          }
        }
      }
      else if (key == 'G' || key == 'H')
      {
        if (motorGetState() == MOTOR_STATE_IDLE && motorGetFault() == MOTOR_FAULT_NONE)
        {
          const int8_t sign = (key == 'G') ? +1 : -1;
          if (motorSelectSpinProbe(sign))
          {
            uartPrintf(_DEF_UART1,
                "START ONE-REV ENCODER FOC SPIN sign=%+d Iq=%ldmA "
                "(0.1s up/0.9s hold/0.1s down, 4000 counts, no speed PI)\r\n",
                (int)sign, (long)(MOTOR_SPIN_PROBE_IQ_A * 1000.0f));
            motorStart();
          }
        }
      }
      else if (key == 'T' || key == 'Y')
      {
        if (motorGetState() == MOTOR_STATE_IDLE && motorGetFault() == MOTOR_FAULT_NONE)
        {
          const int8_t sign = (key == 'T') ? +1 : -1;
          if (motorSelectHighIqProbe(sign))
          {
            uartPrintf(_DEF_UART1,
                "START HIGH-IQ ENCODER FOC sign=%+d Iq=%ldmA (NO position PD; auto PWM OFF)\r\n",
                (int)sign, (long)(MOTOR_HIGH_IQ_PROBE_A * 1000.0f));
            motorStart();
          }
        }
      }
      else if (key == 'F' || key == 'R')
      {
        if (motorGetState() == MOTOR_STATE_IDLE && motorGetFault() == MOTOR_FAULT_NONE)
        {
          const int8_t sign = (key == 'F') ? +1 : -1;
          if (motorSelectTorqueProbe(sign))
          {
            uartPrintf(_DEF_UART1, "START FIXED-IQ PROBE sign=%+d (no position PD)\r\n", (int)sign);
            motorStart();
          }
        }
      }
      else if (key >= '1' && key <= '5')
      {
        if (motorGetState() == MOTOR_STATE_IDLE)
        {
          offset_probe_choice = (uint8_t)(key - '1');
          const int32_t deg = ((int32_t)offset_probe_choice - 2) *
                              OFFSET_PROBE_STEP_DEG;
          uartPrintf(_DEF_UART1, "OFFSET SELECTED %+ld electrical deg (not applied until P/N).\r\n",
                     (long)deg);
        }
      }
      else if (key == 'P' || key == 'N')
      {
        if (motorGetState() == MOTOR_STATE_IDLE &&
            motorGetFault() == MOTOR_FAULT_NONE)
        {
          const int8_t sign = (key == 'P') ? +1 : -1;
          if (motorSelectOffsetProbe(sign, offset_probe_choice))
          {
            const int32_t deg = ((int32_t)offset_probe_choice - 2) *
                                OFFSET_PROBE_STEP_DEG;
            uartPrintf(_DEF_UART1,
                "START OFFSET PROBE extra_offset=%+ld deg Iq=%+ldmA (one run, auto PWM OFF)\r\n",
                (long)deg, (long)((float)sign * OFFSET_PROBE_IQ_A * 1000.0f));
            motorStart();
          }
        }
      }
      else if (key == 'X')
      {
        if (motorGetState() != MOTOR_STATE_FAULT)
        {
          uartPrintf(_DEF_UART1, "STOP COMMAND\r\n");
          motorStop();
        }
      }
      else if (key == 'C')
      {
        motorClearFault();
      }
    }
    if (now - print_ms >= 250U)
    {
      print_ms = now;
      const uint32_t count = encoderGetCount();
      int32_t delta = (int32_t)(count - last_count);
      if (delta > 2000)
          delta -= 4000;
      else if (delta < -2000)
          delta += 4000;
      last_count = count;

      uartPrintf(_DEF_UART1,
          "ENC count=%lu delta=%ld angle_m/e=%ld/%ld mrad speed_m/e=%ld/%ld mrad/s MOE=%u state=%u fault=%u\r\n",
          (unsigned long)count, (long)delta,
          (long)(encoderGetMechanicalAngle()*1000.0f),
          (long)(encoderGetElectricalAngle()*1000.0f),
          (long)(encoderGetMechanicalSpeed()*1000.0f),
          (long)(encoderGetElectricalSpeed()*1000.0f),
          (unsigned int)((TIM1->BDTR & TIM_BDTR_MOE) != 0U),
          (unsigned int)motorGetState(), (unsigned int)motorGetFault());
      // Fault diagnostics are printed only at the 250 ms monitor rate.
      if (motorGetState() == MOTOR_STATE_FAULT)
      {
        pwm_break_snapshot_t snap;
        if (pwmGetBreakSnapshot(&snap))
        {
          uartPrintf(_DEF_UART1,
              "BKIN SNAP: SR=0x%08lX BDTR=0x%08lX PA6=%lu\r\n",
              (unsigned long)snap.sr,
              (unsigned long)snap.bdtr,
              (unsigned long)((snap.gpioa_idr >> 6) & 1U));
        }
      }
      uartPrintf(_DEF_UART1,
          "BKIN DEBUG stage=%u SR=0x%08lX BDTR=0x%08lX PA6=%lu bdtr_before=%lu sr_before=%lu\r\n",
          (unsigned int)debug_bkin_stage,
          (unsigned long)debug_bkin_sr,
          (unsigned long)debug_bkin_bdtr,
          (unsigned long)debug_bkin_pa6,
          (unsigned long)bdtr_before,
          (unsigned long)sr_before);
      uartPrintf(_DEF_UART1,
          "ADC DEBUG stage=%u\r\n",
          (unsigned int)debug_adc_stage);
      motor_monitor_t mon;

      if (motorGetMonitor(&mon))
      {
        uartPrintf(_DEF_UART1,
            "Id_ref=%ld Id_meas=%ld Iq_ref=%ld Iq_meas=%ld mA Vd=%ld mV\r\n",
            (long)(mon.id_ref * 1000.0f),
            (long)(mon.id_meas * 1000.0f),
            (long)(mon.iq_ref * 1000.0f),
            (long)(mon.iq_meas * 1000.0f),
            (long)(mon.vd_cmd * 1000.0f));
      }
    }
    delay(1U);
  }
}
