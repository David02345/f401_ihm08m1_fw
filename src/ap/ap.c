/* K/L speed PI tuning & portfolio CSV output only. */
#include "ap.h"
#include "util.h"

/* Large diagnostic snapshot stays in BSS, not on the Cortex-M stack. */
static motor_speed_pi_diag_t completed_speed_pi_diag;
extern volatile uint8_t motor_bkin_test_stage;
extern volatile uint32_t motor_bkin_test_sr;
extern volatile uint32_t motor_bkin_test_bdtr;
extern volatile uint32_t motor_bkin_test_pa6;
extern volatile uint8_t motor_bkin_test_latched;

void apInit(void)
{
  uartOpen(_DEF_UART1, 57600);
}

static void apPrintSpeedResult(void)
{
  motor_speed_pi_diag_t * const q = &completed_speed_pi_diag;
  if (!motorGetSpeedPiProbeDiag(q)) return;
  static const char * const results[] = {
      "UNFINISHED", "PROFILE STOPPED", "ALIGN UNSTABLE",
      "OVERSPEED CUT", "WRONG DIRECTION CUT", "STALL CUT",
      "TRAVEL CUT", "PRE-START EXCURSION CUT", "STARTUP TIMEOUT",
      "ALIGN VECTOR CHECK FAILED", "RESTART TIMEOUT", "PROFILE STOP TIMEOUT"};
  const uint8_t rc = q->result;
  uartPrintf(_DEF_UART1, "\r\n=== ENCODER FOC TRAPEZOID SPEED (PWM OFF) ===\r\n");
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
      (rc < sizeof(results) / sizeof(results[0])) ? results[rc] : "UNKNOWN",
      (unsigned int)motorGetFault());
  uartPrintf(_DEF_UART1,
      "ALIGN_CHECK: mode=STATIC_ONLY vector_sweep=SKIPPED "
      "static_pass=%u static_peak=%ld static_limit=%u\r\n",
      (unsigned int)(q->align_static_settle_peak_counts <= ALIGN_SETTLE_MAX_COUNTS),
      (long)q->align_static_settle_peak_counts,
      (unsigned int)ALIGN_SETTLE_MAX_COUNTS);
  uartPrintf(_DEF_UART1,
      "ALIGN_STATIC: final_count=%ld (encoder offset installed on success; "
      "no 3.5A rotation check)\r\n", (long)q->align_base_count);
  uartPrintf(_DEF_UART1,
      "ANGLE_DIAG: align_begin=%ld ramp_end=%ld align_final=%ld "
      "startup_zero=%ld raw_theta_mrad=%ld offset_mrad=%ld "
      "theta_after_offset_mrad=%ld theta_at_start_mrad=%ld\r\n",
      (long)q->diag_align_begin_count, (long)q->diag_align_ramp_count,
      (long)q->diag_align_final_count, (long)q->diag_start_zero_count,
      (long)(q->diag_align_raw_theta * 1000.0f),
      (long)(q->diag_offset_theta * 1000.0f),
      (long)(q->diag_calibrated_theta * 1000.0f),
      (long)(q->diag_start_theta * 1000.0f));
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
      "PROFILE: start_ms=%lu start_ref_rpm=%ld accel_ms=%u "
      "hold_ms=%u decel_ms=%u stop_timeout_ms=%u "
      "(profile starts at RUN, from nonzero handoff RPM)\r\n",
      (unsigned long)(q->profile_start_tick / 20U),
      (long)q->profile_initial_rpm,
      (unsigned int)MOTOR_SPEED_PROFILE_ACCEL_MS,
      (unsigned int)MOTOR_SPEED_PROFILE_HOLD_MS,
      (unsigned int)MOTOR_SPEED_PROFILE_DECEL_MS,
      (unsigned int)MOTOR_SPEED_PROFILE_STOP_TIMEOUT_MS);
  uartPrintf(_DEF_UART1,
      "t_ms,phase,profile_segment,raw_count,unwrapped_count,rpm_raw,rpm_lpf,rpm_ref,"
      "iq_ref_mA,iq_meas_mA,id_meas_mA,vd_mV,vq_mV,vmag_mV,theta_e_mrad\r\n");
  for (uint32_t i=0U; i<q->sample_count && i<MOTOR_SPEED_PI_SAMPLE_COUNT; ++i)
  {
    const motor_speed_pi_sample_t *s = &q->samples[i];
    uartPrintf(_DEF_UART1,
        "%lu,%u,%u,%ld,%+ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
        (unsigned long)(s->tick / 20U), (unsigned int)s->phase,
        (unsigned int)s->profile_segment,
        (long)s->raw_count, (long)s->unwrapped_count,
        (long)s->rpm_raw, (long)s->rpm_filtered, (long)s->rpm_ref,
        (long)(s->iq_ref * 1000.0f), (long)(s->iq_meas * 1000.0f),
        (long)(s->id_meas * 1000.0f),
        (long)(s->vd * 1000.0f), (long)(s->vq * 1000.0f),
        (long)(s->vmag * 1000.0f),
        (long)(s->theta_e * 1000.0f));
  }
  uartPrintf(_DEF_UART1,
      "NOTE: raw TIM2 counts invert RPM sign. No UART in 20kHz ISR. "
      "PWM OFF may coast.\r\n");
}

void apMain(void)
{
  uint32_t slow_ms = millis();
  uint32_t print_ms = millis();
  uint32_t last_count = encoderGetCount();
  bool fault_reported = false;
#if MOTOR_ALLOW_POWER_STAGE == 0U
  uint32_t speed_prev_ms = millis();
#endif
  pwmDisableOutput();
  uartPrintf(_DEF_UART1,
      "\r\n=== STM32F401 ENCODER FOC K/L SPEED PROFILE ===\r\n");
#if MOTOR_ALLOW_POWER_STAGE == 0U
  uartPrintf(_DEF_UART1, "PWM LOCKED by MOTOR_ALLOW_POWER_STAGE=0\r\n");
#else
  uartPrintf(_DEF_UART1,
      "WARNING: Power stage enabled. K=+180RPM, L=-180RPM, X=STOP, C=CLEAR FAULT\r\n");
#endif
  uartPrintf(_DEF_UART1,
      "Profile: RUN 500ms transition, 3000ms hold, 500ms decel; "
      "max 500ms stop check. UART 57600 baud.\r\n");
  while (1)
  {
    const uint32_t now = millis();
#if MOTOR_ALLOW_POWER_STAGE == 0U
    const uint32_t elapsed_ms = now - speed_prev_ms;
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
    }
    /* Always fetch the ISR summary before resetting the selection. */
    if (motorGetState() == MOTOR_STATE_ALIGN_DIAG_DONE)
    {
      apPrintSpeedResult();
      motorStop();
      uartPrintf(_DEF_UART1, "COMMISSIONING FINISHED; PWM=OFF, state=IDLE.\r\n\r\n");
    }
    if (uartAvailable(_DEF_UART1))
    {
      const uint8_t key = uartRead(_DEF_UART1);
      if (key == 'K' || key == 'L')
      {
        if (motorGetState() == MOTOR_STATE_IDLE && motorGetFault() == MOTOR_FAULT_NONE)
        {
          const int8_t sign = (key == 'K') ? +1 : -1;
          if (motorSelectSpeedPiProbe(sign))
          {
            uartPrintf(_DEF_UART1,
                "START ENCODER FOC sign=%+d goal=%ldRPM, "
                "startup=%ldmA, profile=%u/%u/%ums, auto PWM OFF\r\n",
                (int)sign, (long)((float)sign * MOTOR_SPEED_PI_GOAL_RPM),
                (long)(MOTOR_SPEED_STARTUP_IQ_A * 1000.0f),
                (unsigned int)MOTOR_SPEED_PROFILE_ACCEL_MS,
                (unsigned int)MOTOR_SPEED_PROFILE_HOLD_MS,
                (unsigned int)MOTOR_SPEED_PROFILE_DECEL_MS);
            motorStart();
          }
        }
        else uartPrintf(_DEF_UART1, "BUSY/FAULT: use X or C first\r\n");
      }
      else if (key == 'X')
      {
        if (motorGetState() != MOTOR_STATE_FAULT)
        {
          motorStop();
          uartPrintf(_DEF_UART1, "STOP COMMAND, PWM OFF\r\n");
        }
      }
      else if (key == 'C')
      {
        motorClearFault();
        fault_reported = false;
        uartPrintf(_DEF_UART1, "FAULT CLEAR requested\r\n");
      }
    }
    if (motorGetState() == MOTOR_STATE_FAULT && !fault_reported)
    {
      fault_reported = true;
      uartPrintf(_DEF_UART1, "FAULT code=%u state=%u, PWM OFF\r\n",
          (unsigned int)motorGetFault(), (unsigned int)motorGetState());
      if (motorGetFault() == MOTOR_FAULT_BKIN)
        uartPrintf(_DEF_UART1,
            "BKIN TEST: stage=%u PA6=%lu BIF=%u MOE=%u SR=0x%08lX BDTR=0x%08lX\r\n",
            (unsigned int)motor_bkin_test_stage,
            (unsigned long)((GPIOA->IDR >> 6) & 1U),
            (unsigned int)((TIM1->SR & TIM_SR_BIF) != 0U),
            (unsigned int)((TIM1->BDTR & TIM_BDTR_MOE) != 0U),
            (unsigned long)TIM1->SR, (unsigned long)TIM1->BDTR);
      if (motor_bkin_test_latched)
        uartPrintf(_DEF_UART1,
            "BKIN AT ENABLE: PA6=%lu BIF=%u MOE=%u SR=0x%08lX BDTR=0x%08lX\r\n",
            (unsigned long)motor_bkin_test_pa6,
            (unsigned int)((motor_bkin_test_sr & TIM_SR_BIF) != 0U),
            (unsigned int)((motor_bkin_test_bdtr & TIM_BDTR_MOE) != 0U),
            (unsigned long)motor_bkin_test_sr,
            (unsigned long)motor_bkin_test_bdtr);
      pwm_break_snapshot_t snap;
      if (pwmGetBreakSnapshot(&snap))
        uartPrintf(_DEF_UART1,
            "BKIN SNAP: SR=0x%08lX BDTR=0x%08lX PA6=%lu\r\n",
            (unsigned long)snap.sr, (unsigned long)snap.bdtr,
            (unsigned long)((snap.gpioa_idr >> 6) & 1U));
    }
    if (now - print_ms >= 250U)
    {
      print_ms = now;
      const uint32_t count = encoderGetCount();
      int32_t delta = (int32_t)count - (int32_t)last_count;
      if (delta > (int32_t)ENCODER_MAX_COUNT / 2) delta -= ENCODER_MAX_COUNT;
      if (delta < -(int32_t)ENCODER_MAX_COUNT / 2) delta += ENCODER_MAX_COUNT;
      last_count = count;
      uartPrintf(_DEF_UART1,
          "ENC count=%lu delta=%ld speed_m/e=%ld/%ld mrad/s MOE=%u state=%u fault=%u\r\n",
          (unsigned long)count, (long)delta,
          (long)(encoderGetMechanicalSpeed() * 1000.0f),
          (long)(encoderGetElectricalSpeed() * 1000.0f),
          (unsigned int)((TIM1->BDTR & TIM_BDTR_MOE) != 0U),
          (unsigned int)motorGetState(), (unsigned int)motorGetFault());
    }
    delay(1U);
  }
}
