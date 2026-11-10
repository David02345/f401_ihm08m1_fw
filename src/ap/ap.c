/*
 * ap.c
 *
 *  Created on: 2026. 5. 26.
 *      Author: YDG
 */
#include "ap.h"
#include "util.h"

static float speed_target = 0.0f;

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
  uartPrintf(_DEF_UART1, "DIAG: S runs Id=0.5A, theta 0->90deg (2s), then MOE OFF automatically.\r\n");
  uartPrintf(_DEF_UART1, "DIAG: No electrical encoder offset is saved; not a verified FOC alignment.\r\n");

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
      motor_align_diag_t diag;
      if (motorGetAlignDiag(&diag))
      {
        uartPrintf(_DEF_UART1,
            "\r\n=== ALIGN VECTOR DIAGNOSTIC (PWM OFF) ===\r\n");
        uartPrintf(_DEF_UART1,
            "START count=%ld END count=%ld sample_mask=0x%lX\r\n",
            (long)diag.start_count, (long)diag.end_count,
            (unsigned long)diag.valid_mask);
        for (uint32_t i = 0U; i < 3U; ++i)
        {
          if ((diag.valid_mask & (1U << i)) == 0U) continue;
          const motor_align_diag_sample_t *s = &diag.samples[i];
          uartPrintf(_DEF_UART1,
              "S%lu angle=%ldmrad count=%ld tick=%lu Id=%ld/%ldmA Iq=%ldmA Vd/Vq=%ld/%ldmV\r\n",
              (unsigned long)i, (long)(s->theta_e * 1000.0f),
              (long)s->encoder_count, (unsigned long)s->tick,
              (long)(s->id_ref * 1000.0f), (long)(s->id_meas * 1000.0f),
              (long)(s->iq_meas * 1000.0f),
              (long)(s->vd * 1000.0f), (long)(s->vq * 1000.0f));
          uartPrintf(_DEF_UART1,
              "S%lu Ia/Ib/Ic=%ld/%ld/%ldmA duty(U/V/W)=%ld/%ld/%ld permille\r\n",
              (unsigned long)i,
              (long)(s->ia * 1000.0f), (long)(s->ib * 1000.0f),
              (long)(s->ic * 1000.0f),
              (long)(s->duty_u * 1000.0f), (long)(s->duty_v * 1000.0f),
              (long)(s->duty_w * 1000.0f));
          uartPrintf(_DEF_UART1,
              "S%lu CCR1/2/3=%lu/%lu/%lu CCER=0x%08lX MOE=%u\r\n",
              (unsigned long)i,
              (unsigned long)s->ccr1, (unsigned long)s->ccr2,
              (unsigned long)s->ccr3, (unsigned long)s->ccer,
              (unsigned int)s->moe);
        }
        if ((diag.valid_mask & 5U) == 5U)
        {
          int32_t d = diag.samples[2].encoder_count - diag.samples[0].encoder_count;
          if (d > (int32_t)(ENCODER_MAX_COUNT / 2U)) d -= (int32_t)ENCODER_MAX_COUNT;
          if (d < -(int32_t)(ENCODER_MAX_COUNT / 2U)) d += (int32_t)ENCODER_MAX_COUNT;
          uartPrintf(_DEF_UART1,
              "ENC DELTA (sample 0deg -> 90deg) = %ld counts (wrapped)\r\n", (long)d);
        }
        uartPrintf(_DEF_UART1,
            "NOTE: CCR registers only; actual gate/phase voltages require probing.\r\n");
      }
      motorStop();
      uartPrintf(_DEF_UART1, "DIAG FINISHED; PWM=OFF, state=IDLE.\r\n\r\n");
    }
    if (uartAvailable(_DEF_UART1))
    {
      uint8_t key = uartRead(_DEF_UART1);
      uartPrintf(_DEF_UART1, "RX KEY = %c\r\n", key);

      if (key == 'S')
      {
        if (motorGetState() == MOTOR_STATE_IDLE && motorGetFault() == MOTOR_FAULT_NONE)
        {
          uartPrintf(_DEF_UART1, "START COMMAND\r\n");
          //motorSetSpeedReference(0.0f);
          motorStart();
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
