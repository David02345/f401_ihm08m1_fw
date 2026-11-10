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
  uint32_t speed_prev_ms = millis();
  uint32_t slow_ms = millis();
  uint32_t print_ms = millis();
  uint32_t last_count = encoderGetCount();
  pwmDisableOutput();
  uartPrintf(_DEF_UART1, "\r\n=== ENCODER FOC CLEAN PROJECT ===\r\n");
  uartPrintf(_DEF_UART1, "TIM2 Encoder Mode initialized.\r\n");
  uartPrintf(_DEF_UART1, "Motor PWM output LOCKED: MOTOR_ALLOW_POWER_STAGE=0.\r\n");
  uartPrintf(_DEF_UART1, "Encoder monitor every 250ms; P=print, ?=help.\r\n");

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
    if (uartAvailable(_DEF_UART1))
    {
      const uint8_t key = uartRead(_DEF_UART1);

      if (key == 'S')
      {
        if (motorGetState() == MOTOR_STATE_IDLE && motorGetFault() == MOTOR_FAULT_NONE)
        {
            motorSetSpeedReference(0.0f);
            motorStart();
        }
      }
      else if (key == 'X')
      {
        if (motorGetState() != MOTOR_STATE_FAULT)
        {
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
    }
    delay(1U);
  }
}
