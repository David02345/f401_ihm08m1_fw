/* Minimal PWM-OFF encoder bring-up monitor. No Hall diagnostics, no auto-start. */
#include "ap.h"

void apInit(void)
{
  uartOpen(_DEF_UART1, 57600);
}

void apMain(void)
{
  uint32_t slow_ms = millis();
  uint32_t print_ms = millis();
  uint32_t last_count = encoderGetCount();
  pwmDisableOutput();
  uartPrintf(_DEF_UART1, "\r\n=== ENCODER FOC CLEAN PROJECT ===\r\n");
  uartPrintf(_DEF_UART1, "TIM2 encoder driver is USER TODO (encoder.c unchanged).\r\n");
  uartPrintf(_DEF_UART1, "Motor PWM output LOCKED: MOTOR_ALLOW_POWER_STAGE=0.\r\n");
  uartPrintf(_DEF_UART1, "Encoder monitor every 250ms; P=print, ?=help.\r\n");

  while (1)
  {
    const uint32_t now = millis();
    if (now - slow_ms >= 100U)
    {
      slow_ms = now;
      motorLowSpeedTask();
    }
    if (uartAvailable(_DEF_UART1))
    {
      const uint8_t key = uartRead(_DEF_UART1);
      if (key == '?' || key == 'h' || key == 'H')
        uartPrintf(_DEF_UART1, "PWM OFF | P: print encoder count/angle/speed | ?: help\r\n");
      if (key == 'p' || key == 'P') print_ms = now - 250U;
      /* No START command is enabled while encoder.c is incomplete. */
    }
    if (now - print_ms >= 250U)
    {
      print_ms = now;
      const uint32_t count = encoderGetCount();
      const int32_t delta = (int32_t)(count - last_count);
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
