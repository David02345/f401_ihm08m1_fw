/*
 * ap.c
 *
 *  Created on: 2026. 5. 26.45
 *      Author: YDG
 */
#include "ap.h"

extern TIM_HandleTypeDef htim1;

void apInit(void)
{
  uartOpen(_DEF_UART1, 57600);
}


void apMain(void)
{
  uint32_t pre_time_motor_slow = millis();
  uint32_t pre_time_debug = millis();

  uint32_t prev_adc_count = 0;

  delay(1000);
  motorStart();

  delay(500);

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_SPEED
if (motorGetState() == MOTOR_STATE_SPEED_LOOP)
{
  pwmEnableOutput();
}
#elif MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT
if (motorGetState() == MOTOR_STATE_CURRENT_LOOP)
{
  pwmEnableOutput();
}
#elif MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP
if (motorGetState() == MOTOR_STATE_OPEN_LOOP)
{
  pwmEnableOutput();
}
#endif

  prev_adc_count = adcGetCurrentUpdateCount();

  while (1)
  {
    uint32_t now = millis();

    if (now - pre_time_motor_slow >= 10U)
    {
      pre_time_motor_slow = now;
      motorLowSpeedTask();
    }

    if (now - pre_time_debug >= 500)
    {
      pre_time_debug = now;

      uint32_t adc_count = adcGetCurrentUpdateCount();
      uint32_t adc_diff = adc_count - prev_adc_count;
      prev_adc_count = adc_count;

      uint32_t adc_hz = adc_diff * 2U;   // 500ms 기준이므로 x2
      int32_t vbus_mv = (int32_t)(motorGetVbus() * 1000.0f);

      uartPrintf(_DEF_UART1, "ADC Count : %lu\r\n", (unsigned long)adc_count);
      uartPrintf(_DEF_UART1, "ADC Hz    : %lu\r\n", (unsigned long)adc_hz);
      uartPrintf(_DEF_UART1, "State     : %d\r\n", (int)motorGetState());
      uartPrintf(_DEF_UART1, "Fault     : %d\r\n", (int)motorGetFault());
      uartPrintf(_DEF_UART1, "VBUS      : %ld mV\r\n", (long)vbus_mv);
    }
  }
}

