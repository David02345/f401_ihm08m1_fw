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
  uint32_t pre_time_slow = millis();
  uint32_t pre_time_debug = millis();

  uint32_t prev_adc_count = 0;

  motor_monitor_t monitor = {0};

#if !_USE_HALL_TEST_ONLY
  uint32_t pretime_current = 0;
  bool pretime_current_applied = false;
#endif

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP
  uint32_t open_loop_start = 0U;
  bool open_loop_stopped = false;
#endif
  delay(1000);

  motorStart();


#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_SPEED

  if (motorGetState() == MOTOR_STATE_SPEED_LOOP)
  {
    pwmEnableOutput();
  }

#elif MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT

  if (motorGetState() == MOTOR_STATE_CURRENT_LOOP)
  {
    motorSetCurrentReference(0.0f, 0.0f);
#if !_USE_HALL_TEST_ONLY
    pwmEnableOutput();
    pretime_current = millis();
#endif
  }

#elif MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP

  if (motorGetState() == MOTOR_STATE_OPEN_LOOP)
  {
    open_loop_start = millis();
    pwmEnableOutput();
  }

#endif

  prev_adc_count = adcGetCurrentUpdateCount();

  while (1)
  {
    uint32_t now = millis();

    if (now - pre_time_slow >= 10U)
    {
      pre_time_slow = now;
      motorLowSpeedTask();
    }

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP

    if ((open_loop_stopped == false) &&
        (motorGetState() == MOTOR_STATE_OPEN_LOOP) &&
        ((now - open_loop_start) >= 5000U))
    {
      motorStop();
      open_loop_stopped = true;
    }
    else if (motorGetState() == MOTOR_STATE_FAULT)
    {
      open_loop_stopped = true;
    }

#endif

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && (!_USE_HALL_TEST_ONLY)
    if ((motorGetState() == MOTOR_STATE_CURRENT_LOOP) &&
        (pretime_current_applied == false) && ((now - pretime_current) >= 1000U))
    {
      motorSetCurrentReference(0.0f, 0.05f);

      pretime_current_applied = true;
    }
#endif

    if (now - pre_time_debug >= 500)
    {
      pre_time_debug = now;

      if (motorGetMonitor(&monitor) != true)
      {
        continue;
      }

      uint32_t adc_count = adcGetCurrentUpdateCount();
      uint32_t adc_diff = adc_count - prev_adc_count;
      prev_adc_count = adc_count;

      uint32_t adc_hz = adc_diff * 2U;   // 500ms 기준이므로 x2
      int32_t vbus_mv = (int32_t)(monitor.vbus * 1000.0f);

      int32_t ia_ma = (int32_t)(monitor.ia * 1000.0f);
      int32_t ib_ma = (int32_t)(monitor.ib * 1000.0f);
      int32_t ic_ma = (int32_t)(monitor.ic * 1000.0f);
      int32_t current_sum_ma = (int32_t)((monitor.ia + monitor.ib + monitor.ic) * 1000.0f);

      int32_t id_ref_ma  = (int32_t)(monitor.id_ref  * 1000.0f);
      int32_t id_meas_ma = (int32_t)(monitor.id_meas * 1000.0f);
      int32_t iq_ref_ma  = (int32_t)(monitor.iq_ref  * 1000.0f);
      int32_t iq_meas_ma =(int32_t)(monitor.iq_meas * 1000.0f);

      int32_t speed_target_mrad = (int32_t)(monitor.speed_target * 1000.0f);
      int32_t speed_ref_mrad    = (int32_t)(monitor.speed_ref * 1000.0f);
      int32_t speed_meas_mrad   = (int32_t)(monitor.speed_meas * 1000.0f);

      int32_t theta_mrad = (int32_t)(monitor.theta_e * 1000.0f);


      uartPrintf(_DEF_UART1, "ADC Count : %lu\r\n", (unsigned long)adc_count);
      uartPrintf(_DEF_UART1, "ADC Hz    : %lu\r\n", (unsigned long)adc_hz);
      uartPrintf(_DEF_UART1, "State     : %d\r\n", (int)monitor.state);
      uartPrintf(_DEF_UART1, "Fault     : %d\r\n", (int)monitor.fault);
      uartPrintf(_DEF_UART1, "VBUS      : %ld mV\r\n\n", (long)vbus_mv);
      uartPrintf(_DEF_UART1, "Ia        : %ld mA\r\n", (long)ia_ma);
      uartPrintf(_DEF_UART1, "Ib        : %ld mA\r\n", (long)ib_ma);
      uartPrintf(_DEF_UART1, "Ic        : %ld mA\r\n", (long)ic_ma);
      uartPrintf(_DEF_UART1, "Iabc Sum  : %ld mA\r\n\n", (long)current_sum_ma);
      uartPrintf(_DEF_UART1, "Id        : %ld / %ld mA\r\n", (long)id_meas_ma, (long)id_ref_ma);
      uartPrintf(_DEF_UART1, "Iq        : %ld / %ld mA\r\n\n", (long)iq_meas_ma, (long)iq_ref_ma);
      uartPrintf(_DEF_UART1, "Speed     : %ld / %ld / %ld mrad/s\r\n", (long)speed_target_mrad,
                                                                       (long)speed_ref_mrad,
                                                                       (long)speed_meas_mrad);
      uartPrintf(_DEF_UART1, "Theta_e   : %ld mrad\r\n\n", (long)theta_mrad);


      #if _USE_HALL_SENSOR
      int32_t hall_speed_mrad = (int32_t)(hallGetMechanicalSpeed() * 1000.0f);
      uint8_t hall_state = hallGetState();
      uartPrintf(_DEF_UART1,"Hall      : %u%u%u, Sector %d, Dir %d, Valid %d\r\n",
                 (unsigned int)((hall_state >> 2) & 1U),
                 (unsigned int)((hall_state >> 1) & 1U),
                 (unsigned int)(hall_state & 1U),
                 (int)hallGetSectorIndex(),
                 (int)hallGetDirection(),
                 (int)hallIsValid());
      uartPrintf(_DEF_UART1, "Hall Speed : %ld mrad/s\r\n", (long)hall_speed_mrad);
      #endif
    }
  }
}

