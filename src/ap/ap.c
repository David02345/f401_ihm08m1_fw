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

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT
  uint32_t current_test_start = 0U;
  bool current_test_stopped = false;

  uint32_t current_test_adc_start_count = 0U;
  bool current_test_started = false;
#endif

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP
  uint32_t open_loop_start = 0U;
  bool open_loop_stopped = false;

#if _USE_HALL_OFFSET_CALIBRATION
  const uint32_t open_loop_test_time_ms = HALL_CAL_TEST_TIME_MS;
#else
  const uint32_t open_loop_test_time_ms = OPEN_LOOP_TEST_TIME_MS;
#endif
#endif

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP) && (_USE_HALL_OFFSET_CALIBRATION)
  int8_t hall_cal_sector = -1;
  int8_t hall_cal_direction = 0;
  float hall_cal_theta_e = 0.0f;
#endif

  delay(1000);

  motorStart();
  motorSetCurrentReference(0.0f, 0.0f);

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT

  if (motorGetState() == MOTOR_STATE_CURRENT_LOOP)
  {
    motorSetCurrentReference(0.0f, 0.0f);
#if !_USE_HALL_TEST_ONLY
    current_test_adc_start_count = adcGetCurrentUpdateCount();
    current_test_start = millis();
    current_test_started = true;

    pwmEnableOutput();
#endif
  }

#elif MOTOR_CONTROL_MODE == MOTOR_CONTROL_SPEED

  if (motorGetState() == MOTOR_STATE_SPEED_LOOP)
  {
    pwmEnableOutput();
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



#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT)

    if ((current_test_started == true) && (current_test_stopped == false) &&
        (motorGetState() == MOTOR_STATE_CURRENT_LOOP) &&
        ((now - current_test_start) >= CURRENT_LOOP_TEST_TIME_MS))
    {
      motorGetMonitor(&monitor);

      uint32_t current_test_elapsed_ms = now - current_test_start;

      uint32_t current_test_adc_end_count = adcGetCurrentUpdateCount();

      uint32_t current_test_adc_samples = current_test_adc_end_count -
                                          current_test_adc_start_count;

      uint32_t current_test_adc_hz = 0U;

      if (current_test_elapsed_ms > 0U)
      {
        current_test_adc_hz = (current_test_adc_samples * 1000U) / current_test_elapsed_ms;
      }

      int32_t ia_ma = (int32_t)(monitor.ia * 1000.0f);
      int32_t ib_ma = (int32_t)(monitor.ib * 1000.0f);
      int32_t ic_ma = (int32_t)(monitor.ic * 1000.0f);

      int32_t current_sum_ma = (int32_t)((monitor.ia + monitor.ib + monitor.ic) * 1000.0f);

      int32_t vd_mv = (int32_t)(monitor.vd_cmd * 1000.0f);
      int32_t vq_mv = (int32_t)(monitor.vq_cmd * 1000.0f);

      int32_t duty_u_permille = (int32_t)(monitor.duty_u * 1000.0f);
      int32_t duty_v_permille = (int32_t)(monitor.duty_v * 1000.0f);
      int32_t duty_w_permille = (int32_t)(monitor.duty_w * 1000.0f);

      motorSetCurrentReference(0.0f, 0.0f);
      motorStop();
      current_test_stopped = true;

      prev_adc_count = current_test_adc_end_count;
      uartPrintf(_DEF_UART1,
                 "\r\n"
                 "===== CURRENT NEUTRAL TEST =====\r\n");

      uartPrintf(_DEF_UART1,
                 "Stop Reason : 100 ms COMPLETE\r\n");

      uartPrintf(_DEF_UART1,
                 "Elapsed     : %lu ms\r\n",
                 (unsigned long)current_test_elapsed_ms);

      uartPrintf(_DEF_UART1,
                 "ADC Samples : %lu\r\n",
                 (unsigned long)current_test_adc_samples);

      uartPrintf(_DEF_UART1,
                 "ADC Hz      : %lu\r\n",
                 (unsigned long)current_test_adc_hz);

      uartPrintf(_DEF_UART1,
                 "State Snap  : %d\r\n",
                 (int)monitor.state);

      uartPrintf(_DEF_UART1,
                 "Fault       : %d\r\n\r\n",
                 (int)monitor.fault);

      uartPrintf(_DEF_UART1,
                 "Ia          : %ld mA\r\n",
                 (long)ia_ma);

      uartPrintf(_DEF_UART1,
                 "Ib          : %ld mA\r\n",
                 (long)ib_ma);

      uartPrintf(_DEF_UART1,
                 "Ic          : %ld mA\r\n",
                 (long)ic_ma);

      uartPrintf(_DEF_UART1,
                 "Iabc Sum    : %ld mA\r\n\r\n",
                 (long)current_sum_ma);

      uartPrintf(_DEF_UART1,
                 "Vdq         : %ld / %ld mV\r\n",
                 (long)vd_mv,
                 (long)vq_mv);

      uartPrintf(_DEF_UART1,
                 "Duty UVW    : %ld / %ld / %ld permille\r\n",
                 (long)duty_u_permille,
                 (long)duty_v_permille,
                 (long)duty_w_permille);

      uartPrintf(_DEF_UART1,
                 "RESULT      : PASS - NO SW OVERCURRENT\r\n");

      uartPrintf(_DEF_UART1,
                 "RESULT      : COMPLETE - NO 0.8 A TRIP\r\n");
    }

    /*
     * [수정]
     * Software overcurrent 발생 후 main-context 후처리
     *
     * motorCurrentLoop()에서 이미:
     *   pwmDisableOutput()
     *   motor_fault = MOTOR_FAULT_SW_OVERCURRENT
     *   motor_state = MOTOR_STATE_FAULT
     *
     * 가 실행된 상태다.
     */
    else if ((current_test_started == true) &&
             (current_test_stopped == false) &&
             (motorGetState() == MOTOR_STATE_FAULT))
    {
      // [수정] ADC/PWM을 정지하기 전에 trip 순간의 monitor 값을 저장
      motorGetMonitor(&monitor);

      uint32_t current_test_elapsed_ms = now - current_test_start;
      uint32_t current_test_adc_end_count = adcGetCurrentUpdateCount();
      uint32_t current_test_adc_samples = current_test_adc_end_count -
                                          current_test_adc_start_count;

      uint32_t current_test_adc_hz = 0U;

      if (current_test_elapsed_ms > 0U)
      {
        current_test_adc_hz = (current_test_adc_samples * 1000U) /
                               current_test_elapsed_ms;
      }

      int32_t ia_ma = (int32_t)(monitor.ia * 1000.0f);
      int32_t ib_ma = (int32_t)(monitor.ib * 1000.0f);
      int32_t ic_ma = (int32_t)(monitor.ic * 1000.0f);

      int32_t current_sum_ma = (int32_t)((monitor.ia +
                                          monitor.ib +
                                          monitor.ic) * 1000.0f);

      int32_t vd_mv = (int32_t)(monitor.vd_cmd * 1000.0f);

      int32_t vq_mv = (int32_t)(monitor.vq_cmd * 1000.0f);

      int32_t duty_u_permille = (int32_t)(monitor.duty_u * 1000.0f);
      int32_t duty_v_permille = (int32_t)(monitor.duty_v * 1000.0f);
      int32_t duty_w_permille = (int32_t)(monitor.duty_w * 1000.0f);

      motorSetFault(monitor.fault);

      current_test_stopped = true;

      prev_adc_count = current_test_adc_end_count;

      uartPrintf(_DEF_UART1, "\r\n" "===== CURRENT NEUTRAL TEST =====\r\n");
      motor_fault_t test_fault = motorGetFault();

      if (test_fault == MOTOR_FAULT_SW_OVERCURRENT)
      {
        uartPrintf(_DEF_UART1,
                   "Stop Reason : SW OVERCURRENT\r\n");
      }
      else
      {
        uartPrintf(_DEF_UART1,
                   "Stop Reason : OTHER FAULT (%d)\r\n",
                   (int)test_fault);
      }
      uartPrintf(_DEF_UART1, "Elapsed     : %lu ms\r\n", (unsigned long)current_test_elapsed_ms);
      uartPrintf(_DEF_UART1, "ADC Samples : %lu\r\n", (unsigned long)current_test_adc_samples);
      uartPrintf(_DEF_UART1, "ADC Hz      : %lu\r\n", (unsigned long)current_test_adc_hz);
      uartPrintf(_DEF_UART1, "State Snap  : %d\r\n", (int)monitor.state);
      uartPrintf(_DEF_UART1, "Fault       : %d\r\n\r\n", (int)monitor.fault);
      uartPrintf(_DEF_UART1, "Ia          : %ld mA\r\n", (long)ia_ma);
      uartPrintf(_DEF_UART1, "Ib          : %ld mA\r\n", (long)ib_ma);
      uartPrintf(_DEF_UART1, "Ic          : %ld mA\r\n", (long)ic_ma);
      uartPrintf(_DEF_UART1, "Iabc Sum    : %ld mA\r\n\r\n", (long)current_sum_ma);
      uartPrintf(_DEF_UART1, "Vdq         : %ld / %ld mV\r\n", (long)vd_mv, (long)vq_mv);
      uartPrintf(_DEF_UART1, "Duty UVW    : %ld / %ld / %ld permille\r\n", (long)duty_u_permille,
                                                                           (long)duty_v_permille,
                                                                           (long)duty_w_permille);
      uartPrintf(_DEF_UART1, "RESULT      : FAIL - SW OVERCURRENT\r\n");
      uartPrintf(_DEF_UART1, "================================\r\n\r\n");
    }

#endif

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP

    if ((open_loop_stopped == false) && (motorGetState() == MOTOR_STATE_OPEN_LOOP) &&
        ((now - open_loop_start) >= open_loop_test_time_ms))
    {
      motorStop();
      open_loop_stopped = true;
    }
    else if (motorGetState() == MOTOR_STATE_FAULT)
    {
      open_loop_stopped = true;
    }

#endif

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP) && (_USE_HALL_OFFSET_CALIBRATION)

    if (motorGetHallCalibrationEvent(&hall_cal_sector, &hall_cal_direction, &hall_cal_theta_e) == true)
    {
      int32_t hall_cal_theta_mrad = (int32_t)(hall_cal_theta_e * 1000.0f);

      uartPrintf(_DEF_UART1, "HALL_CAL : Sector %d, Dir %d, Theta %ld mrad\r\n",
                                                           (int)hall_cal_sector,
                                                           (int)hall_cal_direction,
                                                           (long)hall_cal_theta_mrad);
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


      motor_abc_u16_t raw;
      motor_abc_f_t offset;

      adcGetCurrentRaw(&raw);
      adcGetCurrentOffset(&offset);

      uartPrintf(_DEF_UART1, "Current Raw : %u, %u, %u\r\n", raw.a, raw.b, raw.c);
      uartPrintf(_DEF_UART1, "Current Off : %ld, %ld, %ld\r\n\n", (long)offset.a,
                                                                  (long)offset.b,
                                                                  (long)offset.c);

#if _USE_HALL_SENSOR
      int32_t hall_speed_e_mrad = (int32_t)(hallGetElectricalSpeed() * 1000.0f);
      int32_t hall_speed_m_mrad = (int32_t)(hallGetMechanicalSpeed() * 1000.0f);
      uint8_t hall_state = hallGetState();
      uartPrintf(_DEF_UART1,"Hall      : %u%u%u, Sector %d, Dir %d, Valid %d\r\n",
                 (unsigned int)((hall_state >> 2) & 1U),
                 (unsigned int)((hall_state >> 1) & 1U),
                 (unsigned int)(hall_state & 1U),
                 (int)hallGetSectorIndex(),
                 (int)hallGetDirection(),
                 (int)hallIsValid());
      uartPrintf(_DEF_UART1, "Hall Speed E : %ld mrad/s\r\n", (long)hall_speed_e_mrad);
      uartPrintf(_DEF_UART1, "Hall Speed M : %ld mrad/s\r\n", (long)hall_speed_m_mrad);

      float hall_theta_e;
      float hall_error_e;
      int32_t hall_theta_e_mrad;
      int32_t hall_error_e_mrad;

      hall_theta_e = hallGetElectricalAngle();

      hall_error_e = hall_theta_e - monitor.theta_e;

      while (hall_error_e > PI)
      {
        hall_error_e -= 2.0f * PI;
      }

      while (hall_error_e < -PI)
      {
        hall_error_e += 2.0f * PI;
      }

      hall_theta_e_mrad = (int32_t)(hall_theta_e * 1000.0f);

      hall_error_e_mrad = (int32_t)(hall_error_e * 1000.0f);

      uartPrintf(_DEF_UART1, "Hall Theta E : %ld mrad\r\n",   (long)hall_theta_e_mrad);
      uartPrintf(_DEF_UART1, "Hall Error E : %ld mrad\r\n\n", (long)hall_error_e_mrad);

#endif
      int32_t vd_mv = (int32_t)(monitor.vd_cmd * 1000.0f);
      int32_t vq_mv = (int32_t)(monitor.vq_cmd * 1000.0f);

      int32_t duty_u_permille = (int32_t)(monitor.duty_u * 1000.0f);
      int32_t duty_v_permille = (int32_t)(monitor.duty_v * 1000.0f);
      int32_t duty_w_permille = (int32_t)(monitor.duty_w * 1000.0f);

      uartPrintf(_DEF_UART1, "Vdq       : %ld / %ld mV\r\n", (long)vd_mv, (long)vq_mv);

      uartPrintf(_DEF_UART1, "Duty UVW  : %ld / %ld / %ld permille\r\n\n", (long)duty_u_permille,
                                                                           (long)duty_v_permille,
                                                                           (long)duty_w_permille);
    }
  }
}

