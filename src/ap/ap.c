/*
 * ap.c
 *
 *  Created on: 2026. 5. 26.45
 *      Author: YDG
 */
#include "ap.h"

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)

static void apPrintCurrentDiagTiming(void)
{
  motor_diag_timing_t dbg = {0};

  motorCurrentDiagGetTiming(&dbg);

  uartPrintf(_DEF_UART1,
             "DBG CNT cmd / ADC ISR : %lu / %lu\r\n",
             (unsigned long)dbg.cnt_before_output,
             (unsigned long)dbg.cnt_first_isr);

  uartPrintf(_DEF_UART1,
             "DBG JEOC / IRQ pending: %lu / %lu\r\n",
             (unsigned long)dbg.jeoc_before_output,
             (unsigned long)dbg.adc_irq_pending_before_output);

  uartPrintf(_DEF_UART1,
             "DBG ISR seen / delay : %u / %lu us\r\n",
             (unsigned int)dbg.first_isr_seen,
             (unsigned long)(dbg.cycles_to_first_isr /
                 (SystemCoreClock / 1000000U)));
  uartPrintf(_DEF_UART1,
             "DBG Raw before output: %u, %u, %u\r\n",
             (unsigned int)dbg.raw_before_a,
             (unsigned int)dbg.raw_before_b,
             (unsigned int)dbg.raw_before_c);
}

#endif

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

  bool current_test_stopped = false;
  bool current_test_started = false;

#endif


#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP

  uint32_t open_loop_start = 0U;
  bool open_loop_stopped = false;

#if _USE_HALL_OFFSET_CALIBRATION

  const uint32_t open_loop_test_time_ms =
      HALL_CAL_TEST_TIME_MS;

#else

  const uint32_t open_loop_test_time_ms =
      OPEN_LOOP_TEST_TIME_MS;

#endif
#endif


#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP) && \
    (_USE_HALL_OFFSET_CALIBRATION)

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
#if CURRENT_ADC_NOISE_TEST_ENABLE

    motorSetCurrentReference(0.0f, 0.0f);

#else

    motorSetCurrentReference(0.0f, -0.4f);// 여기

#endif

#if !_USE_HALL_TEST_ONLY

    current_test_started = true;

    motorCurrentDiagStart();

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


#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT

    /*
     * Current neutral PWM 시험 정상 종료
     */
    if ((current_test_started == true) &&
        (current_test_stopped == false) &&
        (motorGetState() == MOTOR_STATE_CURRENT_LOOP) &&
        (motorCurrentDiagIsDone() == true))
    {
      uint32_t current_test_adc_samples;
      motor_abc_f_t current_min;
      motor_abc_f_t current_max;

#if !(CURRENT_ADC_NOISE_TEST_ENABLE)
      uint32_t l_sample_count = 0U;
      float l_id = 0.0f;
      float l_iq = 0.0f;
      float l_vd = 0.0f;
      float l_vq = 0.0f;
      float l_theta = 0.0f;
      int32_t id_ref_ma;
      int32_t iq_ref_ma;
#else
      uint32_t noise_count = 0U;
      motor_abc_u16_t noise_raw;
      motor_abc_f_t noise_current;
#endif

      // 정지 전에 측정값 보존. 시험 종료 후에는 저장된 CSV만 출력.
      motorGetMonitor(&monitor);
      motorCurrentDiagGetCurrentMinMax(&current_min, &current_max);
      current_test_adc_samples = motorCurrentDiagGetSampleCount();
#if !(CURRENT_ADC_NOISE_TEST_ENABLE)
      id_ref_ma = (int32_t)(monitor.id_ref * 1000.0f);
      iq_ref_ma = (int32_t)(monitor.iq_ref * 1000.0f);
#endif
      motorSetCurrentReference(0.0f, 0.0f);
      motorStop();
      current_test_stopped = true;
      prev_adc_count = adcGetCurrentUpdateCount();

#if CURRENT_ADC_NOISE_TEST_ENABLE
#if CURRENT_ADC_PWM_OFF_TEST_ENABLE
      uartPrintf(_DEF_UART1, "\r\n===== PWM OFF ADC NOISE - SUMMARY =====\r\n");
#else
      uartPrintf(_DEF_UART1, "\r\n===== NEUTRAL PWM ON ADC NOISE - SUMMARY =====\r\n");
#endif
#else
      uartPrintf(_DEF_UART1, "\r\n===== CURRENT PI - SUMMARY =====\r\n");
      uartPrintf(_DEF_UART1, "Id Ref/Meas : %ld / %ld mA\r\n",
                 (long)id_ref_ma, (long)(monitor.id_meas * 1000.0f));
      uartPrintf(_DEF_UART1, "Iq Ref/Meas : %ld / %ld mA\r\n",
                 (long)iq_ref_ma, (long)(monitor.iq_meas * 1000.0f));
      uartPrintf(_DEF_UART1, "Vd/Vq       : %ld / %ld mV\r\n",
                 (long)(monitor.vd_cmd * 1000.0f),
                 (long)(monitor.vq_cmd * 1000.0f));
#endif
      uartPrintf(_DEF_UART1, "RESULT      : COMPLETE (Fault %d)\r\n", (int)monitor.fault);
      uartPrintf(_DEF_UART1, "Samples     : %lu\r\n",
                 (unsigned long)current_test_adc_samples);
      apPrintCurrentDiagTiming();  // 추가
      uartPrintf(_DEF_UART1, "Ia Min/Max  : %ld / %ld mA\r\n",
                 (long)(current_min.a * 1000.0f),
                 (long)(current_max.a * 1000.0f));
      uartPrintf(_DEF_UART1, "Ib Min/Max  : %ld / %ld mA\r\n",
                 (long)(current_min.b * 1000.0f),
                 (long)(current_max.b * 1000.0f));
      uartPrintf(_DEF_UART1, "Ic Min/Max  : %ld / %ld mA\r\n",
                 (long)(current_min.c * 1000.0f),
                 (long)(current_max.c * 1000.0f));
#if CURRENT_ADC_NOISE_TEST_ENABLE

  noise_count = motorCurrentNoiseGetCount();

#if CURRENT_ADC_PWM_OFF_TEST_ENABLE
  uartPrintf(_DEF_UART1, "\r\n===== PWM OFF ADC NOISE TEST =====\r\n");
#else
  uartPrintf(_DEF_UART1, "\r\n===== NEUTRAL PWM ON ADC NOISE TEST =====\r\n");
#endif
  uartPrintf(_DEF_UART1, "Duty cmd: 500 / 500 / 500 permille\r\n");
  uartPrintf(_DEF_UART1, "Ts      : %lu us\r\n", (unsigned long)CURRENT_LOOP_PERIOD_US);

  uartPrintf(
      _DEF_UART1,
      "Samples : %lu\r\n\r\n",
      (unsigned long)noise_count);

  uartPrintf(
      _DEF_UART1,
      "N,Time_us,RawA,RawB,RawC,"
      "Ia_mA,Ib_mA,Ic_mA,IabcSum_mA\r\n");


  for (uint32_t i = 0U;
       i < noise_count;
       i++)
  {
    if (motorCurrentNoiseGetSample(i,
                                   &noise_raw,
                                   &noise_current) == true)
    {
      int32_t ia_ma;
      int32_t ib_ma;
      int32_t ic_ma;
      int32_t sum_ma;

      ia_ma =
          (int32_t)(noise_current.a * 1000.0f);

      ib_ma =
          (int32_t)(noise_current.b * 1000.0f);

      ic_ma =
          (int32_t)(noise_current.c * 1000.0f);

      sum_ma = ia_ma + ib_ma + ic_ma;


      uartPrintf(
          _DEF_UART1,
          "%lu,%lu,%u,%u,%u,%ld,%ld,%ld,%ld\r\n",
          (unsigned long)i,
          (unsigned long)(i * CURRENT_LOOP_PERIOD_US),

          noise_raw.a,
          noise_raw.b,
          noise_raw.c,

          (long)ia_ma,
          (long)ib_ma,
          (long)ic_ma,
          (long)sum_ma);
    }
  }

  uartPrintf(
      _DEF_UART1,
      "======================================\r\n");

#else
  l_sample_count = motorCurrentLTestGetCount();

  uartPrintf(_DEF_UART1, "\r\n===== CURRENT PI IQ TEST =====\r\n");
  uartPrintf(_DEF_UART1, "Id Ref  : %ld mA\r\n", (long)id_ref_ma);
  uartPrintf(_DEF_UART1, "Iq Ref  : %ld mA\r\n", (long)iq_ref_ma);
  uartPrintf(_DEF_UART1, "Kp      : 0.30\r\n");
  uartPrintf(_DEF_UART1, "Ki      : 246.0\r\n");
  uartPrintf(_DEF_UART1, "Ts      : %lu us\r\n", (unsigned long)CURRENT_LOOP_PERIOD_US);
  uartPrintf(_DEF_UART1, "Samples : %lu\r\n\r\n", (unsigned long)l_sample_count);

        uartPrintf(_DEF_UART1,
                   "N,Time_us,Id_mA,Iq_mA,Vd_mV,Vq_mV,Theta_mrad\r\n");


        for (uint32_t i = 0U; i < l_sample_count; i++)
        {
          if (motorCurrentLTestGetSample(i,
                                         &l_id,
                                         &l_iq,
                                         &l_vd,
                                         &l_vq,
                                         &l_theta) == true)
          {
            uartPrintf(_DEF_UART1,
                       "%lu,%lu,%ld,%ld,%ld,%ld,%ld\r\n",
                       (unsigned long)i,
                       (unsigned long)(i * CURRENT_LOOP_PERIOD_US),

                       (long)(l_id * 1000.0f),
                       (long)(l_iq * 1000.0f),

                       (long)(l_vd * 1000.0f),
                       (long)(l_vq * 1000.0f),

                       (long)(l_theta * 1000.0f));
          }
        }

        uartPrintf(_DEF_UART1,
                   "============================\r\n");


#endif

    }


    /*
     * Current neutral PWM 시험 Fault 종료
     */
    else if ((current_test_started == true) &&
             (current_test_stopped == false) &&
             (motorGetState() == MOTOR_STATE_FAULT))
    {
      uint32_t current_test_adc_samples;
      motor_fault_t test_fault;
      motor_abc_u16_t trip_raw = {0};
      motor_abc_f_t trip_current = {0};
      motor_abc_f_t current_min;
      motor_abc_f_t current_max;

      // ADC/PWM 정지 전에 Fault 원인과 Trip 순간 값을 보존한다.
      motorGetMonitor(&monitor);
      motorCurrentDiagGetTripSnapshot(&trip_raw, &trip_current);
      motorCurrentDiagGetCurrentMinMax(&current_min, &current_max);
      test_fault = monitor.fault;
      current_test_adc_samples = motorCurrentDiagGetSampleCount();

      // ISR에서 MOE 차단 완료. main에서 타이머/ADC 정지 처리.
      motorSetFault(test_fault);
      current_test_stopped = true;
      prev_adc_count = adcGetCurrentUpdateCount();

#if CURRENT_ADC_NOISE_TEST_ENABLE
      uartPrintf(_DEF_UART1, "\r\n===== ADC NOISE TEST - FAULT =====\r\n");
#else
      uartPrintf(_DEF_UART1, "\r\n===== CURRENT PI TEST - FAULT =====\r\n");
#endif
      uartPrintf(_DEF_UART1, "Fault       : %d\r\n", (int)test_fault);
      uartPrintf(_DEF_UART1, "Samples     : %lu\r\n",
                 (unsigned long)current_test_adc_samples);
      uartPrintf(_DEF_UART1, "Startup Trip: %lu\r\n",
                 (unsigned long)motorCurrentDiagGetStartupTripSample());
      apPrintCurrentDiagTiming();
      uartPrintf(_DEF_UART1, "Trip Raw    : %u, %u, %u\r\n",
                 trip_raw.a, trip_raw.b, trip_raw.c);
      uartPrintf(_DEF_UART1, "Trip Current: %ld, %ld, %ld mA\r\n",
                 (long)(trip_current.a * 1000.0f),
                 (long)(trip_current.b * 1000.0f),
                 (long)(trip_current.c * 1000.0f));
      uartPrintf(_DEF_UART1, "Ia Min/Max  : %ld / %ld mA\r\n",
                 (long)(current_min.a * 1000.0f),
                 (long)(current_max.a * 1000.0f));
      uartPrintf(_DEF_UART1, "Ib Min/Max  : %ld / %ld mA\r\n",
                 (long)(current_min.b * 1000.0f),
                 (long)(current_max.b * 1000.0f));
      uartPrintf(_DEF_UART1, "Ic Min/Max  : %ld / %ld mA\r\n",
                 (long)(current_min.c * 1000.0f),
                 (long)(current_max.c * 1000.0f));
      uartPrintf(_DEF_UART1, "RESULT      : FAULT\r\n");
    }

#endif


#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP

    if ((open_loop_stopped == false) &&
        (motorGetState() == MOTOR_STATE_OPEN_LOOP) &&
        ((now - open_loop_start) >=
         open_loop_test_time_ms))
    {
      motorStop();
      open_loop_stopped = true;
    }
    else if (motorGetState() == MOTOR_STATE_FAULT)
    {
      open_loop_stopped = true;
    }

#endif


#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP) && \
    (_USE_HALL_OFFSET_CALIBRATION)

    if (motorGetHallCalibrationEvent(
            &hall_cal_sector,
            &hall_cal_direction,
            &hall_cal_theta_e) == true)
    {
      int32_t hall_cal_theta_mrad =
          (int32_t)(hall_cal_theta_e * 1000.0f);

      uartPrintf(
          _DEF_UART1,
          "HALL_CAL : Sector %d, Dir %d, Theta %ld mrad\r\n",
          (int)hall_cal_sector,
          (int)hall_cal_direction,
          (long)hall_cal_theta_mrad);
    }

#endif


    /*
     * 기존 500 ms UART monitoring
     */
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT
    if ((current_test_started == false) &&
        (now - pre_time_debug >= 500U))
#else
    if (now - pre_time_debug >= 500U)
#endif
    {
      uint32_t adc_count;
      uint32_t adc_diff;
      uint32_t adc_hz;

      int32_t vbus_mv;

      int32_t ia_ma;
      int32_t ib_ma;
      int32_t ic_ma;
      int32_t current_sum_ma;

      int32_t id_ref_ma;
      int32_t id_meas_ma;
      int32_t iq_ref_ma;
      int32_t iq_meas_ma;

      int32_t speed_target_mrad;
      int32_t speed_ref_mrad;
      int32_t speed_meas_mrad;
      int32_t theta_mrad;

      motor_abc_u16_t raw;
      motor_abc_f_t offset;


      pre_time_debug = now;

      if (motorGetMonitor(&monitor) != true)
      {
        continue;
      }

      adc_count = adcGetCurrentUpdateCount();
      adc_diff = adc_count - prev_adc_count;
      prev_adc_count = adc_count;
      adc_hz = adc_diff * 2U;

      vbus_mv = (int32_t)(monitor.vbus * 1000.0f);

      ia_ma = (int32_t)(monitor.ia * 1000.0f);
      ib_ma = (int32_t)(monitor.ib * 1000.0f);
      ic_ma = (int32_t)(monitor.ic * 1000.0f);

      current_sum_ma = (int32_t)((monitor.ia +
                                  monitor.ib +
                                  monitor.ic) * 1000.0f);


      id_ref_ma = (int32_t)(monitor.id_ref * 1000.0f);
      id_meas_ma = (int32_t)(monitor.id_meas * 1000.0f);
      iq_ref_ma = (int32_t)(monitor.iq_ref * 1000.0f);
      iq_meas_ma = (int32_t)(monitor.iq_meas * 1000.0f);


      speed_target_mrad = (int32_t)(monitor.speed_target * 1000.0f);
      speed_ref_mrad = (int32_t)(monitor.speed_ref * 1000.0f);
      speed_meas_mrad = (int32_t)(monitor.speed_meas * 1000.0f);
      theta_mrad = (int32_t)(monitor.theta_e * 1000.0f);


      uartPrintf(
          _DEF_UART1,
          "ADC Count : %lu\r\n",
          (unsigned long)adc_count);

      uartPrintf(
          _DEF_UART1,
          "ADC Hz    : %lu\r\n",
          (unsigned long)adc_hz);

      uartPrintf(
          _DEF_UART1,
          "State     : %d\r\n",
          (int)monitor.state);

      uartPrintf(
          _DEF_UART1,
          "Fault     : %d\r\n",
          (int)monitor.fault);

      uartPrintf(
          _DEF_UART1,
          "VBUS      : %ld mV\r\n\n",
          (long)vbus_mv);


      uartPrintf(
          _DEF_UART1,
          "Ia        : %ld mA\r\n",
          (long)ia_ma);

      uartPrintf(
          _DEF_UART1,
          "Ib        : %ld mA\r\n",
          (long)ib_ma);

      uartPrintf(
          _DEF_UART1,
          "Ic        : %ld mA\r\n",
          (long)ic_ma);

      uartPrintf(
          _DEF_UART1,
          "Iabc Sum  : %ld mA\r\n\n",
          (long)current_sum_ma);


      uartPrintf(
          _DEF_UART1,
          "Id        : %ld / %ld mA\r\n",
          (long)id_meas_ma,
          (long)id_ref_ma);

      uartPrintf(
          _DEF_UART1,
          "Iq        : %ld / %ld mA\r\n\n",
          (long)iq_meas_ma,
          (long)iq_ref_ma);


      uartPrintf(
          _DEF_UART1,
          "Speed     : %ld / %ld / %ld mrad/s\r\n",
          (long)speed_target_mrad,
          (long)speed_ref_mrad,
          (long)speed_meas_mrad);

      uartPrintf(
          _DEF_UART1,
          "Theta_e   : %ld mrad\r\n\n",
          (long)theta_mrad);


      adcGetCurrentRaw(&raw);
      adcGetCurrentOffset(&offset);


      uartPrintf(
          _DEF_UART1,
          "Current Raw : %u, %u, %u\r\n",
          raw.a,
          raw.b,
          raw.c);

      uartPrintf(
          _DEF_UART1,
          "Current Off : %ld, %ld, %ld\r\n\n",
          (long)offset.a,
          (long)offset.b,
          (long)offset.c);


#if _USE_HALL_SENSOR

      {
        int32_t hall_speed_e_mrad;
        int32_t hall_speed_m_mrad;

        uint8_t hall_state;

        float hall_theta_e;
        float hall_error_e;

        int32_t hall_theta_e_mrad;
        int32_t hall_error_e_mrad;

        hall_speed_e_mrad = (int32_t)(hallGetElectricalSpeed() * 1000.0f);
        hall_speed_m_mrad = (int32_t)(hallGetMechanicalSpeed() * 1000.0f);
        hall_state = hallGetState();

        uartPrintf(
            _DEF_UART1,
            "Hall      : %u%u%u, Sector %d, Dir %d, Valid %d\r\n",
            (unsigned int)((hall_state >> 2) & 1U),
            (unsigned int)((hall_state >> 1) & 1U),
            (unsigned int)(hall_state & 1U),
            (int)hallGetSectorIndex(),
            (int)hallGetDirection(),
            (int)hallIsValid());

        uartPrintf(
            _DEF_UART1,
            "Hall Speed E : %ld mrad/s\r\n",
            (long)hall_speed_e_mrad);

        uartPrintf(
            _DEF_UART1,
            "Hall Speed M : %ld mrad/s\r\n",
            (long)hall_speed_m_mrad);

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


        uartPrintf(
            _DEF_UART1,
            "Hall Theta E : %ld mrad\r\n",
            (long)hall_theta_e_mrad);

        uartPrintf(
            _DEF_UART1,
            "Hall Error E : %ld mrad\r\n\n",
            (long)hall_error_e_mrad);
      }

#endif

      {
        int32_t vd_mv = (int32_t)(monitor.vd_cmd * 1000.0f);
        int32_t vq_mv = (int32_t)(monitor.vq_cmd * 1000.0f);
        int32_t duty_u_permille = (int32_t)(monitor.duty_u * 1000.0f);
        int32_t duty_v_permille = (int32_t)(monitor.duty_v * 1000.0f);
        int32_t duty_w_permille = (int32_t)(monitor.duty_w * 1000.0f);


        uartPrintf(
            _DEF_UART1,
            "Vdq       : %ld / %ld mV\r\n",
            (long)vd_mv,
            (long)vq_mv);

        uartPrintf(
            _DEF_UART1,
            "Duty UVW  : %ld / %ld / %ld permille\r\n\n",
            (long)duty_u_permille,
            (long)duty_v_permille,
            (long)duty_w_permille);
      }
    }
  }
}
