/*
 * ap.c
 *
 *  Created on: 2026. 5. 26.45
 *      Author: YDG
 */
#include "ap.h"


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

    motorSetCurrentReference(0.0f, 0.2f);

#endif

    motorCurrentDiagStart();; // 여기서 설정

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
      uint32_t current_test_elapsed_ms;
      uint32_t current_test_adc_samples;
      uint32_t current_test_adc_hz = 0U;

      int32_t ia_ma;
      int32_t ib_ma;
      int32_t ic_ma;
      int32_t current_sum_ma;

      motor_abc_f_t current_min;
      motor_abc_f_t current_max;

      int32_t vd_mv;
      int32_t vq_mv;

      int32_t duty_u_permille;
      int32_t duty_v_permille;
      int32_t duty_w_permille;

      int32_t id_ref_ma;
      int32_t id_meas_ma;
      int32_t iq_ref_ma;
      int32_t iq_meas_ma;

      float id_avg;
      float iq_avg;

      int32_t id_avg_ma;
      int32_t iq_avg_ma;

      float id_tail_avg;
      float iq_tail_avg;
      float vd_tail_avg;
      float vq_tail_avg;

      int32_t id_tail_avg_ma;
      int32_t iq_tail_avg_ma;

      int32_t vd_tail_avg_mv;
      int32_t vq_tail_avg_mv;

#if !(CURRENT_ADC_NOISE_TEST_ENABLE)
      uint32_t l_sample_count = 0U;
      float l_id = 0.0f;
      float l_iq = 0.0f;
      float l_vq = 0.0f;
      float l_theta = 0.0f;
#endif

      uint32_t noise_count = 0U;
      motor_abc_u16_t noise_raw;
      motor_abc_f_t noise_current;
      /*
       * motorStop() 전에 마지막 monitor 값을 저장한다.
       */
      motorGetMonitor(&monitor);

      id_ref_ma = (int32_t)(monitor.id_ref * 1000.0f);
      id_meas_ma = (int32_t)(monitor.id_meas * 1000.0f);
      iq_ref_ma = (int32_t)(monitor.iq_ref * 1000.0f);
      iq_meas_ma = (int32_t)(monitor.iq_meas * 1000.0f);

      // [수정] 30 ms 전체 Min/Max snapshot
      motorCurrentDiagGetCurrentMinMax(&current_min, &current_max);

      motorCurrentDiagGetDQAverage(&id_avg, &iq_avg);

      id_avg_ma = (int32_t)(id_avg * 1000.0f);
      iq_avg_ma = (int32_t)(iq_avg * 1000.0f);

      motorCurrentDiagGetTailAverage(&id_tail_avg, &iq_tail_avg, &vd_tail_avg, &vq_tail_avg);

      id_tail_avg_ma = (int32_t)(id_tail_avg * 1000.0f);
      iq_tail_avg_ma = (int32_t)(iq_tail_avg * 1000.0f);
      vd_tail_avg_mv = (int32_t)(vd_tail_avg * 1000.0f);
      vq_tail_avg_mv = (int32_t)(vq_tail_avg * 1000.0f);

      // [수정]
      // 실제 ISR 진단 sample 수 기준으로 계산
      current_test_adc_samples = motorCurrentDiagGetSampleCount();

      current_test_elapsed_ms = (current_test_adc_samples * CURRENT_LOOP_PERIOD_US) / 1000U;

      current_test_adc_hz = 1000000U / CURRENT_LOOP_PERIOD_US;


      ia_ma = (int32_t)(monitor.ia * 1000.0f);
      ib_ma = (int32_t)(monitor.ib * 1000.0f);
      ic_ma = (int32_t)(monitor.ic * 1000.0f);

      current_sum_ma = (int32_t)((monitor.ia + monitor.ib + monitor.ic) * 1000.0f);

      vd_mv = (int32_t)(monitor.vd_cmd * 1000.0f);
      vq_mv = (int32_t)(monitor.vq_cmd * 1000.0f);

      duty_u_permille = (int32_t)(monitor.duty_u * 1000.0f);
      duty_v_permille = (int32_t)(monitor.duty_v * 1000.0f);
      duty_w_permille = (int32_t)(monitor.duty_w * 1000.0f);


      motorSetCurrentReference(0.0f, 0.0f);
      motorStop();

      current_test_stopped = true;

      prev_adc_count = adcGetCurrentUpdateCount();

      uartPrintf(
          _DEF_UART1,
          "\r\n"
          "===== CURRENT PI 200mA TEST =====\r\n");

      uartPrintf(
          _DEF_UART1,
          "Stop Reason : %lu ms COMPLETE\r\n",
                     (unsigned long)CURRENT_LOOP_TEST_TIME_MS);
      uartPrintf(
          _DEF_UART1,
          "Elapsed     : %lu ms\r\n",
          (unsigned long)current_test_elapsed_ms);

      uartPrintf(
          _DEF_UART1,
          "ADC Samples : %lu\r\n",
          (unsigned long)current_test_adc_samples);

      uartPrintf(
          _DEF_UART1,
          "ADC Hz      : %lu\r\n",
          (unsigned long)current_test_adc_hz);

      uartPrintf(
          _DEF_UART1,
          "State Snap  : %d\r\n",
          (int)monitor.state);

      uartPrintf(
          _DEF_UART1,
          "Fault       : %d\r\n\r\n",
          (int)monitor.fault);


      uartPrintf(
          _DEF_UART1,
          "Ia          : %ld mA\r\n",
          (long)ia_ma);

      uartPrintf(
          _DEF_UART1,
          "Ib          : %ld mA\r\n",
          (long)ib_ma);

      uartPrintf(
          _DEF_UART1,
          "Ic          : %ld mA\r\n",
          (long)ic_ma);

      uartPrintf(
          _DEF_UART1,
          "Iabc Sum    : %ld mA\r\n\r\n",
          (long)current_sum_ma);

      uartPrintf(
          _DEF_UART1,
          "Ia Min/Max  : %ld / %ld mA\r\n",
          (long)(current_min.a * 1000.0f),
          (long)(current_max.a * 1000.0f));

      uartPrintf(
          _DEF_UART1,
          "Ib Min/Max  : %ld / %ld mA\r\n",
          (long)(current_min.b * 1000.0f),
          (long)(current_max.b * 1000.0f));

      uartPrintf(
          _DEF_UART1,
          "Ic Min/Max  : %ld / %ld mA\r\n\r\n",
          (long)(current_min.c * 1000.0f),
          (long)(current_max.c * 1000.0f));
      uartPrintf(
          _DEF_UART1,
          "Id          : %ld / %ld mA\r\n",
          (long)id_ref_ma,
          (long)id_meas_ma);

      uartPrintf(
          _DEF_UART1,
          "Iq          : %ld / %ld mA\r\n\r\n",
          (long)iq_ref_ma,
          (long)iq_meas_ma);
      // [추가] 30 ms / 600 samples 전체 평균
      uartPrintf(
          _DEF_UART1,
          "Id Average  : %ld mA\r\n",
          (long)id_avg_ma);

      uartPrintf(
          _DEF_UART1,
          "Iq Average  : %ld mA\r\n\r\n",
          (long)iq_avg_ma);
      uartPrintf(
          _DEF_UART1,
          "Id Last100  : %ld mA\r\n",
          (long)id_tail_avg_ma);

      uartPrintf(
          _DEF_UART1,
          "Iq Last100  : %ld mA\r\n",
          (long)iq_tail_avg_ma);

      uartPrintf(
          _DEF_UART1,
          "Vd Last100  : %ld mV\r\n",
          (long)vd_tail_avg_mv);

      uartPrintf(
          _DEF_UART1,
          "Vq Last100  : %ld mV\r\n\r\n",
          (long)vq_tail_avg_mv);
      uartPrintf(
          _DEF_UART1,
          "Vdq         : %ld / %ld mV\r\n",
          (long)vd_mv,
          (long)vq_mv);

      uartPrintf(
          _DEF_UART1,
          "Duty UVW    : %ld / %ld / %ld permille\r\n",
          (long)duty_u_permille,
          (long)duty_v_permille,
          (long)duty_w_permille);


      /*
       * [수정]
       * 기존 PASS 문구와 COMPLETE 문구가 중복 출력되던 부분을
       * COMPLETE 한 줄로 통일한다.
       *
       * 이 결과는 0.8 A 차단이 발생하지 않았다는 의미이며,
       * phase current가 최종 통과 범위 안이라는 자동 판정은 아니다.
       */
      uartPrintf(
          _DEF_UART1,
          "RESULT      : COMPLETE - NO 0.8 A TRIP\r\n");

      /*
       * [수정]
       * 정상 종료 로그의 마지막 구분선을 복구한다.
       */
#if CURRENT_ADC_NOISE_TEST_ENABLE

  noise_count = motorCurrentNoiseGetCount();

  uartPrintf(
      _DEF_UART1,
      "\r\n"
      "===== NEUTRAL PWM ADC NOISE TEST =====\r\n");

  uartPrintf(
      _DEF_UART1,
      "Duty    : 500 / 500 / 500 permille\r\n");

  uartPrintf(
      _DEF_UART1,
      "Ts      : 50 us\r\n");

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

        uartPrintf(_DEF_UART1,
                   "\r\n===== CURRENT PI STEP RESPONSE =====\r\n");

        uartPrintf(_DEF_UART1,
                   "Iq Ref  : 200 mA\r\n");

        uartPrintf(_DEF_UART1,
                   "Kp      : 0.38\r\n");

        uartPrintf(_DEF_UART1,
                   "Ki      : 148.0\r\n");

        uartPrintf(_DEF_UART1,
                   "Ts      : 50 us\r\n");

        uartPrintf(_DEF_UART1,
                   "Samples : %lu\r\n\r\n",
                   (unsigned long)l_sample_count);

        uartPrintf(_DEF_UART1,
                   "N,Time_us,Id_mA,Iq_mA,Vq_mV,Theta_mrad\r\n");


        for (uint32_t i = 0U; i < l_sample_count; i++)
        {
          if (motorCurrentLTestGetSample(i, &l_id, &l_iq, &l_vq, &l_theta) == true)
          {
            uartPrintf(_DEF_UART1,
                       "%lu,%lu,%ld,%ld,%ld,%ld\r\n",
                       (unsigned long)i,
                       (unsigned long)(i * CURRENT_LOOP_PERIOD_US),
                       (long)(l_id * 1000.0f),
                       (long)(l_iq * 1000.0f),
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
      uint32_t current_test_elapsed_ms;
      uint32_t current_test_adc_samples;
      uint32_t current_test_adc_hz = 0U;

      int32_t ia_ma;
      int32_t ib_ma;
      int32_t ic_ma;
      int32_t current_sum_ma;

      motor_abc_f_t current_min;
      motor_abc_f_t current_max;

      int32_t vd_mv;
      int32_t vq_mv;

      int32_t duty_u_permille;
      int32_t duty_v_permille;
      int32_t duty_w_permille;

      motor_fault_t test_fault;


      /*
       * ADC와 PWM 정지 후처리 전에 trip 순간의 값을 저장한다.
       */
      motorGetMonitor(&monitor);

      /*
       * [수정]
       * motorSetFault() 호출 전 Fault 원인을 별도 변수에 저장한다.
       */
      // [수정]
      motorCurrentDiagGetCurrentMinMax(&current_min,
                                       &current_max);

      test_fault = monitor.fault;


      // [수정]
      // trip이 발생한 실제 ISR sample 번호
      current_test_adc_samples =
          motorCurrentDiagGetSampleCount();

      current_test_elapsed_ms =
          (current_test_adc_samples *
           CURRENT_LOOP_PERIOD_US) / 1000U;

      current_test_adc_hz =
          1000000U / CURRENT_LOOP_PERIOD_US;

      if (current_test_elapsed_ms > 0U)
      {
        current_test_adc_hz =
            (current_test_adc_samples * 1000U) /
            current_test_elapsed_ms;
      }


      ia_ma =
          (int32_t)(monitor.ia * 1000.0f);

      ib_ma =
          (int32_t)(monitor.ib * 1000.0f);

      ic_ma =
          (int32_t)(monitor.ic * 1000.0f);

      current_sum_ma =
          (int32_t)((monitor.ia +
                     monitor.ib +
                     monitor.ic) * 1000.0f);


      vd_mv =
          (int32_t)(monitor.vd_cmd * 1000.0f);

      vq_mv =
          (int32_t)(monitor.vq_cmd * 1000.0f);


      duty_u_permille =
          (int32_t)(monitor.duty_u * 1000.0f);

      duty_v_permille =
          (int32_t)(monitor.duty_v * 1000.0f);

      duty_w_permille =
          (int32_t)(monitor.duty_w * 1000.0f);


      /*
       * ISR에서는 MOE를 즉시 차단한 상태다.
       * main context에서 injected ADC와 PWM timer를 정지한다.
       */
      motorSetFault(test_fault);

      current_test_stopped = true;

      prev_adc_count = adcGetCurrentUpdateCount();


      uartPrintf(
          _DEF_UART1,
          "\r\n"
          "===== CURRENT FIXED ALPHA TEST =====\r\n");


      /*
       * [수정]
       * 실제 Fault 원인에 따라 Stop Reason을 구분한다.
       */
      if (test_fault == MOTOR_FAULT_SW_OVERCURRENT)
      {
        uartPrintf(
            _DEF_UART1,
            "Stop Reason : SW OVERCURRENT\r\n");
      }
      else
      {
        uartPrintf(
            _DEF_UART1,
            "Stop Reason : OTHER FAULT (%d)\r\n",
            (int)test_fault);
      }


      uartPrintf(
          _DEF_UART1,
          "Elapsed     : %lu ms\r\n",
          (unsigned long)current_test_elapsed_ms);

      uartPrintf(
          _DEF_UART1,
          "ADC Samples : %lu\r\n",
          (unsigned long)current_test_adc_samples);

      uartPrintf(
          _DEF_UART1,
          "ADC Hz      : %lu\r\n",
          (unsigned long)current_test_adc_hz);

      uartPrintf(
          _DEF_UART1,
          "State Snap  : %d\r\n",
          (int)monitor.state);

      uartPrintf(
          _DEF_UART1,
          "Fault       : %d\r\n\r\n",
          (int)test_fault);


      uartPrintf(
          _DEF_UART1,
          "Ia          : %ld mA\r\n",
          (long)ia_ma);

      uartPrintf(
          _DEF_UART1,
          "Ib          : %ld mA\r\n",
          (long)ib_ma);

      uartPrintf(
          _DEF_UART1,
          "Ic          : %ld mA\r\n",
          (long)ic_ma);

      uartPrintf(
          _DEF_UART1,
          "Iabc Sum    : %ld mA\r\n\r\n",
          (long)current_sum_ma);

      uartPrintf(
          _DEF_UART1,
          "Ia Min/Max  : %ld / %ld mA\r\n",
          (long)(current_min.a * 1000.0f),
          (long)(current_max.a * 1000.0f));

      uartPrintf(
          _DEF_UART1,
          "Ib Min/Max  : %ld / %ld mA\r\n",
          (long)(current_min.b * 1000.0f),
          (long)(current_max.b * 1000.0f));

      uartPrintf(
          _DEF_UART1,
          "Ic Min/Max  : %ld / %ld mA\r\n\r\n",
          (long)(current_min.c * 1000.0f),
          (long)(current_max.c * 1000.0f));

      uartPrintf(
          _DEF_UART1,
          "Vdq         : %ld / %ld mV\r\n",
          (long)vd_mv,
          (long)vq_mv);

      uartPrintf(
          _DEF_UART1,
          "Duty UVW    : %ld / %ld / %ld permille\r\n",
          (long)duty_u_permille,
          (long)duty_v_permille,
          (long)duty_w_permille);


      /*
       * [수정]
       * 기존에는 모든 Fault를 SW OVERCURRENT로 출력했다.
       * 실제 Fault 원인에 따라 최종 결과를 한 번만 출력한다.
       */
      if (test_fault == MOTOR_FAULT_SW_OVERCURRENT)
      {
        uartPrintf(
            _DEF_UART1,
            "RESULT      : FAIL - SW OVERCURRENT\r\n");
      }
      else
      {
        uartPrintf(
            _DEF_UART1,
            "RESULT      : FAIL - OTHER FAULT (%d)\r\n",
            (int)test_fault);
      }


      uartPrintf(
          _DEF_UART1,
          "================================\r\n\r\n");
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
    if (now - pre_time_debug >= 500U)
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


      adc_count =
          adcGetCurrentUpdateCount();

      adc_diff =
          adc_count - prev_adc_count;

      prev_adc_count =
          adc_count;

      adc_hz =
          adc_diff * 2U;


      vbus_mv =
          (int32_t)(monitor.vbus * 1000.0f);


      ia_ma =
          (int32_t)(monitor.ia * 1000.0f);

      ib_ma =
          (int32_t)(monitor.ib * 1000.0f);

      ic_ma =
          (int32_t)(monitor.ic * 1000.0f);

      current_sum_ma =
          (int32_t)((monitor.ia +
                     monitor.ib +
                     monitor.ic) * 1000.0f);


      id_ref_ma =
          (int32_t)(monitor.id_ref * 1000.0f);

      id_meas_ma =
          (int32_t)(monitor.id_meas * 1000.0f);

      iq_ref_ma =
          (int32_t)(monitor.iq_ref * 1000.0f);

      iq_meas_ma =
          (int32_t)(monitor.iq_meas * 1000.0f);


      speed_target_mrad =
          (int32_t)(monitor.speed_target * 1000.0f);

      speed_ref_mrad =
          (int32_t)(monitor.speed_ref * 1000.0f);

      speed_meas_mrad =
          (int32_t)(monitor.speed_meas * 1000.0f);

      theta_mrad =
          (int32_t)(monitor.theta_e * 1000.0f);


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


        hall_speed_e_mrad =
            (int32_t)(hallGetElectricalSpeed() *
                      1000.0f);

        hall_speed_m_mrad =
            (int32_t)(hallGetMechanicalSpeed() *
                      1000.0f);

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


        hall_theta_e =
            hallGetElectricalAngle();

        hall_error_e =
            hall_theta_e -
            monitor.theta_e;


        while (hall_error_e > PI)
        {
          hall_error_e -= 2.0f * PI;
        }

        while (hall_error_e < -PI)
        {
          hall_error_e += 2.0f * PI;
        }


        hall_theta_e_mrad =
            (int32_t)(hall_theta_e * 1000.0f);

        hall_error_e_mrad =
            (int32_t)(hall_error_e * 1000.0f);


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
        int32_t vd_mv =
            (int32_t)(monitor.vd_cmd * 1000.0f);

        int32_t vq_mv =
            (int32_t)(monitor.vq_cmd * 1000.0f);

        int32_t duty_u_permille =
            (int32_t)(monitor.duty_u * 1000.0f);

        int32_t duty_v_permille =
            (int32_t)(monitor.duty_v * 1000.0f);

        int32_t duty_w_permille =
            (int32_t)(monitor.duty_w * 1000.0f);


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
