/*
 * ap.c
 *
 *  Created on: 2026. 5. 26.45
 *      Author: YDG
 */
#include "ap.h"

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)

// Break 발생 순간의 TIM1/GPIO 상태 출력
static void apPrintBreakSnapshot(void)
{
  pwm_break_snapshot_t bk = {0};

  if (pwmGetBreakSnapshot(&bk) == false)
  {
    uartPrintf(_DEF_UART1,
               "BKIN snapshot: NONE\r\n");
    return;
  }

  uartPrintf(
      _DEF_UART1,
      "BKIN IRQ SR/DIER: 0x%08lX / 0x%08lX\r\n",
      (unsigned long)bk.sr,
      (unsigned long)bk.dier
  );

  uartPrintf(
      _DEF_UART1,
      "BKIN IRQ CNT/BDTR: %lu / 0x%08lX\r\n",
      (unsigned long)bk.cnt,
      (unsigned long)bk.bdtr
  );

  uartPrintf(
      _DEF_UART1,
      "BKIN IRQ BIF/BIE : %u / %u\r\n",
      (unsigned int)((bk.sr & TIM_SR_BIF) != 0U),
      (unsigned int)((bk.dier & TIM_DIER_BIE) != 0U)
  );

  uartPrintf(
      _DEF_UART1,
      "BKIN IRQ PA6/MOE: %u / %u\r\n",
      (unsigned int)((bk.gpioa_idr & GPIO_PIN_6) != 0U),
      (unsigned int)((bk.bdtr & TIM_BDTR_MOE) != 0U)
  );

  uartPrintf(
      _DEF_UART1,
      "BKIN IRQ GPIOA_IDR: 0x%08lX\r\n",
      (unsigned long)bk.gpioa_idr
  );
}

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
#if 0
void apMain(void)
{
  delay(1000);

  pwmStart();
  pwmEnableOutput();

  uartPrintf(_DEF_UART1,
      "\r\n===== HARDWARE BKIN TEST =====\r\n");

  uartPrintf(_DEF_UART1,
      "MOE Before : %lu\r\n",
      (unsigned long)((TIM1->BDTR & TIM_BDTR_MOE) != 0U));

  uartPrintf(_DEF_UART1,
      "Connect PA6(BKIN) to GND\r\n");

  while ((pwmIsBreakFault() == false) &&
         ((TIM1->BDTR & TIM_BDTR_MOE) != 0U))
  {
    delay(1);
  }

  uartPrintf(_DEF_UART1,
      "MOE After  : %lu\r\n",
      (unsigned long)((TIM1->BDTR & TIM_BDTR_MOE) != 0U));

  uartPrintf(_DEF_UART1,
      "Break IRQ  : %u\r\n",
      (unsigned int)pwmIsBreakFault());

  pwmStop();

  while (1)
  {
    delay(1000);
  }
}
#endif

void apMain(void)
{
  uint32_t pre_time_slow = millis();
  uint32_t pre_time_debug = millis();

  uint32_t prev_adc_count = 0;

  motor_monitor_t monitor = {0};


#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT

  bool current_test_stopped = false;
  bool current_test_started = false;

  #if CURRENT_ROTATION_TEST_ENABLE
  uint32_t current_test_start_ms = 0U;
  #endif

  #if (CURRENT_ADC_RAW_SCAN_ENABLE == 1U) && (CURRENT_ADC_UART_SILENT_TEST == 1U)

  uint32_t silent_start_ms = 0U;
  uint32_t silent_adc_start = 0U;
  uint32_t silent_adc_end = 0U;
  uint32_t silent_elapsed_ms = 0U;

  motor_adc_raw_scan_t silent_scan = {0};
  motor_state_t silent_state;
  motor_fault_t silent_fault;

    #if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U

    uint16_t split_min[2][3] = {{0}};
    uint16_t split_max[2][3] = {{0}};
    uint32_t split_spike[2][3] = {{0}};

    #endif

  #endif

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

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
      (CURRENT_FOC_BENCH_ENABLE == 1U)

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // CYCCNT는 초기화하지 않음 (Hall 시간 측정과 공유)

#endif

  delay(1000);

#if 0  // Hall EXTI 소프트웨어 테스트

  pwmDisableOutput();

  uint32_t c0 = hallGetIrqCount();

  EXTI->SWIER = GPIO_PIN_3;
  delay(10);
  uint32_t c1 = hallGetIrqCount();

  EXTI->SWIER = GPIO_PIN_10;
  delay(10);
  uint32_t c2 = hallGetIrqCount();

  EXTI->SWIER = GPIO_PIN_15;
  delay(10);
  uint32_t c3 = hallGetIrqCount();

  uartPrintf(_DEF_UART1,
      "\r\n===== HALL EXTI SELF TEST =====\r\n");

  uartPrintf(_DEF_UART1,
      "IRQ Count : %lu -> %lu -> %lu -> %lu\r\n",
      (unsigned long)c0,
      (unsigned long)c1,
      (unsigned long)c2,
      (unsigned long)c3);

  uartPrintf(_DEF_UART1,
      "Hall Transition : %lu\r\n",
      (unsigned long)hallGetTransitionCount());

  while (1)
  {
    delay(1000);
  }

#endif

#if 0
  pwmDisableOutput();

  while (1)
  {
    uint8_t h1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
    uint8_t h2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
    uint8_t h3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);

    uartPrintf(_DEF_UART1,
        "Hall GPIO: %d%d%d | IRQ: %lu | Transition: %lu\r\n",
        h1, h2, h3,
        (unsigned long)hallGetIrqCount(),
        (unsigned long)hallGetTransitionCount());

    delay(250);
  }
#endif

  motorStart();
  motorSetCurrentReference(0.0f, 0.0f);

#if (CURRENT_FOC_BENCH_ENABLE == 1U) || (CURRENT_ADC_RAW_SCAN_ENABLE == 1U)

  pwmDisableOutput();

#endif

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT

  if (motorGetState() == MOTOR_STATE_CURRENT_LOOP)
  {
  #if CURRENT_ADC_NOISE_TEST_ENABLE

    motorSetCurrentReference(0.0f, 0.0f);

  #elif CURRENT_IQ_STEP_TEST_ENABLE

      // Step Test는 0mA에서 시작
    motorSetCurrentReference(0.0f, 0.0f);

  #elif CURRENT_ROTATION_TEST_ENABLE

  // PWM 활성화 후 최초 100ms는 0A
    motorSetCurrentReference(0.0f, 0.0f);
  #else

  #if CURRENT_ADC_RAW_SCAN_ENABLE == 1U
    motorSetCurrentReference(0.0f, 0.0f);
  #else
    motorSetCurrentReference(0.0f, 0.0f);
  #endif

  #endif


  #if !_USE_HALL_TEST_ONLY

    // ADC Noise Test 또는 기존 진단이 활성화된 경우
    // 시작 조건은 Current Loop 상태에서만 충족
    #if (CURRENT_NEUTRAL_DIAG_ENABLE == 1U) && \
        (CURRENT_ADC_RAW_SCAN_ENABLE == 0U) && \
        (CURRENT_FOC_BENCH_ENABLE == 0U)

    current_test_started = true;
    motorCurrentDiagStart();
    current_test_start_ms = millis();  // 추가

    #endif

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



#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_ADC_RAW_SCAN_ENABLE == 1U) && \
    (CURRENT_ADC_UART_SILENT_TEST == 1U)


  // 이전 Raw Scan 통계 초기화
  motorAdcRawScanGetStats(&silent_scan);

#if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U

  // 모터 분리 상태에서만 시험
  // U/V/W 50% Neutral PWM 설정
  pwmDisableOutput();
  pwmSetDuty(0.5f, 0.5f, 0.5f);

  // PWM 활성화 이전 보호 조건 확인
  if ((motorGetState() != MOTOR_STATE_CURRENT_LOOP) ||
      (motorGetFault() != MOTOR_FAULT_NONE))
  {
      motorSetFault(MOTOR_FAULT_INIT_FAIL);
  }
  else if ((pwmIsBreakFault() == true) ||
           ((TIM1->SR & TIM_SR_BIF) != 0U))
  {
      motorSetFault(MOTOR_FAULT_BKIN);
  }
  else
  {
      pwmEnableOutput();

      // 명령 후 MOE 실제 활성화 확인
      if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
      {
          motorSetFault(MOTOR_FAULT_INIT_FAIL);
      }
  }

#endif

  silent_adc_start = adcGetCurrentUpdateCount();
  silent_start_ms = millis();


  uint32_t last_uart_ms = silent_start_ms;
  uint32_t uart_print_count = 0U;


#if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U

    // PWM ON: ISR의 완료 플래그 또는 Fault로 종료
    // 시간 제한은 while 내부에서 별도로 감시
    while (1)

#else

    // PWM OFF: 기존 시간 기준 Raw Scan
    while ((uint32_t)(millis() - silent_start_ms)
           < CURRENT_ADC_UART_SILENT_MS)

#endif
  {
    uint32_t now = millis();

    // 기존과 동일하게 10ms마다 실행
    if ((uint32_t)(now - pre_time_slow) >= 10U)
    {
        pre_time_slow = now;
        motorLowSpeedTask();
    }

    // Fault 확인
    if (motorGetState() != MOTOR_STATE_CURRENT_LOOP)
    {
        break;
    }

#if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U

      // ADC ISR이 멈추더라도 시험 시간이 무한정
      // 연장되지 않도록 메인 루프에서 별도로 감시
      if ((uint32_t)(millis() - silent_start_ms) >
          (CURRENT_ADC_PWM_ON_SCAN_MS + 20U))
      {
          pwmDisableOutput();
          motorSetFault(MOTOR_FAULT_DIAG_TIMEOUT);
          break;
      }

#endif

#if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U

    // ISR이 지정된 샘플 수에서 PWM을 차단하면 종료
    if (motorAdcPwmOnScanIsDone() == true)
    {
        break;
    }

#endif

#if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 0U

    // 기존 PWM OFF Raw Scan에서는 MOE OFF 유지
    if ((TIM1->BDTR & TIM_BDTR_MOE) != 0U)
    {
        motorSetFault(MOTOR_FAULT_INIT_FAIL);
        break;
    }

#else

    // PWM ON 시험 중 예상치 않은 출력 차단 감지
    if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
    {
        if (motorAdcPwmOnScanIsDone() == false)
        {
            motorSetFault(MOTOR_FAULT_INIT_FAIL);
        }

        break;
    }

#endif


    #if CURRENT_ADC_TEST_UART_ENABLE == 1U

        // 500ms마다 UART 송신
        if ((uint32_t)(now - last_uart_ms) >= 500U)
        {
            last_uart_ms = now;

            motor_monitor_t mon = {0};
            motor_abc_u16_t raw = {0};
            motor_abc_f_t offset = {0};

            uint32_t count = adcGetCurrentUpdateCount();

            motorGetMonitor(&mon);
            adcGetCurrentRaw(&raw);
            adcGetCurrentOffset(&offset);

            uartPrintf(_DEF_UART1,
                "\r\nADC Count : %lu\r\n",
                (unsigned long)count);

            uartPrintf(_DEF_UART1,
                "Ia/Ib/Ic : %ld / %ld / %ld mA\r\n",
                (long)(mon.ia * 1000.0f),
                (long)(mon.ib * 1000.0f),
                (long)(mon.ic * 1000.0f));

            uartPrintf(_DEF_UART1,
                "Current Raw : %u / %u / %u\r\n",
                (unsigned int)raw.a,
                (unsigned int)raw.b,
                (unsigned int)raw.c);

            uartPrintf(_DEF_UART1,
                "Current Off : %ld / %ld / %ld\r\n",
                (long)offset.a,
                (long)offset.b,
                (long)offset.c);

            uartPrintf(_DEF_UART1,
                "State : %d, Fault : %d, MOE : %u\r\n",
                (int)motorGetState(),
                (int)motorGetFault(),
                (unsigned int)
                    ((TIM1->BDTR & TIM_BDTR_MOE) != 0U));

            uart_print_count++;
        }

    #endif
    }


    // 통계 확보 및 측정 종료
    // 두 함수 호출 사이에는 소수의 ADC 샘플 차이가 날 수 있음


    silent_state = motorGetState();
    silent_fault = motorGetFault();

#if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U

    // Fault 없이 목표 샘플에 도달했는지 검사
    if ((silent_state == MOTOR_STATE_CURRENT_LOOP) &&
        (silent_fault == MOTOR_FAULT_NONE) &&
        ((motorAdcPwmOnScanIsDone() == false) ||
         ((TIM1->BDTR & TIM_BDTR_MOE) != 0U)))
    {
        motorSetFault(MOTOR_FAULT_DIAG_TIMEOUT);

        silent_state = motorGetState();
        silent_fault = motorGetFault();
    }

#endif

    // UART 출력 전에 ADC와 PWM 정지
    // Fault 상태는 보존
    if (silent_state == MOTOR_STATE_CURRENT_LOOP)
    {
        motorStop();
    }
    else if (silent_state == MOTOR_STATE_FAULT)
    {
        // 기존 Fault 원인은 유지
        // 출력과 ADC/타이머를 명시적으로 정지
        pwmDisableOutput();
        adcInjectedStop();
        pwmStop();
    }
    else
    {
        pwmDisableOutput();
        adcInjectedStop();
        pwmStop();
    }

    silent_adc_end = adcGetCurrentUpdateCount();
    silent_elapsed_ms = millis() - silent_start_ms;

#if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U

    motorAdcRawScanGetSplitStats(split_min, split_max, split_spike);

#endif

    motorAdcRawScanGetStats(&silent_scan);

#if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U

    const uint32_t expected_samples =
        (CURRENT_ADC_PWM_ON_SCAN_MS * 1000U) /
        CURRENT_LOOP_PERIOD_US;

    bool scan_success =
        (silent_fault == MOTOR_FAULT_NONE) &&
        (silent_state == MOTOR_STATE_CURRENT_LOOP) &&
        (silent_scan.sample_count == expected_samples) &&
        ((TIM1->BDTR & TIM_BDTR_MOE) == 0U);

#endif

#if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U

    uartPrintf(_DEF_UART1,
        "\r\n===== PWM ON RAW SCAN RESULT =====\r\n");

    uartPrintf(_DEF_UART1,
        "RESULT : %s\r\n",
        scan_success ? "COMPLETE" : "FAILED");

    uartPrintf(_DEF_UART1,
        "Expected Samples : %lu\r\n",
        (unsigned long)expected_samples);

#endif

    // UART 출력은 여기부터 시작
    uartPrintf(_DEF_UART1,
        "\r\n===== ADC UART SILENT TEST =====\r\n");

    uartPrintf(_DEF_UART1,
        "Elapsed ms : %lu\r\n",
        (unsigned long)silent_elapsed_ms);

    uartPrintf(_DEF_UART1,
        "Periodic UART Prints : %lu\r\n",
        (unsigned long)uart_print_count);

    uartPrintf(_DEF_UART1,
        "ADC Count Diff : %lu\r\n",
        (unsigned long)(silent_adc_end - silent_adc_start));

    uartPrintf(_DEF_UART1,
        "SCAN Samples : %lu\r\n",
        (unsigned long)silent_scan.sample_count);


#if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U

    const char *group_name[2] =
    {
        "STARTUP (first 4 samples)",
        "STEADY (remaining samples)"
    };

    uartPrintf(_DEF_UART1,
        "\r\n===== STARTUP / STEADY COMPARISON =====\r\n");

    for (uint32_t g = 0U; g < 2U; g++)
    {
        uartPrintf(_DEF_UART1,
            "%s\r\n", group_name[g]);

        uartPrintf(_DEF_UART1,
            "Raw Min A/B/C : %u / %u / %u\r\n",
            split_min[g][0],
            split_min[g][1],
            split_min[g][2]);

        uartPrintf(_DEF_UART1,
            "Raw Max A/B/C : %u / %u / %u\r\n",
            split_max[g][0],
            split_max[g][1],
            split_max[g][2]);

        uartPrintf(_DEF_UART1,
            "Spike Count A/B/C : %lu / %lu / %lu\r\n",
            (unsigned long)split_spike[g][0],
            (unsigned long)split_spike[g][1],
            (unsigned long)split_spike[g][2]);
    }

    uartPrintf(_DEF_UART1,
        "=======================================\r\n");

#endif

#if CURRENT_ADC_PWM_ON_SCAN_ENABLE == 1U

    uartPrintf(_DEF_UART1,
        "PWM ON Scan Target : %lu ms\r\n",
        (unsigned long)CURRENT_ADC_PWM_ON_SCAN_MS);

#endif

    uartPrintf(_DEF_UART1,
        "Raw Min A/B/C : %u / %u / %u\r\n",
        (unsigned int)silent_scan.raw_min[0],
        (unsigned int)silent_scan.raw_min[1],
        (unsigned int)silent_scan.raw_min[2]);

    uartPrintf(_DEF_UART1,
        "Raw Max A/B/C : %u / %u / %u\r\n",
        (unsigned int)silent_scan.raw_max[0],
        (unsigned int)silent_scan.raw_max[1],
        (unsigned int)silent_scan.raw_max[2]);

    uartPrintf(_DEF_UART1,
        "Ia Min/Max : %ld / %ld mA\r\n",
        (long)(silent_scan.current_min[0] * 1000.0f),
        (long)(silent_scan.current_max[0] * 1000.0f));

    uartPrintf(_DEF_UART1,
        "Ib Min/Max : %ld / %ld mA\r\n",
        (long)(silent_scan.current_min[1] * 1000.0f),
        (long)(silent_scan.current_max[1] * 1000.0f));

    uartPrintf(_DEF_UART1,
        "Ic Min/Max : %ld / %ld mA\r\n",
        (long)(silent_scan.current_min[2] * 1000.0f),
        (long)(silent_scan.current_max[2] * 1000.0f));

    uartPrintf(_DEF_UART1,
        "Spike Count A/B/C : %lu / %lu / %lu\r\n",
        (unsigned long)silent_scan.spike_count[0],
        (unsigned long)silent_scan.spike_count[1],
        (unsigned long)silent_scan.spike_count[2]);


    uartPrintf(_DEF_UART1,
        "Max Spike Streak A/B/C : %lu / %lu / %lu\r\n",
        (unsigned long)silent_scan.spike_max_streak[0],
        (unsigned long)silent_scan.spike_max_streak[1],
        (unsigned long)silent_scan.spike_max_streak[2]);

    uartPrintf(_DEF_UART1,
        "Spike Adjacent Pairs A/B/C : %lu / %lu / %lu\r\n",
        (unsigned long)silent_scan.spike_adjacent_pairs[0],
        (unsigned long)silent_scan.spike_adjacent_pairs[1],
        (unsigned long)silent_scan.spike_adjacent_pairs[2]);

    uartPrintf(_DEF_UART1,
        "OC Samples A/B/C : %lu / %lu / %lu\r\n",
        (unsigned long)silent_scan.oc_sample_count[0],
        (unsigned long)silent_scan.oc_sample_count[1],
        (unsigned long)silent_scan.oc_sample_count[2]);

    uartPrintf(_DEF_UART1,
        "Max OC Streak A/B/C : %lu / %lu / %lu\r\n",
        (unsigned long)silent_scan.oc_max_streak[0],
        (unsigned long)silent_scan.oc_max_streak[1],
        (unsigned long)silent_scan.oc_max_streak[2]);

    uartPrintf(_DEF_UART1,
        "First OC Sample : %lu\r\n",
        (unsigned long)silent_scan.first_oc_sample);

    uartPrintf(_DEF_UART1,
        "First OC Raw A/B/C : %u / %u / %u\r\n",
        (unsigned int)silent_scan.first_oc_raw[0],
        (unsigned int)silent_scan.first_oc_raw[1],
        (unsigned int)silent_scan.first_oc_raw[2]);


    uartPrintf(_DEF_UART1,
        "First Spike Sample : %lu\r\n",
        (unsigned long)silent_scan.first_spike_sample);

    uartPrintf(_DEF_UART1,
        "First Spike Raw : %u / %u / %u\r\n",
        (unsigned int)silent_scan.first_spike_raw[0],
        (unsigned int)silent_scan.first_spike_raw[1],
        (unsigned int)silent_scan.first_spike_raw[2]);

    uartPrintf(_DEF_UART1,
        "TIM1 MOE : %u\r\n",
        (unsigned int)((TIM1->BDTR & TIM_BDTR_MOE) != 0U));

    uartPrintf(_DEF_UART1,
        "State before stop : %d\r\n",
        (int)silent_state);

    uartPrintf(_DEF_UART1,
        "Fault : %d\r\n",
        (int)silent_fault);

    uartPrintf(_DEF_UART1,
        "================================\r\n");

    return;

#else


  while (1)
  {
    uint32_t now = millis();


    if (now - pre_time_slow >= 10U)
    {
      pre_time_slow = now;
      motorLowSpeedTask();
    }

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_ROTATION_TEST_ENABLE == 1U)

    // ADC ISR이 멈춰 정상 종료되지 않는 상황 대비
    if ((current_test_started == true) &&
        (current_test_stopped == false) &&
        (motorGetState() == MOTOR_STATE_CURRENT_LOOP) &&
        (motorCurrentDiagIsDone() == false) &&
        ((uint32_t)(now - current_test_start_ms) >
         (CURRENT_LOOP_TEST_TIME_MS + 200U)))
    {
      // main loop에서 PWM 및 ADC 중지
      motorSetFault(MOTOR_FAULT_DIAG_TIMEOUT);
    }

#endif

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
    #if CURRENT_ADC_SEQUENCE_TEST_ENABLE
  uartPrintf(_DEF_UART1,
             "\r\n===== OFF-ON-OFF ADC SEQUENCE - SUMMARY =====\r\n");
    #elif CURRENT_ADC_PWM_OFF_TEST_ENABLE
  uartPrintf(_DEF_UART1,
             "\r\n===== PWM OFF ADC NOISE - SUMMARY =====\r\n");
    #else
  uartPrintf(_DEF_UART1,
             "\r\n===== NEUTRAL PWM ON ADC NOISE - SUMMARY =====\r\n");
    #endif
  #else
      uartPrintf(_DEF_UART1, "\r\n===== CURRENT PI - SUMMARY =====\r\n");
      uartPrintf(_DEF_UART1,
                 "Hall IRQ Count        : %lu\r\n",
                 (unsigned long)hallGetIrqCount());

      uartPrintf(_DEF_UART1,
                 "Hall Transition Count : %lu\r\n",
                 (unsigned long)hallGetTransitionCount());
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
  #if CURRENT_ADC_SEQUENCE_TEST_ENABLE// 추가
      uartPrintf(
          _DEF_UART1,
          "MOE after PWM OFF : %lu\r\n",
          (unsigned long)motorCurrentDiagGetMoeAfterOff());
  #endif// 추가
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

    #if CURRENT_ADC_SEQUENCE_TEST_ENABLE
  uartPrintf(_DEF_UART1,
             "\r\n===== OFF-ON-OFF ADC SEQUENCE TEST =====\r\n");
    #elif CURRENT_ADC_PWM_OFF_TEST_ENABLE
  uartPrintf(_DEF_UART1,
             "\r\n===== PWM OFF ADC NOISE TEST =====\r\n");
    #else
  uartPrintf(_DEF_UART1,
             "\r\n===== NEUTRAL PWM ON ADC NOISE TEST =====\r\n");
    #endif
  uartPrintf(_DEF_UART1, "Duty cmd: 500 / 500 / 500 permille\r\n");
  uartPrintf(_DEF_UART1, "Ts      : %lu us\r\n", (unsigned long)CURRENT_LOOP_PERIOD_US);

  uartPrintf(
      _DEF_UART1,
      "Samples : %lu\r\n\r\n",
      (unsigned long)noise_count);

    #if CURRENT_ADC_SEQUENCE_TEST_ENABLE

  uint32_t seq_count[3] = {0U, 0U, 0U};

  uint32_t seq_raw_sum[3][3] = {{0U}};

  float seq_current_sum[3][3] = {{0.0f}};

  uartPrintf(
      _DEF_UART1,
      "Stage: OFF1(0-99), ON(100-299), OFF2(300-399)\r\n");

    #endif

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

    #if CURRENT_ADC_SEQUENCE_TEST_ENABLE

      uint32_t stage;

      if (i < CURRENT_SEQUENCE_OFF1_SAMPLES)
      {
        stage = 0U;
      }
      else if (i < (CURRENT_SEQUENCE_OFF1_SAMPLES +
                    CURRENT_SEQUENCE_ON_SAMPLES))
      {
        stage = 1U;
      }
      else
      {
        stage = 2U;
      }

      seq_count[stage]++;

      seq_raw_sum[stage][0] += noise_raw.a;
      seq_raw_sum[stage][1] += noise_raw.b;
      seq_raw_sum[stage][2] += noise_raw.c;

      seq_current_sum[stage][0] += noise_current.a;
      seq_current_sum[stage][1] += noise_current.b;
      seq_current_sum[stage][2] += noise_current.c;

    #endif

    }
  }

    #if CURRENT_ADC_SEQUENCE_TEST_ENABLE

  const char *stage_name[3] =
  {
    "OFF1",
    "ON",
    "OFF2"
  };

  uartPrintf(
      _DEF_UART1,
      "\r\n===== SEQUENCE AVERAGES =====\r\n");

  for (uint32_t stage = 0U; stage < 3U; stage++)
  {
    uint32_t n = seq_count[stage];

    if (n == 0U)
    {
      continue;
    }

    uartPrintf(
        _DEF_UART1,
        "%s : %lu samples\r\n",
        stage_name[stage],
        (unsigned long)n);

    uint32_t avg_a = seq_raw_sum[stage][0] * 100U / n;
    uint32_t avg_b = seq_raw_sum[stage][1] * 100U / n;
    uint32_t avg_c = seq_raw_sum[stage][2] * 100U / n;

    uartPrintf(
        _DEF_UART1,
        "  Raw Avg: %lu.%02lu, %lu.%02lu, %lu.%02lu\r\n",
        (unsigned long)(avg_a / 100U),
        (unsigned long)(avg_a % 100U),
        (unsigned long)(avg_b / 100U),
        (unsigned long)(avg_b % 100U),
        (unsigned long)(avg_c / 100U),
        (unsigned long)(avg_c % 100U));

    uartPrintf(
        _DEF_UART1,
        "  Current Avg: %ld, %ld, %ld mA\r\n",
        (long)(seq_current_sum[stage][0] * 1000.0f / n),
        (long)(seq_current_sum[stage][1] * 1000.0f / n),
        (long)(seq_current_sum[stage][2] * 1000.0f / n));
  }

    #endif

  uartPrintf(
      _DEF_UART1,
      "======================================\r\n");

  #else
  l_sample_count = motorCurrentLTestGetCount();

    #if CURRENT_ROTATION_TEST_ENABLE

  uartPrintf(_DEF_UART1,
           "\r\n===== HALL FOC ROTATION TEST =====\r\n");
  uartPrintf(
      _DEF_UART1,
      "Iq Profile : 0 -> %ld -> 0 mA\r\n",
      (long)(CURRENT_ROTATION_REF_A * 1000.0f));

  uartPrintf(
      _DEF_UART1,
      "Iq Hold    : 100 ~ 1100 ms\r\n");
    #elif CURRENT_IQ_STEP_TEST_ENABLE

  uartPrintf(_DEF_UART1,
           "\r\n===== CURRENT PI IQ STEP RESPONSE TEST =====\r\n");

    #else

  uartPrintf(_DEF_UART1,
           "\r\n===== CURRENT PI IQ TEST =====\r\n");

    #endif
  uartPrintf(_DEF_UART1, "Id Ref  : %ld mA\r\n", (long)id_ref_ma);
  uartPrintf(_DEF_UART1, "Iq Ref  : %ld mA\r\n", (long)iq_ref_ma);
  uartPrintf(_DEF_UART1, "Kp      : 0.30\r\n");
  uartPrintf(_DEF_UART1, "Ki      : 246.0\r\n");
  uartPrintf(_DEF_UART1, "Ts      : %lu us\r\n", (unsigned long)CURRENT_LOOP_PERIOD_US);
  uartPrintf(_DEF_UART1, "Samples : %lu\r\n\r\n", (unsigned long)l_sample_count);


    #if CURRENT_ROTATION_TEST_ENABLE

  uartPrintf(_DEF_UART1,
             "N,Time_us,IqRef_mA,Id_mA,Iq_mA,"
             "Vd_mV,Vq_mV,Theta_mrad,HallState,HallSector,HallValid\r\n");

    #elif CURRENT_IQ_STEP_TEST_ENABLE

  uartPrintf(_DEF_UART1,
             "N,Time_us,IqRef_mA,Id_mA,Iq_mA,"
             "Vd_mV,Vq_mV,Theta_mrad\r\n");

    #else

  uartPrintf(_DEF_UART1,
             "N,Time_us,Id_mA,Iq_mA,"
             "Vd_mV,Vq_mV,Theta_mrad\r\n");

    #endif

for (uint32_t i = 0U; i < l_sample_count; i++)
{
  if (motorCurrentLTestGetSample(i,
                                 &l_id,
                                 &l_iq,
                                 &l_vd,
                                 &l_vq,
                                 &l_theta) == true)
  {
    #if CURRENT_ROTATION_TEST_ENABLE

  uint32_t sample_index =
      i * CURRENT_ROTATION_LOG_DIVIDER;

  uint32_t time_us =
      sample_index * CURRENT_LOOP_PERIOD_US;

  int32_t iq_ref_ma = 0;

  if ((sample_index >= CURRENT_ROTATION_START_SAMPLE) &&
      (sample_index < CURRENT_ROTATION_END_SAMPLE))
  {
    iq_ref_ma =
        (int32_t)(CURRENT_ROTATION_REF_A * 1000.0f);
  }


  uint8_t hall_state = 0U;
  int8_t hall_sector = -1;
  uint8_t hall_valid = 0U;

  motorCurrentLTestGetHallSample(
      i,
      &hall_state,
      &hall_sector,
      &hall_valid);

  uartPrintf(
      _DEF_UART1,
      "%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%u,%d,%u\r\n",
      (unsigned long)i,
      (unsigned long)time_us,
      (long)iq_ref_ma,
      (long)(l_id * 1000.0f),
      (long)(l_iq * 1000.0f),
      (long)(l_vd * 1000.0f),
      (long)(l_vq * 1000.0f),
      (long)(l_theta * 1000.0f),
      (unsigned int)hall_state,
      (int)hall_sector,
      (unsigned int)hall_valid);


    #elif CURRENT_IQ_STEP_TEST_ENABLE

    int32_t iq_ref_ma = 0;

    if ((i >= CURRENT_IQ_STEP_STAGE_SAMPLES) &&
        (i < CURRENT_IQ_STEP_STAGE_SAMPLES * 2U))
    {
      iq_ref_ma =
          (int32_t)(CURRENT_IQ_STEP_REF_A * 1000.0f);
    }

    uartPrintf(_DEF_UART1,
               "%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
               (unsigned long)i,
               (unsigned long)(i * CURRENT_LOOP_PERIOD_US),
               (long)iq_ref_ma,
               (long)(l_id * 1000.0f),
               (long)(l_iq * 1000.0f),
               (long)(l_vd * 1000.0f),
               (long)(l_vq * 1000.0f),
               (long)(l_theta * 1000.0f));

    #else

    uartPrintf(_DEF_UART1,
               "%lu,%lu,%ld,%ld,%ld,%ld,%ld\r\n",
               (unsigned long)i,
               (unsigned long)(i * CURRENT_LOOP_PERIOD_US),
               (long)(l_id * 1000.0f),
               (long)(l_iq * 1000.0f),
               (long)(l_vd * 1000.0f),
               (long)(l_vq * 1000.0f),
               (long)(l_theta * 1000.0f));

    #endif
  }
}

#if CURRENT_IQ_STEP_TEST_ENABLE

{
  const char *stage_name[3] = {
      "BEFORE STEP",
      "IQ STEP",
      "AFTER STEP"
  };

  const uint32_t stage_start[3] = {
      0U,
      CURRENT_IQ_STEP_STAGE_SAMPLES,
      CURRENT_IQ_STEP_STAGE_SAMPLES * 2U
  };

  const uint32_t stage_end[3] = {
      CURRENT_IQ_STEP_STAGE_SAMPLES,
      CURRENT_IQ_STEP_STAGE_SAMPLES * 2U,
      l_sample_count
  };

  uartPrintf(_DEF_UART1,
             "\r\n===== IQ STEP STATISTICS =====\r\n");

  for (uint32_t stage = 0U; stage < 3U; stage++)
  {
    float iq_sum = 0.0f;
    float iq_sq_sum = 0.0f;
    float vq_sum = 0.0f;

    uint32_t count = 0U;

    for (uint32_t i = stage_start[stage];
         i < stage_end[stage]; i++)
    {
      if (motorCurrentLTestGetSample(
              i, &l_id, &l_iq,
              &l_vd, &l_vq, &l_theta) == true)
      {
        iq_sum += l_iq;
        iq_sq_sum += l_iq * l_iq;
        vq_sum += l_vq;

        count++;
      }
    }

    if (count == 0U)
    {
      continue;
    }

    float iq_avg = iq_sum / (float)count;
    float vq_avg = vq_sum / (float)count;

    float variance =
        iq_sq_sum / (float)count - iq_avg * iq_avg;

    if (variance < 0.0f)
    {
      variance = 0.0f;
    }

    float iq_std = sqrtf(variance);

    uartPrintf(_DEF_UART1,
               "\r\n[%s]\r\n", stage_name[stage]);

    uartPrintf(_DEF_UART1,
               "Samples : %lu\r\n",
               (unsigned long)count);

    uartPrintf(_DEF_UART1,
               "Iq Avg  : %ld mA\r\n",
               (long)(iq_avg * 1000.0f));

    uartPrintf(_DEF_UART1,
               "Iq Std  : %ld mA\r\n",
               (long)(iq_std * 1000.0f));

    uartPrintf(_DEF_UART1,
               "Vq Avg  : %ld mV\r\n",
               (long)(vq_avg * 1000.0f));
  }
}

#endif
        uartPrintf(_DEF_UART1,
                   "============================\r\n");

    #if CURRENT_ROTATION_TEST_ENABLE
    // 마지막 10개 기록 샘플 = 100ms 구간 분석
    #else
    // 마지막 20개 기록 샘플 = 1ms 구간 분석
    #endif
        {
          uint32_t tail_count = CURRENT_DIAG_TAIL_SAMPLES;

          if (tail_count > l_sample_count)
          {
            tail_count = l_sample_count;
          }

          float id_sum = 0.0f;
          float iq_sum = 0.0f;
          float vq_sum = 0.0f;

          float iq_min = 0.0f;
          float iq_max = 0.0f;

          uint32_t valid_count = 0U;

          for (uint32_t i = l_sample_count - tail_count;
               i < l_sample_count; i++)
          {
            if (motorCurrentLTestGetSample(i,
                                           &l_id,
                                           &l_iq,
                                           &l_vd,
                                           &l_vq,
                                           &l_theta) == true)
            {
              id_sum += l_id;
              iq_sum += l_iq;
              vq_sum += l_vq;

              if (valid_count == 0U)
              {
                iq_min = l_iq;
                iq_max = l_iq;
              }
              else
              {
                if (l_iq < iq_min) iq_min = l_iq;
                if (l_iq > iq_max) iq_max = l_iq;
              }

              valid_count++;
            }
          }

          if (valid_count > 0U)
          {
            float id_avg = id_sum / valid_count;
            float iq_avg = iq_sum / valid_count;
            float vq_avg = vq_sum / valid_count;

            uartPrintf(_DEF_UART1,
                       "\r\n===== CURRENT PI TAIL SUMMARY =====\r\n");

            uartPrintf(_DEF_UART1,
                       "Tail Samples : %lu\r\n",
                       (unsigned long)valid_count);

            uartPrintf(_DEF_UART1,
                       "Id Avg       : %ld mA\r\n",
                       (long)(id_avg * 1000.0f));

            uartPrintf(_DEF_UART1,
                       "Iq Avg       : %ld mA\r\n",
                       (long)(iq_avg * 1000.0f));

            uartPrintf(_DEF_UART1,
                       "Iq Min/Max   : %ld / %ld mA\r\n",
                       (long)(iq_min * 1000.0f),
                       (long)(iq_max * 1000.0f));

            uartPrintf(_DEF_UART1,
                       "Vq Avg       : %ld mV\r\n",
                       (long)(vq_avg * 1000.0f));

            uartPrintf(_DEF_UART1,
                       "Iq Error Avg : %ld mA\r\n",
                       (long)((iq_ref_ma / 1000.0f - iq_avg)
                              * 1000.0f));
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
      if (test_fault == MOTOR_FAULT_BKIN)
      {
        apPrintBreakSnapshot();

        uartPrintf(
            _DEF_UART1,
            "NOTE: PA6 is level at IRQ entry, not pulse history.\r\n"
        );
      }
      uartPrintf(_DEF_UART1, "Samples     : %lu\r\n",
                 (unsigned long)current_test_adc_samples);

  #if CURRENT_ROTATION_TEST_ENABLE

      uint32_t elapsed_ms =
          (uint32_t)(now - current_test_start_ms);

      uint32_t adc_avg_hz = 0U;

      if (elapsed_ms > 0U)
      {
        adc_avg_hz =
            (current_test_adc_samples * 1000U) / elapsed_ms;
      }

      uartPrintf(_DEF_UART1,
          "Elapsed    : %lu ms\r\n",
          (unsigned long)elapsed_ms);

      uartPrintf(_DEF_UART1,
          "ADC Avg Hz : %lu\r\n",
          (unsigned long)adc_avg_hz);

  #endif

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

      uint32_t elapsed_ms = now - pre_time_debug;

      pre_time_debug = now;

      if (motorGetMonitor(&monitor) != true)
      {
        continue;
      }

      adc_count = adcGetCurrentUpdateCount();
      adc_diff = adc_count - prev_adc_count;
      prev_adc_count = adc_count;
      adc_hz = (elapsed_ms > 0U)
             ? (adc_diff * 1000U) / elapsed_ms
             : 0U;

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


#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_ADC_RAW_SCAN_ENABLE == 1U)

    motor_adc_raw_scan_t scan = {0};

    motorAdcRawScanGetStats(&scan);

    uartPrintf(_DEF_UART1,
        "SCAN Samples : %lu\r\n",
        (unsigned long)scan.sample_count);

    uartPrintf(_DEF_UART1,
        "Raw Min A/B/C : %u / %u / %u\r\n",
        (unsigned int)scan.raw_min[0],
        (unsigned int)scan.raw_min[1],
        (unsigned int)scan.raw_min[2]);

    uartPrintf(_DEF_UART1,
        "Raw Max A/B/C : %u / %u / %u\r\n",
        (unsigned int)scan.raw_max[0],
        (unsigned int)scan.raw_max[1],
        (unsigned int)scan.raw_max[2]);

    uartPrintf(_DEF_UART1,
        "Ia Min/Max : %ld / %ld mA\r\n",
        (long)(scan.current_min[0] * 1000.0f),
        (long)(scan.current_max[0] * 1000.0f));

    uartPrintf(_DEF_UART1,
        "Ib Min/Max : %ld / %ld mA\r\n",
        (long)(scan.current_min[1] * 1000.0f),
        (long)(scan.current_max[1] * 1000.0f));

    uartPrintf(_DEF_UART1,
        "Ic Min/Max : %ld / %ld mA\r\n",
        (long)(scan.current_min[2] * 1000.0f),
        (long)(scan.current_max[2] * 1000.0f));

    uartPrintf(_DEF_UART1,
        "Spike Count A/B/C : %lu / %lu / %lu\r\n",
        (unsigned long)scan.spike_count[0],
        (unsigned long)scan.spike_count[1],
        (unsigned long)scan.spike_count[2]);

    uartPrintf(_DEF_UART1,
        "First Spike Sample : %lu\r\n",
        (unsigned long)scan.first_spike_sample);

    uartPrintf(_DEF_UART1,
        "First Spike Raw : %u / %u / %u\r\n",
        (unsigned int)scan.first_spike_raw[0],
        (unsigned int)scan.first_spike_raw[1],
        (unsigned int)scan.first_spike_raw[2]);

    uartPrintf(_DEF_UART1,
        "TIM1 MOE : %u\r\n",
        (unsigned int)((TIM1->BDTR & TIM_BDTR_MOE) != 0U));

#elif  (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_FOC_BENCH_ENABLE == 1U)

      uint32_t foc_count = 0U;
      uint32_t foc_total_cycles = 0U;
      uint32_t foc_max_cycles = 0U;

      motorFocBenchGetStats(
          &foc_count,
          &foc_total_cycles,
          &foc_max_cycles);

      uint32_t cycles_per_us = SystemCoreClock / 1000000U;

      uint32_t foc_avg_cycles = 0U;

      if (foc_count > 0U)
      {
          foc_avg_cycles = foc_total_cycles / foc_count;
      }

      uartPrintf(_DEF_UART1,
          "FOC Count  : %lu\r\n",
          (unsigned long)foc_count);

      uartPrintf(_DEF_UART1,
          "FOC Avg us : %lu\r\n",
          (unsigned long)(foc_avg_cycles / cycles_per_us));

      uartPrintf(_DEF_UART1,
          "FOC Max us : %lu\r\n",
          (unsigned long)(foc_max_cycles / cycles_per_us));

      uartPrintf(_DEF_UART1,
          "TIM1 MOE   : %u\r\n",
          (unsigned int)((TIM1->BDTR & TIM_BDTR_MOE) != 0U));

#endif


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
#endif  // CURRENT_ADC_UART_SILENT_TEST 분기
}

