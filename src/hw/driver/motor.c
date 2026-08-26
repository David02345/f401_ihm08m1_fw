/*
 * motor.c
 *
 *  Created on: 2026. 7. 4.
 *      Author: YDG
 */

#include "motor.h"
#include "motor_types.h"
#include "util.h"


static volatile motor_state_t motor_state;
static volatile motor_fault_t motor_fault = MOTOR_FAULT_NONE;

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)

// [수정] ADC ISR 기준 시험 시간 관리
static volatile uint32_t current_diag_sample_count = 0U;
static volatile bool current_diag_active = false;
static volatile bool current_diag_done = false;
static volatile motor_abc_f_t current_diag_i_min;
static volatile motor_abc_f_t current_diag_i_max;
static volatile float current_diag_id_sum = 0.0f;
static volatile float current_diag_iq_sum = 0.0f;

// [추가] 마지막 100 samples = 마지막 5 ms 평균
#define CURRENT_DIAG_TAIL_SAMPLES       100U

static volatile uint32_t current_diag_tail_count = 0U;

static volatile float current_diag_tail_id_sum = 0.0f;
static volatile float current_diag_tail_iq_sum = 0.0f;

static volatile float current_diag_tail_vd_sum = 0.0f;
static volatile float current_diag_tail_vq_sum = 0.0f;

// [L TEST] 첫 3 ms current response 저장
static volatile float current_l_test_id[CURRENT_L_TEST_SAMPLE_COUNT];
static volatile float current_l_test_iq[CURRENT_L_TEST_SAMPLE_COUNT];
static volatile float current_l_test_theta[CURRENT_L_TEST_SAMPLE_COUNT];

static volatile uint32_t current_l_test_count = 0U;

#endif


#if !((MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
      (CURRENT_NEUTRAL_DIAG_ENABLE == 1U))
static motor_duty_t motor_duty;
#endif
static motor_duty_t motor_duty;
static pid_ctrl_t pi_id;
static pid_ctrl_t pi_iq;
static pid_ctrl_t pi_spd;

static volatile float motor_vbus = 0.0f;
static volatile uint16_t motor_speed_cmd_raw = 0;
static uint16_t motor_temp_raw = 0;

static float speed_w_ref  = 0.0f;
static float speed_w_meas = 0.0f;
static float speed_w_target = 0.0f;

static uint16_t speed_loop_divider = 0;

static float current_id_ref = 0.0f;
static float current_iq_ref = 0.0f;

static volatile motor_monitor_t motor_monitor;

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP

static float open_loop_theta_e = 0.0f;
static float open_loop_speed_e = 0.0f;
static float open_loop_vd = 0.0f;
static float open_loop_vq = 0.0f;

static uint32_t open_loop_align_count = 0U;
static bool open_loop_alignment_done = false;

#if _USE_HALL_OFFSET_CALIBRATION

static volatile bool hall_cal_event_pending = false;
static volatile int8_t hall_cal_event_sector = -1;
static volatile int8_t hall_cal_event_direction = 0;
static volatile float hall_cal_event_theta_e = 0.0f;

static int8_t hall_cal_prev_sector = -1;

#endif
#endif
static void motorCurrentLoop(float id_ref, float iq_ref, float theta_e);
bool motorInit(void)
{
  bool ret = true;

  motor_state = MOTOR_STATE_IDLE;
  motor_fault = MOTOR_FAULT_NONE;

  if(pidInit(&pi_id, CUR_KP, CUR_KI, CUR_KD, OUTPUT_VD_REF_MIN, OUTPUT_VD_REF_MAX, INTEGRAL_ID_MIN, INTEGRAL_ID_MAX) != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
  if(pidInit(&pi_iq, CUR_KP, CUR_KI, CUR_KD, OUTPUT_VQ_REF_MIN, OUTPUT_VQ_REF_MAX, INTEGRAL_IQ_MIN, INTEGRAL_IQ_MAX) != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
  if(pidInit(&pi_spd, SPD_KP, SPD_KI, SPD_KD, OUTPUT_IQ_REF_MIN, OUTPUT_IQ_REF_MAX ,INTEGRAL_SPD_MIN, INTEGRAL_SPD_MAX) != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
  if(adcInit() != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
  else
  {
    adcSetInjectedCallback(motorControlUpdate);
  }

  if(pwmInit() != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
  if(focInit() != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }

#if _USE_HALL_SENSOR
  if(hallInit() != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
#elif _USE_ENCODER
  if(encoderInit() != true)
  {
    ret = false;
    motor_state = MOTOR_STATE_FAULT;
  }
#endif
  if (ret != true)
  {
    motorSetFault(MOTOR_FAULT_INIT_FAIL);
  }

  return ret;
}

void motorStart(void)
{
  if(motor_state != MOTOR_STATE_IDLE)
  {
    return;
  }

  if(adcUpdateRegular() != true)
  {
    motorSetFault(MOTOR_FAULT_ADC_REGULAR_FAIL);
    return;
  }

  motor_vbus = adcGetVbusVoltage();
  motor_speed_cmd_raw = adcGetSpeedRaw();
  motor_temp_raw = adcGetTempRaw();

  if(motor_vbus < MOTOR_VBUS_MIN)
  {
    motorSetFault(MOTOR_FAULT_VBUS_LOW);
    return;
  }

  pidReset(&pi_id);
  pidReset(&pi_iq);
  pidReset(&pi_spd);

  speed_loop_divider = 0;

  speed_w_target = 0.0f;
  speed_w_ref = 0.0f;
  speed_w_meas = 0.0f;

  current_id_ref = 0.0f;
  current_iq_ref = 0.0f;

  adcInjectedStart();
  pwmStart();

  if(adcCalibrateCurrentOffset() != true)
  {
    motorSetFault(MOTOR_FAULT_ADC_OFFSET_FAIL);
    return;
  }


#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT

  motor_state = MOTOR_STATE_CURRENT_LOOP;

#elif MOTOR_CONTROL_MODE == MOTOR_CONTROL_SPEED

  motor_state = MOTOR_STATE_SPEED_LOOP;

#elif MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP

#if _USE_HALL_OFFSET_CALIBRATION

  open_loop_theta_e = 0.0f;
  open_loop_speed_e = 0.0f;

  open_loop_vd = HALL_CAL_ALIGN_VD;
  open_loop_vq = 0.0f;

  open_loop_align_count = 0U;
  open_loop_alignment_done = false;

  hall_cal_event_pending = false;
  hall_cal_event_sector = -1;
  hall_cal_event_direction = 0;
  hall_cal_event_theta_e = 0.0f;

  hall_cal_prev_sector = hallGetSectorIndex();

#else

  open_loop_theta_e = OPEN_LOOP_ALIGN_THETA_E;
  open_loop_speed_e = 0.0f;

  open_loop_vd = 0.0f;
  open_loop_vq = OPEN_LOOP_ALIGN_VQ;

  open_loop_align_count = 0U;
  open_loop_alignment_done = false;

#endif

  motor_state = MOTOR_STATE_OPEN_LOOP;

#endif

  //pwmEnableOutput();
}




#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)

void motorCurrentDiagStart(void)
{
  uint32_t primask;


  /*
   * [수정]
   * Current loop 시작 전 PWM을 neutral 상태로 준비.
   *
   * 첫 ADC ISR부터 P-only current loop가
   * 새로운 duty를 계산한다.
   */
  motor_duty.u = 0.5f;
  motor_duty.v = 0.5f;
  motor_duty.w = 0.5f;

  pwmSetDuty(motor_duty.u, motor_duty.v, motor_duty.w);


  primask = __get_PRIMASK();
  __disable_irq();


  /*
   * 30 ms test counter reset
   */
  current_diag_sample_count = 0U;
  current_diag_done = false;

  current_diag_id_sum = 0.0f;
  current_diag_iq_sum = 0.0f;

  // [추가] 마지막 100 samples 평균용 초기화
  current_diag_tail_count = 0U;

  current_diag_tail_id_sum = 0.0f;
  current_diag_tail_iq_sum = 0.0f;

  current_diag_tail_vd_sum = 0.0f;
  current_diag_tail_vq_sum = 0.0f;

  current_diag_i_min.a = 0.0f;
  current_diag_i_min.b = 0.0f;
  current_diag_i_min.c = 0.0f;

  current_diag_i_max.a = 0.0f;
  current_diag_i_max.b = 0.0f;
  current_diag_i_max.c = 0.0f;

  current_diag_active = true;

  // [L TEST]
  current_l_test_count = 0U;

  for (uint32_t i = 0U;
       i < CURRENT_L_TEST_SAMPLE_COUNT;
       i++)
  {
    current_l_test_id[i] = 0.0f;
    current_l_test_iq[i] = 0.0f;
    current_l_test_theta[i] = 0.0f;
  }

  /*
   * 실제 출력 시작
   */
  pwmEnableOutput();


  if (primask == 0U)
  {
    __enable_irq();
  }
}


bool motorCurrentDiagIsDone(void)
{
  return current_diag_done;
}


uint32_t motorCurrentDiagGetSampleCount(void)
{
  return current_diag_sample_count;
}

void motorCurrentDiagGetDQAverage(float *id_avg, float *iq_avg)
{
  uint32_t sample_count;

  sample_count = current_diag_sample_count;

  if (id_avg != NULL)
  {
    if (sample_count > 0U)
    {
      *id_avg = current_diag_id_sum / (float)sample_count;
    }
    else
    {
      *id_avg = 0.0f;
    }
  }

  if (iq_avg != NULL)
  {
    if (sample_count > 0U)
    {
      *iq_avg = current_diag_iq_sum / (float)sample_count;
    }
    else
    {
      *iq_avg = 0.0f;
    }
  }
}

void motorCurrentDiagGetTailAverage(float *id_avg, float *iq_avg, float *vd_avg, float *vq_avg)
{
  uint32_t count;

  count = current_diag_tail_count;


  if (id_avg != NULL)
  {
    if (count > 0U)
    {
      *id_avg =
          current_diag_tail_id_sum / (float)count;
    }
    else
    {
      *id_avg = 0.0f;
    }
  }


  if (iq_avg != NULL)
  {
    if (count > 0U)
    {
      *iq_avg =
          current_diag_tail_iq_sum / (float)count;
    }
    else
    {
      *iq_avg = 0.0f;
    }
  }


  if (vd_avg != NULL)
  {
    if (count > 0U)
    {
      *vd_avg =
          current_diag_tail_vd_sum / (float)count;
    }
    else
    {
      *vd_avg = 0.0f;
    }
  }


  if (vq_avg != NULL)
  {
    if (count > 0U)
    {
      *vq_avg =
          current_diag_tail_vq_sum / (float)count;
    }
    else
    {
      *vq_avg = 0.0f;
    }
  }
}

void motorCurrentDiagGetCurrentMinMax(motor_abc_f_t *i_min, motor_abc_f_t *i_max)
{
  if (i_min != NULL)
  {
    i_min->a = current_diag_i_min.a;
    i_min->b = current_diag_i_min.b;
    i_min->c = current_diag_i_min.c;
  }

  if (i_max != NULL)
  {
    i_max->a = current_diag_i_max.a;
    i_max->b = current_diag_i_max.b;
    i_max->c = current_diag_i_max.c;
  }
}

uint32_t motorCurrentLTestGetCount(void)
{
  return current_l_test_count;
}


bool motorCurrentLTestGetSample(uint32_t index, float *id, float *iq, float *theta_e)
{
  if (index >= current_l_test_count)
  {
    return false;
  }

  if (id != NULL)
  {
    *id = current_l_test_id[index];
  }

  if (iq != NULL)
  {
    *iq = current_l_test_iq[index];
  }

  if (theta_e != NULL)
  {
    *theta_e = current_l_test_theta[index];
  }

  return true;
}
#endif




void motorStop(void)
{
  pwmDisableOutput();
  adcInjectedStop();
  pwmStop();

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)

  // [수정]
  current_diag_active = false;

#endif

  pidReset(&pi_id);
  pidReset(&pi_iq);
  pidReset(&pi_spd);

  speed_loop_divider = 0;

  speed_w_target = 0.0f;
  speed_w_ref = 0.0f;
  speed_w_meas = 0.0f;

  current_id_ref = 0.0f;
  current_iq_ref = 0.0f;

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP

  open_loop_theta_e = 0.0f;
  open_loop_speed_e = 0.0f;
  open_loop_vd = 0.0f;
  open_loop_vq = 0.0f;

  open_loop_align_count = 0U;
  open_loop_alignment_done = false;

#if _USE_HALL_OFFSET_CALIBRATION

  hall_cal_event_pending = false;
  hall_cal_event_sector = -1;
  hall_cal_event_direction = 0;
  hall_cal_event_theta_e = 0.0f;
  hall_cal_prev_sector = -1;

#endif

#endif

  motor_state = MOTOR_STATE_IDLE;
}
void motorLowSpeedTask(void)
{

  if (motor_state == MOTOR_STATE_FAULT)
  {
    return;
  }

#if _USE_HALL_SENSOR
  hallUpdateTimeout();
#endif

  if (pwmIsBreakFault() == true)
  {
    motorSetFault(MOTOR_FAULT_BKIN);
    return;
  }

  if ((motor_state == MOTOR_STATE_CURRENT_LOOP) ||
      (motor_state == MOTOR_STATE_SPEED_LOOP)   ||
      (motor_state == MOTOR_STATE_OPEN_LOOP))
  {
    return;
  }

  if (adcUpdateRegular() != true)
  {
    motorSetFault(MOTOR_FAULT_ADC_REGULAR_FAIL);
    return;
  }

  motor_vbus = adcGetVbusVoltage();
  motor_speed_cmd_raw = adcGetSpeedRaw();
  motor_temp_raw = adcGetTempRaw();

  if (motor_vbus < MOTOR_VBUS_MIN)
  {
    motorSetFault(MOTOR_FAULT_VBUS_LOW);
    return;
  }
}
/*
void motorLowSpeedTask(void)
{
  if(motor_state == MOTOR_STATE_FAULT)
  {
    return;
  }

#if _USE_HALL_SENSOR
    hallUpdateTimeout();
#endif

  if (pwmIsBreakFault())
  {
    motorSetFault(MOTOR_FAULT_BKIN);
    return;
  }

  if(adcUpdateRegular() != true)
  {
    motorSetFault(MOTOR_FAULT_ADC_REGULAR_FAIL);
    return;
  }
  motor_vbus = adcGetVbusVoltage();

  if(motor_vbus < MOTOR_VBUS_MIN)
  {
    motorSetFault(MOTOR_FAULT_VBUS_LOW);
    return;
  }
  motor_speed_cmd_raw = adcGetSpeedRaw();
  motor_temp_raw = adcGetTempRaw();
}
*/
#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)

static void motorNeutralPwmDiag(void)
{
  float theta_e;


  /*
   * 30 ms test가 시작되지 않았거나
   * 이미 완료됐다면 아무것도 하지 않는다.
   */
  if (current_diag_active == false)
  {
    return;
  }


  /*
   * [수정]
   * 실제 Current FOC에서 사용할 rotor electrical angle
   */
#if _USE_HALL_SENSOR

  theta_e = hallGetElectricalAngle();

#elif _USE_ENCODER

  theta_e = encoderGetElectricalAngle();

#else

#error "HALL SENSOR / ENCODER ERROR"

#endif


  motorCurrentLoop(current_id_ref, current_iq_ref, theta_e);


  if (motor_state == MOTOR_STATE_FAULT)
  {
    current_diag_active = false;
    current_diag_done = false;

    return;
  }

  /*
   * [L TEST]
   * motorCurrentLoop()에서 이번 ISR의 Id/Iq 계산 완료.
   *
   * 첫 sample은 voltage step 직전/직후의 기준점 역할을 한다.
   */
  if (current_l_test_count < CURRENT_L_TEST_SAMPLE_COUNT)
  {
    current_l_test_id[current_l_test_count] = motor_monitor.id_meas;
    current_l_test_iq[current_l_test_count] = motor_monitor.iq_meas;
    current_l_test_theta[current_l_test_count] = motor_monitor.theta_e;
    current_l_test_count++;
  }

  current_diag_sample_count++;

  current_diag_id_sum += motor_monitor.id_meas;
  current_diag_iq_sum += motor_monitor.iq_meas;

  if ((current_diag_sample_count + CURRENT_DIAG_TAIL_SAMPLES) >
      CURRENT_LOOP_TEST_SAMPLE_COUNT)
  {
    current_diag_tail_count++;

    current_diag_tail_id_sum += motor_monitor.id_meas;
    current_diag_tail_iq_sum += motor_monitor.iq_meas;

    current_diag_tail_vd_sum += motor_monitor.vd_cmd;
    current_diag_tail_vq_sum += motor_monitor.vq_cmd;
  }

  if (current_diag_sample_count == 1U)
  {
    current_diag_i_min.a = motor_monitor.ia;
    current_diag_i_min.b = motor_monitor.ib;
    current_diag_i_min.c = motor_monitor.ic;

    current_diag_i_max.a = motor_monitor.ia;
    current_diag_i_max.b = motor_monitor.ib;
    current_diag_i_max.c = motor_monitor.ic;
  }
  else
  {
    if (motor_monitor.ia < current_diag_i_min.a)
    {
      current_diag_i_min.a = motor_monitor.ia;
    }

    if (motor_monitor.ia > current_diag_i_max.a)
    {
      current_diag_i_max.a = motor_monitor.ia;
    }


    if (motor_monitor.ib < current_diag_i_min.b)
    {
      current_diag_i_min.b = motor_monitor.ib;
    }

    if (motor_monitor.ib > current_diag_i_max.b)
    {
      current_diag_i_max.b = motor_monitor.ib;
    }


    if (motor_monitor.ic < current_diag_i_min.c)
    {
      current_diag_i_min.c = motor_monitor.ic;
    }

    if (motor_monitor.ic > current_diag_i_max.c)
    {
      current_diag_i_max.c = motor_monitor.ic;
    }
  }


  /*
   * motorCurrentLoop()에서 0.8 A trip이 발생했다면
   * 이미 MOE는 OFF 되어 있다.
   */

  /*
   * [안전]
   * 600 samples = 30 ms
   *
   * main loop가 아니라 ADC ISR에서 직접 MOE OFF.
   */
  if (current_diag_sample_count >=
      CURRENT_LOOP_TEST_SAMPLE_COUNT)
  {
    pwmDisableOutput();

    current_diag_active = false;
    current_diag_done = true;

    return;
  }
}

#endif

static void motorCurrentLoop(float id_ref, float iq_ref, float theta_e)
{
  motor_abc_f_t i_abc;
  motor_alphabeta_t i_ab;
  motor_dq_t i_dq;
  motor_dq_t v_dq;
  motor_alphabeta_t v_ab;
  float max_phase_current;

  adcGetPhaseCurrent(&i_abc);

  motor_monitor.ia = i_abc.a;
  motor_monitor.ib = i_abc.b;
  motor_monitor.ic = i_abc.c;

  max_phase_current = fabsf(i_abc.a);

  if (fabsf(i_abc.b) > max_phase_current)
  {
    max_phase_current = fabsf(i_abc.b);
  }

  if (fabsf(i_abc.c) > max_phase_current)
  {
    max_phase_current = fabsf(i_abc.c);
  }

  if (max_phase_current > CURRENT_TEST_OC_LIMIT_A)
  {
    pwmDisableOutput();

    motor_fault = MOTOR_FAULT_SW_OVERCURRENT;
    motor_state = MOTOR_STATE_FAULT;
    return;
  }



  focClarke(i_abc.a, i_abc.b, i_abc.c, &i_ab);
  focPark(i_ab.alpha, i_ab.beta, theta_e, &i_dq);

  motor_monitor.id_ref = id_ref;
  motor_monitor.id_meas = i_dq.d;
  motor_monitor.iq_ref = iq_ref;
  motor_monitor.iq_meas = i_dq.q;

  motor_monitor.theta_e   = theta_e;


  //v_dq.d = piController(&pi_id, id_ref, i_dq.d, CUR_DT);
  //v_dq.q = piController(&pi_iq, iq_ref, i_dq.q, CUR_DT);
  v_dq.d = CURRENT_L_TEST_VD;
  v_dq.q = CURRENT_L_TEST_VQ;

  /*
  if (v_dq.q > 0.08f)
  {
    v_dq.q = 0.08f;
  }
  else if (v_dq.q < -0.08f)
  {
    v_dq.q = -0.08f;
  }
*/
#if _USE_FOC_SPWM
  focSetVoltageLimit(&v_dq, MOTOR_VLIMIT_SPWM);
#elif _USE_FOC_SVPWM
  focSetVoltageLimit(&v_dq, MOTOR_VLIMIT_SVPWM);
#endif

  motor_monitor.vd_cmd = v_dq.d;
  motor_monitor.vq_cmd = v_dq.q;

  focInvPark(v_dq.d, v_dq.q, theta_e, &v_ab);

#if _USE_FOC_SPWM
  focGenerateSPWM(v_ab.alpha, v_ab.beta, motor_vbus, &motor_duty);
#elif _USE_FOC_SVPWM
  focGenerateSVPWM(v_ab.alpha, v_ab.beta, motor_vbus, &motor_duty);
#endif

  motor_monitor.duty_u = motor_duty.u;
  motor_monitor.duty_v = motor_duty.v;
  motor_monitor.duty_w = motor_duty.w;

  pwmSetDuty(motor_duty.u, motor_duty.v, motor_duty.w);
}

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_SPEED
static void motorSpeedLoop(void)
{
  float ratio;
  float delta;
  float ramp_step;

  if (motor_state != MOTOR_STATE_SPEED_LOOP)
  {
    return;
  }

#if _USE_HALL_SENSOR
  speed_w_meas = hallGetMechanicalSpeed();
  motor_monitor.speed_meas = speed_w_meas;
#elif _USE_ENCODER
  speed_w_meas = encoderGetMechanicalSpeed();
  motor_monitor.speed_meas = speed_w_meas;
#endif

  if (motor_speed_cmd_raw <= SPEED_CMD_DEADBAND_RAW)
  {
    speed_w_target = 0.0f;
  }
  else
  {
    ratio = (float)(motor_speed_cmd_raw - SPEED_CMD_DEADBAND_RAW) /
            (float)(4095U - SPEED_CMD_DEADBAND_RAW);
    ratio = clampFloat(ratio, 0.0f, 1.0f);

    speed_w_target = ratio * OUTPUT_SPD_REF_MAX;
  }

  delta = speed_w_target - speed_w_ref;

  if (delta > 0.0f)
  {
    ramp_step = SPEED_RAMP_UP * SPD_DT;

    if (delta > ramp_step)
    {
      speed_w_ref += ramp_step;
    }
    else
    {
      speed_w_ref = speed_w_target;
    }
  }
  else if (delta < 0.0f)
  {
    ramp_step = SPEED_RAMP_DOWN * SPD_DT;

    if (delta < -ramp_step)
    {
      speed_w_ref -= ramp_step;
    }
    else
    {
      speed_w_ref = speed_w_target;
    }
  }

  speed_w_ref = clampFloat(speed_w_ref, 0.0f, OUTPUT_SPD_REF_MAX);
  motor_monitor.speed_ref = speed_w_ref;

  current_id_ref = 0.0f;
  current_iq_ref = piController(&pi_spd, speed_w_ref, speed_w_meas, SPD_DT);
}
#endif

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP

static void motorOpenLoop(void)
{
  motor_abc_f_t i_abc;

#if _USE_HALL_OFFSET_CALIBRATION
  int8_t current_sector;
#endif

  if (motor_state != MOTOR_STATE_OPEN_LOOP)
  {
    return;
  }

  adcGetPhaseCurrent(&i_abc);

  motor_monitor.ia = i_abc.a;
  motor_monitor.ib = i_abc.b;
  motor_monitor.ic = i_abc.c;

#if _USE_HALL_OFFSET_CALIBRATION

  if (open_loop_alignment_done == false)
  {
    open_loop_theta_e = 0.0f;
    open_loop_speed_e = 0.0f;

    open_loop_vd = HALL_CAL_ALIGN_VD;
    open_loop_vq = 0.0f;

    current_sector = hallGetSectorIndex();

    if (current_sector >= 0)
    {
      hall_cal_prev_sector = current_sector;
    }

    open_loop_align_count++;

    if (open_loop_align_count >= HALL_CAL_ALIGN_COUNT)
    {
      open_loop_alignment_done = true;

      open_loop_speed_e = HALL_CAL_DIRECTION * HALL_CAL_SPEED_E;

      open_loop_vd = HALL_CAL_ROTATE_VD;
      open_loop_vq = 0.0f;

      hall_cal_prev_sector = hallGetSectorIndex();
    }
  }
  else
  {
    open_loop_speed_e =
        HALL_CAL_DIRECTION * HALL_CAL_SPEED_E;

    open_loop_theta_e += open_loop_speed_e * OPEN_DT;

    open_loop_theta_e = wrapFloat(open_loop_theta_e, 0.0f, 2.0f * PI);

    open_loop_vd = HALL_CAL_ROTATE_VD;
    open_loop_vq = 0.0f;

    current_sector = hallGetSectorIndex();

    if (current_sector >= 0)
    {
      if (hall_cal_prev_sector < 0)
      {
        hall_cal_prev_sector = current_sector;
      }
      else if (current_sector != hall_cal_prev_sector)
      {
        int8_t sector_diff;
        int8_t event_direction = 0;
        int8_t expected_direction;

        sector_diff = current_sector - hall_cal_prev_sector;

        if ((sector_diff == -1) || (sector_diff == 5))
        {
          event_direction = 1;
        }
        else if ((sector_diff == 1) || (sector_diff == -5))
        {
          event_direction = -1;
        }
        else
        {
          event_direction = 0;
        }

        expected_direction =
            (HALL_CAL_DIRECTION >= 0.0f) ? 1 : -1;

        if (event_direction == expected_direction)
        {
          hall_cal_event_sector = current_sector;
          hall_cal_event_direction = event_direction;
          hall_cal_event_theta_e = open_loop_theta_e;
          hall_cal_event_pending = true;
        }

        hall_cal_prev_sector = current_sector;
      }
    }
  }

#else

  if (open_loop_alignment_done == false)
  {
    open_loop_theta_e = OPEN_LOOP_ALIGN_THETA_E;
    open_loop_speed_e = 0.0f;

    open_loop_vd = 0.0f;
    open_loop_vq = OPEN_LOOP_ALIGN_VQ;

    open_loop_align_count++;

    if (open_loop_align_count >= OPEN_LOOP_ALIGN_COUNT)
    {
      open_loop_alignment_done = true;

      open_loop_speed_e = OPEN_LOOP_START_SPEED_E;

      open_loop_vd = 0.0f;
      open_loop_vq = OPEN_LOOP_TARGET_VQ;
    }
  }
  else
  {
    open_loop_speed_e += OPEN_LOOP_ACCEL_E * OPEN_DT;

    open_loop_speed_e = clampFloat(open_loop_speed_e, OPEN_LOOP_START_SPEED_E, OPEN_LOOP_TARGET_SPEED);

    open_loop_theta_e += open_loop_speed_e * OPEN_DT;

    open_loop_theta_e = wrapFloat(open_loop_theta_e, 0.0f, 2.0f * PI);
  }

#endif

  focRunOpenLoopVoltage(open_loop_vd, open_loop_vq, open_loop_theta_e, motor_vbus, &motor_duty);

  motor_monitor.theta_e = open_loop_theta_e;

  motor_monitor.id_ref = 0.0f;
  motor_monitor.id_meas = 0.0f;
  motor_monitor.iq_ref = 0.0f;
  motor_monitor.iq_meas = 0.0f;

  motor_monitor.vd_cmd = open_loop_vd;
  motor_monitor.vq_cmd = open_loop_vq;

  motor_monitor.duty_u = motor_duty.u;
  motor_monitor.duty_v = motor_duty.v;
  motor_monitor.duty_w = motor_duty.w;

  pwmSetDuty(motor_duty.u,
             motor_duty.v,
             motor_duty.w);
}

#endif
/*
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP
static void motorOpenLoop(void)
{
  motor_abc_f_t i_abc;

  if(motor_state != MOTOR_STATE_OPEN_LOOP)
  {
    return;
  }

  adcGetPhaseCurrent(&i_abc);

  motor_monitor.ia = i_abc.a;
  motor_monitor.ib = i_abc.b;
  motor_monitor.ic = i_abc.c;

  open_loop_speed_e += OPEN_LOOP_ACCEL_E * OPEN_DT;
  open_loop_speed_e = clampFloat(open_loop_speed_e, 0.5f, OPEN_LOOP_TARGET_SPEED);
  open_loop_theta_e += open_loop_speed_e * OPEN_DT;
  open_loop_theta_e = wrapFloat(open_loop_theta_e, 0.0f, 2.0f * PI);
  focRunOpenLoopVoltage(open_loop_vd, open_loop_vq, open_loop_theta_e, motor_vbus, &motor_duty);

  motor_monitor.theta_e = open_loop_theta_e;

  motor_monitor.id_ref  = 0.0f;
  motor_monitor.id_meas = 0.0f;
  motor_monitor.iq_ref  = 0.0f;
  motor_monitor.iq_meas = 0.0f;

  motor_monitor.vd_cmd  = open_loop_vd;
  motor_monitor.vq_cmd  = open_loop_vq;

  motor_monitor.duty_u = motor_duty.u;
  motor_monitor.duty_v = motor_duty.v;
  motor_monitor.duty_w = motor_duty.w;

  pwmSetDuty(motor_duty.u, motor_duty.v, motor_duty.w);
}
#endif
*/
void motorControlUpdate(void)
{
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT
#if CURRENT_NEUTRAL_DIAG_ENABLE == 0U
  float theta_e;
#endif

  if (motor_state != MOTOR_STATE_CURRENT_LOOP)
  {
      return;
  }
#if CURRENT_NEUTRAL_DIAG_ENABLE == 1U
  motorNeutralPwmDiag();
#else
#if _USE_HALL_SENSOR
  theta_e = hallGetElectricalAngle();
#elif _USE_ENCODER
  theta_e = encoderGetElectricalAngle();
#endif

  motorCurrentLoop(current_id_ref, current_iq_ref, theta_e);
#endif

#elif MOTOR_CONTROL_MODE == MOTOR_CONTROL_SPEED
  float theta_e;

  if (motor_state != MOTOR_STATE_SPEED_LOOP)
  {
      return;
  }

#if _USE_HALL_SENSOR
  theta_e = hallGetElectricalAngle();
#elif _USE_ENCODER
  theta_e = encoderGetElectricalAngle();
#else
#error "HALL SENSOR / ENCODER ERROR"
#endif

  speed_loop_divider++;

  if (speed_loop_divider >= SPEED_LOOP_DIVIDER)
  {
    speed_loop_divider = 0;
    motorSpeedLoop();       // iq_ref만 갱신
  }

  motorCurrentLoop(current_id_ref, current_iq_ref, theta_e);

#endif

#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP
  if (motor_state != MOTOR_STATE_OPEN_LOOP)
  {
      return;
  }

  motorOpenLoop();
#endif
}

void motorSetFault(motor_fault_t fault)
{
  pwmDisableOutput();

  adcInjectedStop();
  pwmStop();

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT) && \
    (CURRENT_NEUTRAL_DIAG_ENABLE == 1U)

  // [수정]
  current_diag_active = false;

#endif

  pidReset(&pi_id);
  pidReset(&pi_iq);
  pidReset(&pi_spd);

  speed_w_target = 0.0f;
  speed_w_ref = 0.0f;
  speed_w_meas = 0.0f;

  current_id_ref = 0.0f;
  current_iq_ref = 0.0f;

  motor_fault = fault;
  motor_state = MOTOR_STATE_FAULT;
}

void motorClearFault(void)
{
  if (motor_state != MOTOR_STATE_FAULT)
  {
    return;
  }

  pwmClearBreakFault();

  motor_fault = MOTOR_FAULT_NONE;
  motor_state = MOTOR_STATE_IDLE;
}

bool motorGetMonitor(motor_monitor_t *monitor)
{
  uint32_t primask;

  if (monitor == NULL)
  {
   return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  monitor->ia = motor_monitor.ia;
  monitor->ib = motor_monitor.ib;
  monitor->ic = motor_monitor.ic;

  monitor->id_ref  = motor_monitor.id_ref;
  monitor->id_meas = motor_monitor.id_meas;
  monitor->iq_ref  = motor_monitor.iq_ref;
  monitor->iq_meas = motor_monitor.iq_meas;

  monitor->vd_cmd = motor_monitor.vd_cmd;
  monitor->vq_cmd = motor_monitor.vq_cmd;

  monitor->theta_e = motor_monitor.theta_e;

  monitor->speed_target = speed_w_target;
  monitor->speed_ref    = speed_w_ref;
  monitor->speed_meas   = speed_w_meas;

  monitor->duty_u = motor_monitor.duty_u;
  monitor->duty_v = motor_monitor.duty_v;
  monitor->duty_w = motor_monitor.duty_w;

  monitor->vbus = motor_vbus;
  monitor->temp_raw = motor_temp_raw;
  monitor->speed_cmd_raw = motor_speed_cmd_raw;

  monitor->state = motor_state;
  monitor->fault = motor_fault;

  if (primask == 0U)
  {
    __enable_irq();
  }

  return true;
}

void motorSetCurrentReference(float id_ref, float iq_ref)
{
#if MOTOR_CONTROL_MODE == MOTOR_CONTROL_CURRENT

  current_id_ref = clampFloat(id_ref, OUTPUT_ID_REF_MIN, OUTPUT_ID_REF_MAX);
  current_iq_ref = clampFloat(iq_ref, OUTPUT_IQ_REF_MIN, OUTPUT_IQ_REF_MAX);

#else
  (void)id_ref;
  (void)iq_ref;
#endif
}

motor_state_t motorGetState(void)
{
  return motor_state;
}

motor_fault_t motorGetFault(void)
{
  return motor_fault;
}

#if (MOTOR_CONTROL_MODE == MOTOR_CONTROL_OPEN_LOOP) && (_USE_HALL_OFFSET_CALIBRATION)

bool motorGetHallCalibrationEvent(int8_t *sector, int8_t *direction, float *theta_e)
{
  bool ret = false;
  uint32_t primask;

  if ((sector == NULL) || (direction == NULL) || (theta_e == NULL))
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  if (hall_cal_event_pending == true)
  {
    *sector = hall_cal_event_sector;
    *direction = hall_cal_event_direction;
    *theta_e = hall_cal_event_theta_e;

    hall_cal_event_pending = false;
    ret = true;
  }

  if (primask == 0U)
  {
    __enable_irq();
  }

  return ret;
}

#endif
