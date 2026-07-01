/*
 * foc.c
 *
 *  Created on: 2026. 7. 1.
 *      Author: YDG
 */

#include "foc.h"

#define SQRT_3    1.732


static float focClamp(float value, float min, float max)
{
  if (value < min)
  {
    value = min;
  }

  if (value > max)
  {
    value = max;
  }

  return value;
}

bool focInit(void)
{
  bool ret = true;


  return ret;
}

void focClarke(float ia, float ib, float ic, foc_alphabeta_t *i_ab)
{
  if (i_ab == NULL)
  {
    return;
  }
  (void)ic;

  i_ab->alpha = ia;
  i_ab->beta  = (ia + 2.0f * ib) / SQRT_3;
}
void focPark(float ialpha, float ibeta, float theta_e, foc_dq_t *i_dq)
{
  if (i_dq == NULL)
  {
    return;
  }

  i_dq->d =  ialpha * cosf(theta_e) + ibeta * sinf(theta_e);
  i_dq->q = -ialpha * sinf(theta_e) + ibeta * cosf(theta_e);
}

void focInvPark(float vd, float vq, float theta_e, foc_alphabeta_t *v_ab)
{
  if (v_ab == NULL)
  {
    return;
  }

  v_ab->alpha = vd * cosf(theta_e) - vq * sinf(theta_e);
  v_ab->beta = vd * sinf(theta_e) + vq * cosf(theta_e);
}
void focInvClarke(float valpha, float vbeta, foc_abc_t *v_abc)
{
  if (v_abc == NULL)
  {
    return;
  }

  v_abc->a = valpha;
  v_abc->b = 0.5f * (- valpha + SQRT_3 * vbeta);
  v_abc->c = 0.5f * (- valpha - SQRT_3 * vbeta);
}

void focSetVoltageLimit(foc_dq_t *v_dq, float v_limit)
{
  float mag;
  float scale;

  if (v_dq== NULL)
  {
    return;
  }

  if (v_limit <= 0.0f)
  {
    v_dq->d = 0.0f;
    v_dq->q = 0.0f;
    return;
  }

  mag = sqrtf(v_dq->d * v_dq->d + v_dq->q * v_dq->q);

  if (mag > v_limit)
  {
    scale = v_limit / mag;

    v_dq->d *= scale;
    v_dq->q *= scale;
  }
}
void focGenerateSPWM(float valpha, float vbeta, float vbus, foc_duty_t *duty)
{
  foc_abc_t v_abc;

  if (duty == NULL)
  {
    return;
  }

  if (vbus <= 0.0f)
  {
    duty->u = 0.5f;
    duty->v = 0.5f;
    duty->w = 0.5f;
    return;
  }
  focInvClarke(valpha, vbeta, &v_abc);

  duty->u = 0.5f + (v_abc.a / vbus);
  duty->v = 0.5f + (v_abc.b / vbus);
  duty->w = 0.5f + (v_abc.c / vbus);

  duty->u = focClamp(duty->u, 0.0f, 1.0f);
  duty->v = focClamp(duty->v, 0.0f, 1.0f);
  duty->w = focClamp(duty->w, 0.0f, 1.0f);
}
void focRunOpenLoopVoltage(float vd, float vq, float theta_e, float vbus, foc_duty_t *duty)
{
  foc_dq_t v_dq;
  foc_alphabeta_t v_ab;

  if (duty == NULL)
  {
    return;
  }
  if (vbus <= 0.0f)
  {
    duty->u = 0.5f;
    duty->v = 0.5f;
    duty->w = 0.5f;
    return;
  }

  v_dq.d = vd;
  v_dq.q = vq;

  focSetVoltageLimit(&v_dq, 0.5f * vbus);

  focInvPark(v_dq.d, v_dq.q, theta_e, &v_ab);

#if defined(_USE_FOC_SVPWM)
  svpwmGenerate(v_ab.alpha, v_ab.beta, vbus, duty);
#elif defined (_USE_FOC_SPWM)
  focGenerateSPWM(v_ab.alpha, v_ab.beta, vbus, duty);
#else
  duty->u = 0.5f;
  duty->v = 0.5f;
  duty->w = 0.5f;

#endif
}
