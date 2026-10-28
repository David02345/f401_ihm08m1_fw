/*
 * pwm.h
 *
 *  Created on: 2026. 6. 30.
 *      Author: YDG
 */

#ifndef SRC_COMMON_HW_INCLUDE_PWM_H_
#define SRC_COMMON_HW_INCLUDE_PWM_H_

#include "hw_def.h"


bool pwmInit(void);
void pwmStart(void);
void pwmStop(void);
void pwmEnableOutput(void);
void pwmDisableOutput(void);
void pwmSetDuty(float duty_u, float duty_v, float duty_w);
bool pwmIsBreakFault(void);
void pwmClearBreakFault(void);






// Break IRQ 발생 순간의 레지스터 상태 저장
typedef struct
{
  bool valid;
  uint32_t sr;
  uint32_t dier;
  uint32_t bdtr;
  uint32_t cnt;
  uint32_t gpioa_idr;
} pwm_break_snapshot_t;

// Break ISR 진입 직후 호출
void pwmCaptureBreakIrq(void);

// 저장된 Break 정보 가져오기
bool pwmGetBreakSnapshot(pwm_break_snapshot_t *snapshot);

#endif /* SRC_COMMON_HW_INCLUDE_PWM_H_ */
