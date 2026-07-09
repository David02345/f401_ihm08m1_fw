/*
 * pwm.c
 *
 *  Created on: 2026. 6. 30.
 *      Author: YDG
 */
#include "pwm.h"
#include "util.h"

TIM_HandleTypeDef htim1;

TIM_ClockConfigTypeDef sClockSourceConfig = {0};
TIM_MasterConfigTypeDef sMasterConfig = {0};
TIM_OC_InitTypeDef sConfigOC = {0};
TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle);

static volatile bool pwm_break_fault = false;


bool pwmInit(void)
{
  bool ret = true;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = PWM_ARR;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = PWM_INIT_PULSE;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = PWM_ADC_TRIG_PULSE;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = PWM_DEADTIME;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }

  HAL_TIM_MspPostInit(&htim1);

  pwmSetDuty(0.5f, 0.5f, 0.5f);
  pwmDisableOutput();

  return ret;
}

void pwmStart(void)
{
  pwmDisableOutput();
  pwmSetDuty(0.5f, 0.5f, 0.5f);

  htim1.Instance->CCER |= (TIM_CCER_CC1E  | TIM_CCER_CC2E  | TIM_CCER_CC3E | TIM_CCER_CC4E |
                           TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE);

  __HAL_TIM_ENABLE(&htim1);
}

void pwmStop(void)
{
  pwmDisableOutput();

  htim1.Instance->CCER &= ~(TIM_CCER_CC1E  | TIM_CCER_CC2E  | TIM_CCER_CC3E | TIM_CCER_CC4E |
                            TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE);

  __HAL_TIM_DISABLE(&htim1);
}

void pwmEnableOutput(void)
{
  __HAL_TIM_MOE_ENABLE(&htim1);
}

void pwmDisableOutput(void)
{
  __HAL_TIM_MOE_DISABLE(&htim1);
}


static uint32_t pwmDutyToCCR(float duty)
{
  uint32_t arr = PWM_ARR;
  uint32_t ccr;

  duty = clampFloat(duty, 0.0f, 1.0f);

  ccr = (uint32_t)((float)(arr + 1) * duty);

  if (ccr > PWM_ARR)
  {
    ccr = PWM_ARR;
  }

  return ccr;
}
void pwmSetDuty(float duty_u, float duty_v, float duty_w)
{
  uint32_t ccr_u, ccr_v, ccr_w;

  ccr_u = pwmDutyToCCR(duty_u);
  ccr_v = pwmDutyToCCR(duty_v);
  ccr_w = pwmDutyToCCR(duty_w);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_u);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_v);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_w);
}

bool pwmIsBreakFault(void)
{
  return pwm_break_fault;
}

void pwmClearBreakFault(void)
{
  pwm_break_fault = false;

  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    pwm_break_fault = true;
  }
}




void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_baseHandle->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PA6     ------> TIM1_BKIN
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM1)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PA7     ------> TIM1_CH1N
    PB0     ------> TIM1_CH2N
    PB1     ------> TIM1_CH3N
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    PA10     ------> TIM1_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_DISABLE();

    /**TIM1 GPIO Configuration
    PA6     ------> TIM1_BKIN
    PA7     ------> TIM1_CH1N
    PB0     ------> TIM1_CH2N
    PB1     ------> TIM1_CH3N
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    PA10     ------> TIM1_CH3
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_1);

    HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
  }
}
