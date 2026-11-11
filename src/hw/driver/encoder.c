/*
 * encoder.c
 *
 *  Created on: 2026. 7. 28.
 *      Author: YDG
 */

#include "encoder.h"
#include "util.h"

static volatile int32_t curr_count = 0;
static volatile int32_t prev_count = 0;
static volatile float speed_e = 0.0f;
static volatile float speed_m = 0.0f;
static volatile float angle_e = 0.0f;
static volatile float angle_m = 0.0f;
static volatile float angle_g = 0.0f;
static volatile float encoder_offset = 0.0f;
static volatile bool encoder_offset_valid = false;

TIM_HandleTypeDef htim2;


bool encoderInit(void)
{
  bool ret = true;
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 0;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 3999;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode          = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity          = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection         = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler         = TIM_ICPSC_DIV1;
  /* Reject narrow glitches on the single-ended quadrature A/B lines.
   * Filter code 8 samples at fDTS/8, 6 equal samples required.
   * This does not debounce mechanical oscillation. */
  sConfig.IC1Filter            = 8;
  sConfig.IC2Polarity          = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection         = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler         = TIM_ICPSC_DIV1;
  sConfig.IC2Filter            = 8;

  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }

  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
  {
    ret = false;
  }

  curr_count = (int32_t)encoderGetCount();
  prev_count = curr_count;

  speed_m = 0.0f;
  speed_e = 0.0f;

  return ret;
}

uint32_t encoderGetCount(void)
{
  return __HAL_TIM_GET_COUNTER(&htim2);
}

void encoderUpdateSpeed(float dt)
{
  if (dt <= 0.0f)
  {
    return;
  }
  int32_t delta;

  curr_count = (int32_t)encoderGetCount();

  delta = curr_count - prev_count;

  if(delta > 2000)
  {
    delta -= 4000;
  }
  else if(delta < -2000)
  {
    delta += 4000;
  }

  prev_count = curr_count;

  speed_m = ((float)delta * COUNT_TO_ANGLE) / dt;
  speed_e = ENCODER_ELEC_SIGN * speed_m * MOTOR_POLE_PAIRS;
}

float encoderGetElectricalAngle(void)
{
  float angle_e_raw = wrapFloat(ENCODER_ELEC_SIGN * encoderGetCount() * \
                      COUNT_TO_ANGLE * MOTOR_POLE_PAIRS, 0.0f, 2*PI);

  angle_e = wrapFloat(angle_e_raw + encoder_offset, 0.0f, 2*PI);

  return angle_e;
}

float encoderGetMechanicalAngle(void)
{
  angle_m = wrapFloat(encoderGetCount() * COUNT_TO_ANGLE, 0, 2*PI);

  return angle_m;
}

float encoderGetGearedAngle(void)
{
  angle_g = wrapFloat(encoderGetMechanicalAngle(), 0, 2*PI);

  return angle_g;
}

float encoderGetElectricalSpeed(void)
{
  return speed_e;
}

float encoderGetMechanicalSpeed(void)
{
  return speed_m;
}

void encoderClearCalibration(void)
{
  encoder_offset = 0.0f;
  encoder_offset_valid = false;
}

bool encoderIsCalibrated(void)
{
  return encoder_offset_valid;
}

void encoderResetSpeedTracking(void)
{
  int32_t count = (int32_t)encoderGetCount();

  curr_count = count;
  prev_count = count;

  speed_m = 0.0f;
  speed_e = 0.0f;
}

void encoderSetElectricalOffset(float offset)
{
  if (!isfinite(offset))
  {
    return;
  }

  encoder_offset = wrapFloat(offset, 0.0f, 2.0f * PI);

  encoder_offset_valid = true;
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* tim_encoderHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_encoderHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
  /* TIM2 clock enable */
  __HAL_RCC_TIM2_CLK_ENABLE();

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**TIM2 GPIO Configuration
  PA15     ------> TIM2_CH1
  PB3     ------> TIM2_CH2
  */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
}
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* tim_encoderHandle)
{

  if(tim_encoderHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PA15     ------> TIM2_CH1
    PB3     ------> TIM2_CH2
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);

  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
}
