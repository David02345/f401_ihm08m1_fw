/*
 * adc.c
 *
 *  Created on: 2026. 7. 2.
 *      Author: YDG
 */

#include "adc.h"


ADC_HandleTypeDef hadc1;

static ADC_ChannelConfTypeDef sConfig = {0};
static ADC_InjectionConfTypeDef sConfigInjected = {0};

static volatile motor_abc_u16_t adc_curr_raw = {0};
static motor_abc_f_t adc_curr_offset = {0.0f, 0.0f, 0.0f};
static volatile uint32_t adc_curr_update_count = 0;

static volatile uint16_t adc_vbus_raw  = 0;
static volatile uint16_t adc_speed_raw = 0;
static volatile uint16_t adc_temp_raw  = 0;


bool adcInit(void)
{
  bool ret = true;

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }

  sConfig.Channel = ADC_CH_VBUS;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_VBUS_ST;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }
  sConfig.Channel = ADC_CH_SPEED;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SPEED_ST;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }
  sConfig.Channel = ADC_CH_TEMP;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_TEMP_ST;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }


  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_CURR_ST;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;

  sConfigInjected.InjectedChannel = ADC_CURR_CH1;
  sConfigInjected.InjectedRank = 1;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }
  sConfigInjected.InjectedChannel = ADC_CURR_CH2;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }
  sConfigInjected.InjectedChannel = ADC_CURR_CH3;
  sConfigInjected.InjectedRank = 3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
    ret = false;
  }

  return ret;
}

void adcIJTStart(void)
{
  HAL_ADCEx_InjectedStart_IT(&hadc1);
}

void adcIJTStop(void)
{
  HAL_ADCEx_InjectedStop_IT(&hadc1);
}
void adcRGRStart(void)
{
  HAL_ADC_Start(&hadc1);
}

void adcRGRStop(void)
{
  HAL_ADC_Stop(&hadc1);
}


void adcCalibrateCurrentOffset(void)
{
  uint32_t sum_a = 0;
  uint32_t sum_b = 0;
  uint32_t sum_c = 0;
  uint32_t prev_count;
  uint32_t timeout;
  motor_abc_u16_t raw;

  const uint32_t sample_count = 1000;

  prev_count = adc_curr_update_count;

  for (uint32_t i = 0; i < sample_count; i++)
  {
    timeout = HAL_GetTick();

    while (adc_curr_update_count == prev_count)
    {
      if ((HAL_GetTick() - timeout) > 100)
      {
        return;
      }
    }

    prev_count = adc_curr_update_count;

    raw.a = adc_curr_raw.a;
    raw.b = adc_curr_raw.b;
    raw.c = adc_curr_raw.c;

    sum_a += raw.a;
    sum_b += raw.b;
    sum_c += raw.c;
  }

  adc_curr_offset.a = (float)sum_a / (float)sample_count;
  adc_curr_offset.b = (float)sum_b / (float)sample_count;
  adc_curr_offset.c = (float)sum_c / (float)sample_count;
}

void adcGetCurrentRaw(motor_abc_u16_t *raw)
{
  if (raw == NULL)
  {
    return;
  }

  raw->a = adc_curr_raw.a;
  raw->b = adc_curr_raw.b;
  raw->c = adc_curr_raw.c;
}

void adcGetPhaseCurrent(motor_abc_f_t *curr)
{
  if (curr == NULL)
  {
    return;
  }

  curr->a = ((float)adc_curr_raw.a - adc_curr_offset.a) * ADC_CURRENT_SCALE;
  curr->b = ((float)adc_curr_raw.b - adc_curr_offset.b) * ADC_CURRENT_SCALE;
  curr->c = ((float)adc_curr_raw.c - adc_curr_offset.c) * ADC_CURRENT_SCALE;
}

bool adcUpdateRegular(void)
{
  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return false;
  }

  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
  {
    HAL_ADC_Stop(&hadc1);
    return false;
  }
  adc_vbus_raw = HAL_ADC_GetValue(&hadc1);

  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
  {
    HAL_ADC_Stop(&hadc1);
    return false;
  }
  adc_speed_raw = HAL_ADC_GetValue(&hadc1);

  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
  {
    HAL_ADC_Stop(&hadc1);
    return false;
  }
  adc_temp_raw = HAL_ADC_GetValue(&hadc1);

  HAL_ADC_Stop(&hadc1);

  return true;
}

uint16_t adcGetVbusRaw(void)
{
  return adc_vbus_raw;
}

uint16_t adcGetSpeedRaw(void)
{
  return adc_speed_raw;
}

uint16_t adcGetTempRaw(void)
{
  return adc_temp_raw;
}

float adcGetVbusVoltage(void)
{
  return (float)adc_vbus_raw * ADC_VBUS_SCALE;
}




void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc_curr_raw.a = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    adc_curr_raw.b = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    adc_curr_raw.c = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

    adc_curr_update_count++;
  }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN10
    PC1     ------> ADC1_IN11
    PC2     ------> ADC1_IN12
    PC3     ------> ADC1_IN13
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA4     ------> ADC1_IN4
    PC4     ------> ADC1_IN14
    PC5     ------> ADC1_IN15
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PC0     ------> ADC1_IN10
    PC1     ------> ADC1_IN11
    PC2     ------> ADC1_IN12
    PC3     ------> ADC1_IN13
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA4     ------> ADC1_IN4
    PC4     ------> ADC1_IN14
    PC5     ------> ADC1_IN15
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4);

    /* ADC1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}
