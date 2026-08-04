/*
 * adc.h
 *
 *  Created on: 2026. 7. 2.
 *      Author: YDG
 */

#ifndef SRC_COMMON_HW_INCLUDE_ADC_H_
#define SRC_COMMON_HW_INCLUDE_ADC_H_

#include "hw_def.h"
#include "motor_types.h"

#ifdef _USE_HW_ADC

#define ADC_CURR_CH1          ADC_CHANNEL_0            /*PHASE A CURRENT*/
#define ADC_CURR_CH2          ADC_CHANNEL_11           /*PHASE B CURRENT*/
#define ADC_CURR_CH3          ADC_CHANNEL_10           /*PHASE C CURRENT*/

#define ADC_CH_VBUS           ADC_CHANNEL_1            /*VBUS*/
#define ADC_CH_SPEED          ADC_CHANNEL_4            /*SPEED*/
#define ADC_CH_TEMP           ADC_CHANNEL_12           /*TEMP*/

//#define ADC_BEMF_CH1        ADC_CHANNEL_13           /*BEMF1*/
//#define ADC_BEMF_CH2        ADC_CHANNEL_14           /*BEMF2*/
//#define ADC_BEMF_CH3        ADC_CHANNEL_15           /*BEMF3*/

#define ADC_CURR_ST           ADC_SAMPLETIME_28CYCLES  /*CURRENT sampling time */
#define ADC_VBUS_ST           ADC_SAMPLETIME_84CYCLES  /*VBUS sampling time*/
#define ADC_SPEED_ST          ADC_SAMPLETIME_84CYCLES  /*SPEED sampling time*/
#define ADC_TEMP_ST           ADC_SAMPLETIME_84CYCLES  /*TEMP sampling time*/
//#define ADC_BEMF_ST         ADC_SAMPLETIME_28CYCLES  /*BEMF sampling time*/

#define ADC_VREF              3.3f
#define ADC_MAX_COUNT         4095.0f

#define ADC_VBUS_SCALE        0.015434f
#define ADC_SPEED_SCALE       (ADC_VREF / ADC_MAX_COUNT)
#define ADC_CURRENT_SCALE     0.01555f



typedef void (*adc_injected_callback_t)(void); //             motor.c에서 사용
void adcSetInjectedCallback(adc_injected_callback_t callback);


bool adcInit(void);
void adcInjectedStart(void);
void adcInjectedStop(void);
void adcRegularStart(void);
void adcRegularStop(void);

bool adcCalibrateCurrentOffset(void);

void adcGetCurrentRaw(motor_abc_u16_t *raw);
void adcGetPhaseCurrent(motor_abc_f_t *curr);

bool adcUpdateRegular(void);
uint16_t adcGetVbusRaw(void);
uint16_t adcGetSpeedRaw(void);
uint16_t adcGetTempRaw(void);
float adcGetVbusVoltage(void);
uint32_t adcGetCurrentUpdateCount(void);
void adcGetCurrentOffset(motor_abc_f_t *offset);

#endif
#endif /* SRC_COMMON_HW_INCLUDE_ADC_H_ */
