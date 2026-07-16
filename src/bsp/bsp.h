/*
 * bsp.h
 *
 *  Created on: 2026. 5. 26.
 *      Author: YDG
 */

#ifndef SRC_BSP_BSP_H_
#define SRC_BSP_BSP_H_




#include "def.h"

#define _USE_LOG_PRINT    1

#if _USE_LOG_PRINT
#define logPrintf(fmt, ...)     printf(fmt, ##__VA_ARGS__)
#else
#define logPrintf(fmt, ...)
#endif


#include "stm32f4xx_hal.h"

/*
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
*/


void bspInit(void);

void delay(uint32_t ms);
uint32_t millis(void);
void Error_Handler(void);
bool microsInit(void);
uint32_t micros(void);
uint32_t cycleGet(void);
uint32_t cycleGetFreq(void);

#endif /* SRC_BSP_BSP_H_ */
