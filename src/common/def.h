/*
 * def.h
 *
 *  Created on: 2026. 5. 26.
 *      Author: YDG
 */

#ifndef SRC_COMMON_DEF_H_
#define SRC_COMMON_DEF_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>


#define PWM_ARR                           2099
#define PWM_ADC_TRIG_PULSE                (PWM_ARR - 100U)
#define PWM_DEADTIME                      80
#define PWM_INIT_PULSE                    1049


#define VBUS                              24
#define VLIMIT                            13.86

#define GEAR_RATIO                        25
#define GEAR_EFFICIENCY                   0.94
#define POLES                             8

#define _DEF_LED1                         0
#define _DEF_LED2                         1
#define _DEF_LED3                         2
#define _DEF_LED4                         3

#define _DEF_UART1                        0
#define _DEF_UART2                        1
#define _DEF_UART3                        2
#define _DEF_UART4                        3





#endif /* SRC_COMMON_DEF_H_ */
