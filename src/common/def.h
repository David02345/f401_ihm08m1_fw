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

#define PI 3.14159265358979323846

#define OPEN_DT                           0.0002f

#define CUR_DT                            0.0002f
#define VEL_DT                            0.001f
#define POS_DT                            0.01f

#define CUR_KP                            0.1f
#define CUR_KI                            10.0f
#define CUR_KD                            0.0f

#define VEL_KP                            0.05f
#define VEL_KI                            1.0f
#define VEL_KD                            0.0f

#define POS_KP                            2.0f
#define POS_KI                            0.0f
#define POS_KD                            0.1f

#define PWM_ARR                           2099
#define PWM_ADC_TRIG_PULSE                (PWM_ARR - 100U)
#define PWM_DEADTIME                      80
#define PWM_INIT_PULSE                    1049

#define OUTPUT_ID_MAX                     100.0f
#define OUTPUT_ID_MIN                     -100.0f
#define OUTPUT_IQ_MAX                     100.0f
#define OUTPUT_IQ_MIN                     -100.0f
#define OUTPUT_VEL_MAX                    100.0f
#define OUTPUT_VEL_MIN                    -100.0f
#define OUTPUT_POS_MAX                    100.0f
#define OUTPUT_POS_MIN                    -100.0f

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
