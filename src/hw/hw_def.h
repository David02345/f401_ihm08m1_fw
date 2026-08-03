/*
 * hw.def.h
 *
 *  Created on: 2026. 5. 26.
 *      Author: YDG
 */

#ifndef SRC_HW_HW_DEF_H_
#define SRC_HW_HW_DEF_H_


#include "def.h"
#include "bsp.h"


#define MOTOR_CONTROL_OPEN_LOOP           0
#define MOTOR_CONTROL_CURRENT             1
#define MOTOR_CONTROL_SPEED               2
#define MOTOR_CONTROL_POSITION            3

//#define MOTOR_CONTROL_MODE                MOTOR_CONTROL_CURRENT
#define MOTOR_CONTROL_MODE                MOTOR_CONTROL_OPEN_LOOP

#define _USE_MOTOR_SIXSTEP                0
#define _USE_MOTOR_FOC                    1
#define _USE_FOC_SPWM                     1
#define _USE_FOC_SVPWM                    0

#if (_USE_FOC_SPWM + _USE_FOC_SVPWM) != 1
#error "Enable exactly one modulation method"
#endif


#define _USE_HALL_SENSOR                  1
#define _USE_ENCODER                      0
#define _USE_HALL_TEST_ONLY               1

#define _USE_HW_ADC
//#define _USE_HW_USB
#define _USE_HW_RTC
#define _USE_HW_RESET
//#define _USE_HW_CDC
#define _USE_HW_FLASH

#define _USE_HW_LED
#define      HW_LED_MAX_CH                1

#define _USE_HW_UART
#define      HW_UART_MAX_CH               2

#define _USE_HW_CLI
#define      HW_CLI_LINE_HIS_MAX          4
#define      HW_CLI_LINE_BUF_MAX          32

#define      HW_CLI_CMD_LIST_MAX          16
#define      HW_CLI_CMD_NAME_MAX          16



#endif /* SRC_HW_HW_DEF_H_ */
