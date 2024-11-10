#ifndef _BSP_H_
#define _BSP_H_

#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* 开关全局中断的宏 */
#define ENABLE_INT()  __set_PRIMASK(0) /* 使能全局中断 */
#define DISABLE_INT() __set_PRIMASK(1) /* 禁止全局中断 */

#include "bsp_key.h"
#include "bsp_rgb.h"
#include "bsp_uart.h"
#include "bsp_iic.h"
#include "bsp_sht40.h"
#include "bsp_w25qxx.h"
#include "bsp_timer.h"

void bsp_Init(void);
void bsp_Idle(void);

void Error_Handler(char *file, uint32_t line);

#endif   // _BSP_H_
