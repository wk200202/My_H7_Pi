#ifndef _BSP_TIMER_H_
#define _BSP_TIMER_H_

#include "bsp.h"

/***************************************************************************
*	                           宏定义
***************************************************************************/

#define TMR_COUNT 4 /* 软件定时器的个数 */

/***************************************************************************
*	                          变量声明
***************************************************************************/

/* 定时器结构体，成员变量必须是 volatile, 否则C编译器优化时可能有问题 */

/* 定时器工作模式 */
typedef enum {
    TMR_ONCE_MODE = 0, /* 一次工作模式 */
    TMR_AUTO_MODE = 1  /* 自动定时工作模式 */
} TMR_MODE_E;

/* 定时器配置结构体 */
typedef struct
{
    volatile uint8_t Mode;     /* 计数器模式，1次性 */
    volatile uint8_t Flag;     /* 定时到达标志  */
    volatile uint32_t Count;   /* 计数器 */
    volatile uint32_t PreLoad; /* 计数器预装值 */
} SOFT_TMR;

/***************************************************************************
*	                          函数声明
***************************************************************************/

void bsp_InitTimer(void);
void bsp_DelayMS(uint32_t n);
void bsp_DelayUS(uint32_t n);
void bsp_StartTimer(uint8_t _id, uint32_t _period);
void bsp_StartAutoTimer(uint8_t _id, uint32_t _period);
void bsp_StopTimer(uint8_t _id);
uint8_t bsp_CheckTimer(uint8_t _id);
int32_t bsp_GetRunTime(void);
int32_t bsp_CheckRunTime(int32_t _LastTime);

#endif   // _BSP_TIMER_H_
