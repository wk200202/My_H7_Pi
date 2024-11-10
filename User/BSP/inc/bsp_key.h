#ifndef _BSP_KEY_H_
#define _BSP_KEY_H_

#include "bsp.h"

/***************************************************************************
*	                           宏定义
***************************************************************************/

#define JOY_DOWN_U KEY_1_DOWN /* 上 */
#define JOY_UP_U   KEY_1_UP
#define JOY_LONG_U KEY_1_LONG

#define JOY_DOWN_D KEY_2_DOWN /* 下 */
#define JOY_UP_D   KEY_2_UP
#define JOY_LONG_D KEY_2_LONG

#define JOY_DOWN_L KEY_3_DOWN /* 左 */
#define JOY_UP_L   KEY_3_UP
#define JOY_LONG_L KEY_3_LONG

#define JOY_DOWN_R KEY_4_DOWN /* 右 */
#define JOY_UP_R   KEY_4_UP
#define JOY_LONG_R KEY_4_LONG

#define JOY_DOWN_OK KEY_5_DOWN /* ok */
#define JOY_UP_OK   KEY_5_UP
#define JOY_LONG_OK KEY_5_LONG

/***************************************************************************
*	                          变量声明
***************************************************************************/

/* 按键ID, 主要用于bsp_KeyState()函数的入口参数 */
typedef enum {
    KID_JOY_U = 0,
    KID_JOY_D,
    KID_JOY_L,
    KID_JOY_R,
    KID_JOY_OK
} KEY_ID_E;

/*
	按键滤波时间50ms, 单位10ms。
	只有连续检测到50ms状态不变才认为有效，包括弹起和按下两种事件
	即使按键电路不做硬件滤波，该滤波机制也可以保证可靠地检测到按键事件
*/
#define KEY_FILTER_TIME 5
#define KEY_LONG_TIME   100 /* 单位10ms， 持续1秒，认为长按事件 */

/*
	每个按键对应1个全局的结构体变量。
*/
typedef struct
{
    uint8_t (*IsKeyDownFunc)(void); /* 按键按下的判断函数,1表示按下 */
    uint8_t Count;                  /* 滤波器计数器 */
    uint16_t LongCount;             /* 长按计数器 */
    uint16_t LongTime;              /* 按键按下持续时间, 0表示不检测长按 */
    uint8_t State;                  /* 按键当前状态（按下还是弹起） */
    uint8_t RepeatSpeed;            /* 连续按键周期 */
    uint8_t RepeatCount;            /* 连续按键计数器 */
} KEY_T;

/*
	定义键值代码, 必须按如下次序定时每个键的按下、弹起和长按事件

	推荐使用enum, 不用#define，原因：
	(1) 便于新增键值,方便调整顺序，使代码看起来舒服点
	(2) 编译器可帮我们避免键值重复。
*/
typedef enum {
    KEY_NONE = 0, /* 0 表示按键事件 */

    KEY_1_DOWN, /* 4键按下 */
    KEY_1_UP,   /* 4键弹起 */
    KEY_1_LONG, /* 4键长按 */

    KEY_2_DOWN, /* 5键按下 */
    KEY_2_UP,   /* 5键弹起 */
    KEY_2_LONG, /* 5键长按 */

    KEY_3_DOWN, /* 6键按下 */
    KEY_3_UP,   /* 6键弹起 */
    KEY_3_LONG, /* 6键长按 */

    KEY_4_DOWN, /* 7键按下 */
    KEY_4_UP,   /* 7键弹起 */
    KEY_4_LONG, /* 7键长按 */

    KEY_5_DOWN, /* 8键按下 */
    KEY_5_UP,   /* 8键弹起 */
    KEY_5_LONG, /* 8键长按 */
} KEY_ENUM;

/* 按键FIFO用到变量 */
#define KEY_FIFO_SIZE 10
typedef struct
{
    uint8_t Buf[KEY_FIFO_SIZE]; /* 键值缓冲区 */
    uint8_t Read;               /* 缓冲区读指针1 */
    uint8_t Write;              /* 缓冲区写指针 */
    uint8_t Read2;              /* 缓冲区读指针2 */
} KEY_FIFO_T;

/***************************************************************************
*	                          函数声明
***************************************************************************/

void bsp_Key_Init(void);
void bsp_KeyScan10ms(void);
void bsp_PutKey(uint8_t _KeyCode);
uint8_t bsp_GetKey(void);
uint8_t bsp_GetKey2(void);
uint8_t bsp_GetKeyState(KEY_ID_E _ucKeyID);
void bsp_SetKeyParam(uint8_t _ucKeyID, uint16_t _LongTime, uint8_t _RepeatSpeed);
void bsp_ClearKey(void);

#endif   // _BSP_KEY_H_
