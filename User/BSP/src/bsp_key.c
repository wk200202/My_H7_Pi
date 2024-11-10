#include "bsp_key.h"

/*************************************************************************
*		                          变量声明	
**************************************************************************
*/
#define HARD_KEY_NUM 5 /* 实体按键个数 */
// #define KEY_COMBO_CTRL /* 是否开启组合键 */

#ifdef KEY_COMBO_CTRL
#define KEY_COMBO_NUM 0                              /* 组合按键个数 */
#define KEY_COUNT     (HARD_KEY_NUM + KEY_COMBO_NUM) /* 8个独立建 + 2个组合按键 */
#else
#define KEY_COUNT HARD_KEY_NUM /* 8个独立按键 */
#endif

/* 使能GPIO时钟 */
#define ALL_KEY_GPIO_CLK_ENABLE()     \
    {                                 \
        __HAL_RCC_GPIOD_CLK_ENABLE(); \
        __HAL_RCC_GPIOB_CLK_ENABLE(); \
    }
;

/* 依次定义GPIO */
typedef struct
{
    GPIO_TypeDef *gpio;
    uint16_t pin;
    uint8_t ActiveLevel; /* 激活电平 */
} X_GPIO_T;

/* GPIO和PIN定义 */
static const X_GPIO_T s_gpio_list[HARD_KEY_NUM] = {
    {GPIOB, GPIO_PIN_15, 0}, /* JOY_U */
    {GPIOD, GPIO_PIN_11, 0}, /* JOY_D */
    {GPIOD, GPIO_PIN_13, 0}, /* JOY_L */
    {GPIOB, GPIO_PIN_14, 0}, /* JOY_R */
    {GPIOD, GPIO_PIN_12, 0}, /* JOY_OK */
};

static KEY_T s_tBtn[KEY_COUNT] = {0}; /* 按键结构体变量 */
static KEY_FIFO_T s_tKey;             /* 按键FIFO变量,结构体 */

/*************************************************************************
*		                          函数声明	
**************************************************************************
*/
static void bsp_InitKeyVar(void);
static void bsp_InitKeyHard(void);
static void bsp_DetectKey(uint8_t i);

#define KEY_PIN_ACTIVE(id)

/*************************************************************************
 *	@brief  	KeyPinActive
 *	@param  	_id:	按键ID
 *	@retval 	1:		按键按下	0:		按键未按下
 *	@note   	底层用于判断GPIO引脚是否有效按下
 *	@author 	HarryWang
 *	@date   	2024/07/31
 **************************************************************************
 */
static uint8_t KeyPinActive(uint8_t _id)
{
    uint8_t level;

    if ((s_gpio_list[_id].gpio->IDR & s_gpio_list[_id].pin) == 0) {
        level = 0;
    } else {
        level = 1;
    }

    if (level == s_gpio_list[_id].ActiveLevel) {
        return 1;
    } else {
        return 0;
    }
}

/*************************************************************************
 *	@brief  	IsKeyDownFunc
 *	@param  	_id:	按键ID
 *	@retval 	1:		按键按下	0:		按键未按下
 *	@note   	判断按键是否按下。单键和组合键区分。单键事件不允许有其他键按下。
 *	@author 	HarryWang
 *	@date   	2024/07/31
 **************************************************************************
 */
static uint8_t IsKeyDownFunc(uint8_t _id)
{
    /* 实体单键 */
    if (_id < HARD_KEY_NUM) {
        uint8_t i;
        uint8_t count = 0;
        uint8_t save = 255;

        /* 判断有几个键按下 */
        for (i = 0; i < HARD_KEY_NUM; i++) {
            if (KeyPinActive(i)) {
                count++;
                save = i;
            }
        }

        if (count == 1 && save == _id) /* 只有1个键按下时才有效 */
        {
            return 1;
        }

        return 0;
    }

#ifdef KEY_COMBO_CTRL
    if (_id == HARD_KEY_NUM + 0) /* K1 + K2 */
    {
        if (KeyPinActive(KID_K1) && KeyPinActive(KID_K2)) {
            return 1;
        } else {
            return 0;
        }
    } else if (_id == HARD_KEY_NUM + 1) /* K2 + K3 */
    {
        if (KeyPinActive(KID_K2) && KeyPinActive(KID_K3)) {
            return 1;
        } else {
            return 0;
        }
    }
#endif

    return 0;
}

/*************************************************************************
 *	@brief  	bsp_InitKey
 *	@param  	None
 *	@retval 	None
 *	@note   	初始化按键. 该函数被 bsp_Init() 调用。
 *	@author 	HarryWang
 *	@date   	2024/07/31
 **************************************************************************
 */
void bsp_Key_Init(void)
{
    bsp_InitKeyVar();  /* 初始化按键变量 */
    bsp_InitKeyHard(); /* 初始化按键硬件 */
}

/*************************************************************************
 *	@brief  	bsp_InitKeyHard
 *	@param  	None
 *	@retval 	None
 *	@note   	配置按键对应的GPIO
 *	@author 	HarryWang
 *	@date   	2024/07/31
 **************************************************************************
 */
static void bsp_InitKeyHard(void)
{
    GPIO_InitTypeDef gpio_init;
    uint8_t i;

    /* 第1步：打开GPIO时钟 */
    ALL_KEY_GPIO_CLK_ENABLE();

    /* 第2步：配置所有的按键GPIO为浮动输入模式(实际上CPU复位后就是输入状态) */
    gpio_init.Mode = GPIO_MODE_INPUT;            /* 设置输入 */
    gpio_init.Pull = GPIO_NOPULL;                /* 上下拉电阻不使能 */
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* GPIO速度等级 */

    for (i = 0; i < HARD_KEY_NUM; i++) {
        gpio_init.Pin = s_gpio_list[i].pin;
        HAL_GPIO_Init(s_gpio_list[i].gpio, &gpio_init);
    }
}

/*************************************************************************
 *	@brief  	bsp_InitKeyVar
 *	@param  	None
 *	@retval 	None
 *	@note   	初始化按键变量结构体
 *	@author 	HarryWang
 *	@date   	2024/07/31
 **************************************************************************
 */
static void bsp_InitKeyVar(void)
{
    uint8_t i;

    /* 对按键FIFO读写指针清零 */
    s_tKey.Write = 0;
    s_tKey.Read = 0;
    s_tKey.Read2 = 0;

    /* 给每个按键结构体成员变量赋一组缺省值 */
    for (i = 0; i < KEY_COUNT; i++) {
        s_tBtn[i].LongTime = KEY_LONG_TIME;    /* 按键长按判定时间阈值，长按时间 0 表示不检测长按键事件 */
        s_tBtn[i].Count = KEY_FILTER_TIME / 2; /* 计数器设置为滤波时间的一半 */
        s_tBtn[i].State = 0;                   /* 按键默认状态，0为未按下 */
        s_tBtn[i].RepeatSpeed = 0;             /* 按键连发的速度，0表示不支持连发 */
        s_tBtn[i].RepeatCount = 0;             /* 连发计数器 */
    }

    /* 如果需要单独更改某个按键的参数，可以在此单独重新赋值 */

    /* 摇杆上下左右，支持长按1秒后，自动连发 */
    bsp_SetKeyParam(KID_JOY_U, 100, 6);
    bsp_SetKeyParam(KID_JOY_D, 100, 6);
    bsp_SetKeyParam(KID_JOY_L, 100, 6);
    bsp_SetKeyParam(KID_JOY_R, 100, 6);
}

/*************************************************************************
 *	@brief  	bsp_PutKey
 *	@param  	_KeyCode:	按键代码
 *	@retval 	None
 *	@note   	将1个键值压入按键FIFO缓冲区
 *	@author 	HarryWang
 *	@date   	2024/07/31
 **************************************************************************
 */
void bsp_PutKey(uint8_t _KeyCode)
{
    s_tKey.Buf[s_tKey.Write] = _KeyCode;

    if (++s_tKey.Write >= KEY_FIFO_SIZE) /* 环形缓冲思想 */
    {
        s_tKey.Write = 0;
    }
}

/*************************************************************************
 *	@brief  	bsp_GetKey
 *	@param  	None
 *	@retval 	按键代码
 *	@note   	从按键FIFO缓冲区读取一个键值
 *	@author 	HarryWang
 *	@date   	2024/07/31
 **************************************************************************
 */
uint8_t bsp_GetKey(void)
{
    uint8_t ret;

    if (s_tKey.Read == s_tKey.Write) /* 读和写指针相同，表示没有按键 */
    {
        return KEY_NONE;
    } else {
        ret = s_tKey.Buf[s_tKey.Read];

        if (++s_tKey.Read >= KEY_FIFO_SIZE) {
            s_tKey.Read = 0;
        }
        return ret;
    }
}

/*************************************************************************
 *	@brief  	bsp_GetKey2
 *	@param  	None
 *	@retval 	按键代码
 *	@note   	从按键FIFO缓冲区读取一个键值。独立的读指针
 *	@author 	HarryWang
 *	@date   	2024/07/31
 **************************************************************************
 */
uint8_t bsp_GetKey2(void)
{
    uint8_t ret;

    if (s_tKey.Read2 == s_tKey.Write) {
        return KEY_NONE;
    } else {
        ret = s_tKey.Buf[s_tKey.Read2];

        if (++s_tKey.Read2 >= KEY_FIFO_SIZE) {
            s_tKey.Read2 = 0;
        }
        return ret;
    }
}

/*************************************************************************
 *	@brief  	bsp_GetKeyState
 *	@param  	_ucKeyID:	按键ID
 *	@retval 	1:		按键按下	0:		按键未按下
 *	@note   	按键ID，从0开始
 *	@author 	HarryWang
 *	@date   	2024/07/31
 **************************************************************************
 */
uint8_t bsp_GetKeyState(KEY_ID_E _ucKeyID)
{
    return s_tBtn[_ucKeyID].State;
}

/*************************************************************************
 *	@brief  	bsp_SetKeyParam
 *	@param  	_ucKeyID:	按键ID 
 *	@param  	_LongTime:	长按事件时间
 *	@param  	_RepeatSpeed:	连发速度
 *	@retval 	None
 *	@note   	按键ID，从0开始
 *	@author 	HarryWang
 *	@date   	2024/07/31
 **************************************************************************
 */
void bsp_SetKeyParam(uint8_t _ucKeyID, uint16_t _LongTime, uint8_t _RepeatSpeed)
{
    s_tBtn[_ucKeyID].LongTime = _LongTime;       /* 长按时间 0 表示不检测长按键事件 */
    s_tBtn[_ucKeyID].RepeatSpeed = _RepeatSpeed; /* 按键连发的速度，0表示不支持连发 */
    s_tBtn[_ucKeyID].RepeatCount = 0;            /* 连发计数器 */
}

/*************************************************************************
 *	@brief  	bsp_ClearKey
 *	@param  	None
 *	@retval 	None
 *	@note   	清空按键FIFO缓冲区
 *	@author 	HarryWang
 *	@date   	2024/07/31
 **************************************************************************
 */
void bsp_ClearKey(void)
{
    s_tKey.Read = s_tKey.Write;
}

/*************************************************************************
 *	@brief  	bsp_DetectKey
 *	@param  	_i:		按键ID
 *	@retval 	None
 *	@note   	检测一个按键。非阻塞状态，必须被周期性(10ms)的调用
 *	@author 	HarryWang
 *	@date   	2024/07/31
 **************************************************************************
 */
static void bsp_DetectKey(uint8_t i)
{
    KEY_T *pBtn;

    pBtn = &s_tBtn[i];    /* 读取相应按键的结构体地址 */
    if (IsKeyDownFunc(i)) /* 按键按下时处理 */
    {
        if (pBtn->Count < KEY_FILTER_TIME) /* 按键消抖滤波器 */
        {
            pBtn->Count = KEY_FILTER_TIME;
        } else if (pBtn->Count < 2 * KEY_FILTER_TIME) {
            pBtn->Count++;
        } else {
            if (pBtn->State == 0) {
                pBtn->State = 1;

                /* 发送按钮按下的消息 */
                bsp_PutKey((uint8_t)(3 * i + 1));
            }

            if (pBtn->LongTime > 0) {
                if (pBtn->LongCount < pBtn->LongTime) {
                    /* 发送按钮持续按下的消息 */
                    if (++pBtn->LongCount == pBtn->LongTime) {
                        /* 键值放入按键FIFO */
                        bsp_PutKey((uint8_t)(3 * i + 3));
                    }
                } else {
                    if (pBtn->RepeatSpeed > 0) {
                        if (++pBtn->RepeatCount >= pBtn->RepeatSpeed) {
                            pBtn->RepeatCount = 0;
                            /* 常按键后，每隔10ms发送1个按键 */
                            bsp_PutKey((uint8_t)(3 * i + 1));
                        }
                    }
                }
            }
        }
    } else /* 按键松手的处理或者按键没有按下的处理 */
    {
        if (pBtn->Count > KEY_FILTER_TIME) {
            pBtn->Count = KEY_FILTER_TIME;
        } else if (pBtn->Count != 0) {
            pBtn->Count--;
        } else {
            if (pBtn->State == 1) {
                pBtn->State = 0;

                /* 发送按钮弹起的消息 */
                bsp_PutKey((uint8_t)(3 * i + 2));
            }
        }

        pBtn->LongCount = 0;
        pBtn->RepeatCount = 0;
    }
}

/*************************************************************************
 *	@brief  	bsp_DetectFastIO
 *	@param  	_i:		按键ID
 *	@retval 	None
 *	@note   	检测高速的输入IO. 周期性(1ms)的调用
 *	@author 	HarryWang
 *	@date   	2024/07/31
 **************************************************************************
 */
static void bsp_DetectFastIO(uint8_t i)
{
    KEY_T *pBtn;

    pBtn = &s_tBtn[i];
    if (IsKeyDownFunc(i)) {
        if (pBtn->State == 0) {
            pBtn->State = 1;

            /* 发送按钮按下的消息 */
            bsp_PutKey((uint8_t)(3 * i + 1));
        }

        if (pBtn->LongTime > 0) {
            if (pBtn->LongCount < pBtn->LongTime) {
                /* 发送按钮持续按下的消息 */
                if (++pBtn->LongCount == pBtn->LongTime) {
                    /* 键值放入按键FIFO */
                    bsp_PutKey((uint8_t)(3 * i + 3));
                }
            } else {
                if (pBtn->RepeatSpeed > 0) {
                    if (++pBtn->RepeatCount >= pBtn->RepeatSpeed) {
                        pBtn->RepeatCount = 0;
                        /* 常按键后，每隔10ms发送1个按键 */
                        bsp_PutKey((uint8_t)(3 * i + 1));
                    }
                }
            }
        }
    } else {
        if (pBtn->State == 1) {
            pBtn->State = 0;

            /* 发送按钮弹起的消息 */
            bsp_PutKey((uint8_t)(3 * i + 2));
        }

        pBtn->LongCount = 0;
        pBtn->RepeatCount = 0;
    }
}

/*************************************************************************
 *	@brief  	bsp_KeyScan10ms
 *	@param  	None
 *	@retval 	None
 *	@note   	扫描所有按键。非阻塞，被systick中断周期性的调用，10ms一次
 *	@author 	HarryWang
 *	@date   	2024/07/31
 **************************************************************************
 */
void bsp_KeyScan10ms(void)
{
    uint8_t i;

    for (i = 0; i < KEY_COUNT; i++) {
        bsp_DetectKey(i);
    }
}

/*************************************************************************
 *	@brief  	bsp_KeyScan1ms
 *	@param  	None
 *	@retval 	None
 *	@note   	扫描所有按键。非阻塞，被systick中断周期性的调用，1ms一次.
 *	@author 	HarryWang
 *	@date   	2024/07/31
 **************************************************************************
 */
void bsp_KeyScan1ms(void)
{
    uint8_t i;

    for (i = 0; i < KEY_COUNT; i++) {
        bsp_DetectFastIO(i);
    }
}
