#include "bsp_timer.h"

/*************************************************************************
*		                          变量声明	
**************************************************************************
*/
/* 这2个全局变量转用于 bsp_DelayMS() 函数 */
static volatile uint32_t s_uiDelayCount = 0;
static volatile uint8_t s_ucTimeOutFlag = 0;

/* 定于软件定时器结构体变量 */
static SOFT_TMR s_tTmr[TMR_COUNT] = {0};

/* 全局运行时间，单位1ms，最长可以表示 49.71天 */
__IO uint32_t g_iRunTime = 0;
/* 等待变量初始化 */
static __IO uint8_t g_ucEnableSystickISR = 0;

/*************************************************************************
*		                          函数声明	
**************************************************************************
*/

static void bsp_SoftTimerDec(SOFT_TMR *_tmr);

/*************************************************************************
*	@brief 		初始化bsp定时器
*	@param 		None
*	@retval		None
*	@note  		包含sysTick定时器和软件定时器的初始化
*	@author		HarryWang
*	@date  		2024/11/10
**************************************************************************
*/
void bsp_InitTimer(void)
{
    uint8_t i;

    /* 清零所有的软件定时器 */
    for (i = 0; i < TMR_COUNT; i++) {
        s_tTmr[i].Count = 0;
        s_tTmr[i].PreLoad = 0;
        s_tTmr[i].Flag = 0;
        s_tTmr[i].Mode = TMR_ONCE_MODE; /* 缺省是1次性工作模式 */
    }
    /* 配置systic中断周期为1ms，并启动systick中断。 */
    SysTick_Config(SystemCoreClock / 1000);
    /* 1表示执行systick中断 */
    g_ucEnableSystickISR = 1;
}

/*************************************************************************
*	@brief 		sysTick中断应用程序
*	@param 		None
*	@retval		None
*	@note  		此函数位于sysTick中断服务程序中，每隔1ms进入1次
*	@note  		若g_ucEnableSystickISR = 0，则不执行此函数
*	@author		HarryWang
*	@date  		2024/11/11
**************************************************************************
*/
extern void bsp_RunPer1ms(void);
extern void bsp_RunPer10ms(void);
void SysTick_ISR(void)
{
    static uint8_t s_count = 0;
    uint8_t i;

    /* 每隔1ms进来1次 （仅用于 bsp_DelayMS） */
    if (s_uiDelayCount > 0) {
        if (--s_uiDelayCount == 0) {
            s_ucTimeOutFlag = 1;
        }
    }

    /* 每隔1ms，对软件定时器的计数器进行减一操作 */
    for (i = 0; i < TMR_COUNT; i++) {
        bsp_SoftTimerDec(&s_tTmr[i]);
    }

    /* 全局运行时间每1ms增1 */
    g_iRunTime++;
    if (g_iRunTime == 0x7FFFFFFF) /* 这个变量是 int32_t 类型，最大数为 0x7FFFFFFF */
    {
        g_iRunTime = 0; /* 判断溢出，清零 */
    }

    bsp_RunPer1ms(); /* 每隔1ms调用一次此函数，此函数在 bsp.c */

    if (++s_count >= 10) {
        s_count = 0;

        bsp_RunPer10ms(); /* 每隔10ms调用一次此函数，此函数在 bsp.c */
    }
}

/*************************************************************************
*	@brief 		软件定时器计数值减1
*	@param 		tmr : 定时器变量指针
*	@retval		None
*	@note  		None
*	@author		HarryWang
*	@date  		2024/11/11
**************************************************************************
*/
static void bsp_SoftTimerDec(SOFT_TMR *_tmr)
{
    if (_tmr->Count > 0) {
        /* 如果定时器变量减到1则设置定时器到达标志 */
        if (--_tmr->Count == 0) {
            _tmr->Flag = 1;

            /* 如果是自动模式，则自动重装计数器 */
            if (_tmr->Mode == TMR_AUTO_MODE) {
                _tmr->Count = _tmr->PreLoad;
            }
        }
    }
}

/*************************************************************************
*	@brief 		毫秒级延迟
*	@param 		n : 延迟长度，单位1ms
*	@retval		None
*	@note  		最小值为2，n=1时，实际延迟2ms
*	@author		HarryWang
*	@date  		2024/11/11
**************************************************************************
*/
void bsp_DelayMS(uint32_t n)
{
    if (n == 0) {
        return;
    } else if (n == 1) {
        n = 2;
    }

    DISABLE_INT(); /* 关中断 */

    s_uiDelayCount = n;
    s_ucTimeOutFlag = 0;

    ENABLE_INT(); /* 开中断 */

    while (1) {
        /*
            等待延迟时间到
            注意：编译器认为 s_ucTimeOutFlag = 0，所以可能优化错误，因此 s_ucTimeOutFlag 变量必须申明为 volatile
        */
        if (s_ucTimeOutFlag == 1) {
            break;
        }
    }
}

/*************************************************************************
*	@brief 		微秒级延迟
*	@param 		n : 延迟长度，单位1us
*	@retval		None
*	@note  		注意此函数必须在systick启动后才能调用
*	@author		HarryWang
*	@date  		2024/11/11
**************************************************************************
*/
void bsp_DelayUS(uint32_t n)
{
    uint32_t ticks;
    uint32_t told;
    uint32_t tnow;
    uint32_t tcnt = 0;
    uint32_t reload;

    reload = SysTick->LOAD;
    ticks = n * (SystemCoreClock / 1000000); /* 需要的节拍数 */

    tcnt = 0;
    told = SysTick->VAL; /* 记录刚进入时的计数器值 */

    while (1) {
        tnow = SysTick->VAL;
        if (tnow != told) {
            /* SYSTICK是一个递减的计数器 */
            if (tnow < told) {
                tcnt += told - tnow;
            }
            /* 重新装载递减 */
            else {
                tcnt += reload - tnow + told;
            }
            told = tnow;

            /* 时间超过/等于要延迟的时间,则退出 */
            if (tcnt >= ticks) {
                break;
            }
        }
    }
}

/*************************************************************************
*	@brief 		启动软件定时器，定时时间到后，自动置标志，只运行1次
*	@param 		_id     : 定时器ID，值域【0,TMR_COUNT-1】
*	@param 		_period : 定时周期，单位1ms
*	@retval		None
*	@note  		注意用户需要自己维护定时器ID，以避免定时器ID冲突
*	@author		HarryWang
*	@date  		2024/11/11
**************************************************************************
*/
void bsp_StartTimer(uint8_t _id, uint32_t _period)
{
    if (_id >= TMR_COUNT) {
        /* 打印出错的源代码文件名、函数名称 */
        printf("Error: file %s, function %s()\r\n", __FILE__, __FUNCTION__);
        while (1)
            ; /* 参数异常，死机等待看门狗复位 */
    }

    DISABLE_INT(); /* 关中断 */

    s_tTmr[_id].Count = _period;      /* 实时计数器初值 */
    s_tTmr[_id].PreLoad = _period;    /* 计数器自动重装值，仅自动模式起作用 */
    s_tTmr[_id].Flag = 0;             /* 定时时间到标志 */
    s_tTmr[_id].Mode = TMR_ONCE_MODE; /* 1次性工作模式 */

    ENABLE_INT(); /* 开中断 */
}

/*************************************************************************
*	@brief 		启动自动定时器，定时时间到后，自动置标志，循环运行
*	@param 		_id     : 定时器ID，值域【0,TMR_COUNT-1】
*	@param 		_period : 定时周期，单位1ms
*	@retval		None
*	@note  		注意用户需要自己维护定时器ID，以避免定时器ID冲突
*	@author		HarryWang
*	@date  		2024/11/11
**************************************************************************
*/
void bsp_StartAutoTimer(uint8_t _id, uint32_t _period)
{
    if (_id >= TMR_COUNT) {
        /* 打印出错的源代码文件名、函数名称 */
        printf("Error: file %s, function %s()\r\n", __FILE__, __FUNCTION__);
        while (1)
            ; /* 参数异常，死机等待看门狗复位 */
    }

    DISABLE_INT(); /* 关中断 */

    s_tTmr[_id].Count = _period;      /* 实时计数器初值 */
    s_tTmr[_id].PreLoad = _period;    /* 计数器自动重装值，仅自动模式起作用 */
    s_tTmr[_id].Flag = 0;             /* 定时时间到标志 */
    s_tTmr[_id].Mode = TMR_AUTO_MODE; /* 自动工作模式 */

    ENABLE_INT(); /* 开中断 */
}

/*************************************************************************
*	@brief 		停止软件定时器计数
*	@param 		_id : 定时器ID，值域【0,TMR_COUNT-1】
*	@retval		None
*	@note  		用户需要自己维护定时器ID，以避免定时器ID冲突
*	@author		HarryWang
*	@date  		2024/11/11
**************************************************************************
*/
void bsp_StopTimer(uint8_t _id)
{
    if (_id >= TMR_COUNT) {
        /* 打印出错的源代码文件名、函数名称 */
        printf("Error: file %s, function %s()\r\n", __FILE__, __FUNCTION__);
        while (1)
            ; /* 参数异常，死机等待看门狗复位 */
    }

    DISABLE_INT(); /* 关中断 */

    s_tTmr[_id].Count = 0;            /* 实时计数器初值 */
    s_tTmr[_id].Flag = 0;             /* 定时时间到标志 */
    s_tTmr[_id].Mode = TMR_ONCE_MODE; /* 自动工作模式 */

    ENABLE_INT(); /* 开中断 */
}

/*************************************************************************
*	@brief 		检测定时器是否超时
*	@param 		_id : 定时器ID，值域【0,TMR_COUNT-1】
*	@retval		 0 表示定时未到， 1表示定时到
*	@note  		用户需要自己维护定时器ID，以避免定时器ID冲突
*	@author		HarryWang
*	@date  		2024/11/11
**************************************************************************
*/
uint8_t bsp_CheckTimer(uint8_t _id)
{
    if (_id >= TMR_COUNT) {
        return 0;
    }

    if (s_tTmr[_id].Flag == 1) {
        s_tTmr[_id].Flag = 0;
        return 1;
    } else {
        return 0;
    }
}

/*************************************************************************
*	@brief 		获取CPU运行时间
*	@param 		None
*	@retval		CPU运行时间，单位1ms
*	@note  		注意CPU最长运行时间为49.71天，超过这个时间需要考虑溢出问题
*	@author		HarryWang
*	@date  		2024/11/11
**************************************************************************
*/
int32_t bsp_GetRunTime(void)
{
    int32_t runtime;

    DISABLE_INT(); /* 关中断 */

    runtime = g_iRunTime; /* 这个变量在Systick中断中被改写，因此需要关中断进行保护 */

    ENABLE_INT(); /* 开中断 */

    return runtime;
}

/*************************************************************************
*	@brief 		计算当前运行时间和给定时刻之间的差值
*	@param 		_LastTime : 上个时刻
*	@retval		差值，单位1ms
*	@note  		None
*	@author		HarryWang
*	@date  		2024/11/11
**************************************************************************
*/
int32_t bsp_CheckRunTime(int32_t _LastTime)
{
    int32_t now_time;
    int32_t time_diff;

    DISABLE_INT(); /* 关中断 */

    now_time = g_iRunTime; /* 这个变量在Systick中断中被改写，因此需要关中断进行保护 */

    ENABLE_INT(); /* 开中断 */

    if (now_time >= _LastTime) {
        time_diff = now_time - _LastTime;
    } else {
        time_diff = 0x7FFFFFFF - _LastTime + now_time;
    }

    return time_diff;
}

/*************************************************************************
*	@brief 		sysTick中断服务程序
*	@param 		None
*	@retval		None
*	@note  		None
*	@author		HarryWang
*	@date  		2024/11/11
**************************************************************************
*/
void SysTick_Handler(void)
{
    HAL_IncTick(); /* ST HAL库的滴答定时中断服务程序 */

    if (g_ucEnableSystickISR == 0) {
        return;
    }

    SysTick_ISR(); /* bsp库的滴答定时中断服务程序 */
}
