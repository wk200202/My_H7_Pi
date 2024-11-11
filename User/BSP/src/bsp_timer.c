#include "bsp_timer.h"

/*************************************************************************
*		                          ��������	
**************************************************************************
*/
/* ��2��ȫ�ֱ���ת���� bsp_DelayMS() ���� */
static volatile uint32_t s_uiDelayCount = 0;
static volatile uint8_t s_ucTimeOutFlag = 0;

/* ���������ʱ���ṹ����� */
static SOFT_TMR s_tTmr[TMR_COUNT] = {0};

/* ȫ������ʱ�䣬��λ1ms������Ա�ʾ 49.71�� */
__IO uint32_t g_iRunTime = 0;
/* �ȴ�������ʼ�� */
static __IO uint8_t g_ucEnableSystickISR = 0;

/*************************************************************************
*		                          ��������	
**************************************************************************
*/

static void bsp_SoftTimerDec(SOFT_TMR *_tmr);

/*************************************************************************
*	@brief 		��ʼ��bsp��ʱ��
*	@param 		None
*	@retval		None
*	@note  		����sysTick��ʱ���������ʱ���ĳ�ʼ��
*	@author		HarryWang
*	@date  		2024/11/10
**************************************************************************
*/
void bsp_InitTimer(void)
{
    uint8_t i;

    /* �������е������ʱ�� */
    for (i = 0; i < TMR_COUNT; i++) {
        s_tTmr[i].Count = 0;
        s_tTmr[i].PreLoad = 0;
        s_tTmr[i].Flag = 0;
        s_tTmr[i].Mode = TMR_ONCE_MODE; /* ȱʡ��1���Թ���ģʽ */
    }
    /* ����systic�ж�����Ϊ1ms��������systick�жϡ� */
    SysTick_Config(SystemCoreClock / 1000);
    /* 1��ʾִ��systick�ж� */
    g_ucEnableSystickISR = 1;
}

/*************************************************************************
*	@brief 		sysTick�ж�Ӧ�ó���
*	@param 		None
*	@retval		None
*	@note  		�˺���λ��sysTick�жϷ�������У�ÿ��1ms����1��
*	@note  		��g_ucEnableSystickISR = 0����ִ�д˺���
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

    /* ÿ��1ms����1�� �������� bsp_DelayMS�� */
    if (s_uiDelayCount > 0) {
        if (--s_uiDelayCount == 0) {
            s_ucTimeOutFlag = 1;
        }
    }

    /* ÿ��1ms���������ʱ���ļ��������м�һ���� */
    for (i = 0; i < TMR_COUNT; i++) {
        bsp_SoftTimerDec(&s_tTmr[i]);
    }

    /* ȫ������ʱ��ÿ1ms��1 */
    g_iRunTime++;
    if (g_iRunTime == 0x7FFFFFFF) /* ��������� int32_t ���ͣ������Ϊ 0x7FFFFFFF */
    {
        g_iRunTime = 0; /* �ж���������� */
    }

    bsp_RunPer1ms(); /* ÿ��1ms����һ�δ˺������˺����� bsp.c */

    if (++s_count >= 10) {
        s_count = 0;

        bsp_RunPer10ms(); /* ÿ��10ms����һ�δ˺������˺����� bsp.c */
    }
}

/*************************************************************************
*	@brief 		�����ʱ������ֵ��1
*	@param 		tmr : ��ʱ������ָ��
*	@retval		None
*	@note  		None
*	@author		HarryWang
*	@date  		2024/11/11
**************************************************************************
*/
static void bsp_SoftTimerDec(SOFT_TMR *_tmr)
{
    if (_tmr->Count > 0) {
        /* �����ʱ����������1�����ö�ʱ�������־ */
        if (--_tmr->Count == 0) {
            _tmr->Flag = 1;

            /* ������Զ�ģʽ�����Զ���װ������ */
            if (_tmr->Mode == TMR_AUTO_MODE) {
                _tmr->Count = _tmr->PreLoad;
            }
        }
    }
}

/*************************************************************************
*	@brief 		���뼶�ӳ�
*	@param 		n : �ӳٳ��ȣ���λ1ms
*	@retval		None
*	@note  		��СֵΪ2��n=1ʱ��ʵ���ӳ�2ms
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

    DISABLE_INT(); /* ���ж� */

    s_uiDelayCount = n;
    s_ucTimeOutFlag = 0;

    ENABLE_INT(); /* ���ж� */

    while (1) {
        /*
            �ȴ��ӳ�ʱ�䵽
            ע�⣺��������Ϊ s_ucTimeOutFlag = 0�����Կ����Ż�������� s_ucTimeOutFlag ������������Ϊ volatile
        */
        if (s_ucTimeOutFlag == 1) {
            break;
        }
    }
}

/*************************************************************************
*	@brief 		΢�뼶�ӳ�
*	@param 		n : �ӳٳ��ȣ���λ1us
*	@retval		None
*	@note  		ע��˺���������systick��������ܵ���
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
    ticks = n * (SystemCoreClock / 1000000); /* ��Ҫ�Ľ����� */

    tcnt = 0;
    told = SysTick->VAL; /* ��¼�ս���ʱ�ļ�����ֵ */

    while (1) {
        tnow = SysTick->VAL;
        if (tnow != told) {
            /* SYSTICK��һ���ݼ��ļ����� */
            if (tnow < told) {
                tcnt += told - tnow;
            }
            /* ����װ�صݼ� */
            else {
                tcnt += reload - tnow + told;
            }
            told = tnow;

            /* ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳� */
            if (tcnt >= ticks) {
                break;
            }
        }
    }
}

/*************************************************************************
*	@brief 		���������ʱ������ʱʱ�䵽���Զ��ñ�־��ֻ����1��
*	@param 		_id     : ��ʱ��ID��ֵ��0,TMR_COUNT-1��
*	@param 		_period : ��ʱ���ڣ���λ1ms
*	@retval		None
*	@note  		ע���û���Ҫ�Լ�ά����ʱ��ID���Ա��ⶨʱ��ID��ͻ
*	@author		HarryWang
*	@date  		2024/11/11
**************************************************************************
*/
void bsp_StartTimer(uint8_t _id, uint32_t _period)
{
    if (_id >= TMR_COUNT) {
        /* ��ӡ�����Դ�����ļ������������� */
        printf("Error: file %s, function %s()\r\n", __FILE__, __FUNCTION__);
        while (1)
            ; /* �����쳣�������ȴ����Ź���λ */
    }

    DISABLE_INT(); /* ���ж� */

    s_tTmr[_id].Count = _period;      /* ʵʱ��������ֵ */
    s_tTmr[_id].PreLoad = _period;    /* �������Զ���װֵ�����Զ�ģʽ������ */
    s_tTmr[_id].Flag = 0;             /* ��ʱʱ�䵽��־ */
    s_tTmr[_id].Mode = TMR_ONCE_MODE; /* 1���Թ���ģʽ */

    ENABLE_INT(); /* ���ж� */
}

/*************************************************************************
*	@brief 		�����Զ���ʱ������ʱʱ�䵽���Զ��ñ�־��ѭ������
*	@param 		_id     : ��ʱ��ID��ֵ��0,TMR_COUNT-1��
*	@param 		_period : ��ʱ���ڣ���λ1ms
*	@retval		None
*	@note  		ע���û���Ҫ�Լ�ά����ʱ��ID���Ա��ⶨʱ��ID��ͻ
*	@author		HarryWang
*	@date  		2024/11/11
**************************************************************************
*/
void bsp_StartAutoTimer(uint8_t _id, uint32_t _period)
{
    if (_id >= TMR_COUNT) {
        /* ��ӡ�����Դ�����ļ������������� */
        printf("Error: file %s, function %s()\r\n", __FILE__, __FUNCTION__);
        while (1)
            ; /* �����쳣�������ȴ����Ź���λ */
    }

    DISABLE_INT(); /* ���ж� */

    s_tTmr[_id].Count = _period;      /* ʵʱ��������ֵ */
    s_tTmr[_id].PreLoad = _period;    /* �������Զ���װֵ�����Զ�ģʽ������ */
    s_tTmr[_id].Flag = 0;             /* ��ʱʱ�䵽��־ */
    s_tTmr[_id].Mode = TMR_AUTO_MODE; /* �Զ�����ģʽ */

    ENABLE_INT(); /* ���ж� */
}

/*************************************************************************
*	@brief 		ֹͣ�����ʱ������
*	@param 		_id : ��ʱ��ID��ֵ��0,TMR_COUNT-1��
*	@retval		None
*	@note  		�û���Ҫ�Լ�ά����ʱ��ID���Ա��ⶨʱ��ID��ͻ
*	@author		HarryWang
*	@date  		2024/11/11
**************************************************************************
*/
void bsp_StopTimer(uint8_t _id)
{
    if (_id >= TMR_COUNT) {
        /* ��ӡ�����Դ�����ļ������������� */
        printf("Error: file %s, function %s()\r\n", __FILE__, __FUNCTION__);
        while (1)
            ; /* �����쳣�������ȴ����Ź���λ */
    }

    DISABLE_INT(); /* ���ж� */

    s_tTmr[_id].Count = 0;            /* ʵʱ��������ֵ */
    s_tTmr[_id].Flag = 0;             /* ��ʱʱ�䵽��־ */
    s_tTmr[_id].Mode = TMR_ONCE_MODE; /* �Զ�����ģʽ */

    ENABLE_INT(); /* ���ж� */
}

/*************************************************************************
*	@brief 		��ⶨʱ���Ƿ�ʱ
*	@param 		_id : ��ʱ��ID��ֵ��0,TMR_COUNT-1��
*	@retval		 0 ��ʾ��ʱδ���� 1��ʾ��ʱ��
*	@note  		�û���Ҫ�Լ�ά����ʱ��ID���Ա��ⶨʱ��ID��ͻ
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
*	@brief 		��ȡCPU����ʱ��
*	@param 		None
*	@retval		CPU����ʱ�䣬��λ1ms
*	@note  		ע��CPU�����ʱ��Ϊ49.71�죬�������ʱ����Ҫ�����������
*	@author		HarryWang
*	@date  		2024/11/11
**************************************************************************
*/
int32_t bsp_GetRunTime(void)
{
    int32_t runtime;

    DISABLE_INT(); /* ���ж� */

    runtime = g_iRunTime; /* ���������Systick�ж��б���д�������Ҫ���жϽ��б��� */

    ENABLE_INT(); /* ���ж� */

    return runtime;
}

/*************************************************************************
*	@brief 		���㵱ǰ����ʱ��͸���ʱ��֮��Ĳ�ֵ
*	@param 		_LastTime : �ϸ�ʱ��
*	@retval		��ֵ����λ1ms
*	@note  		None
*	@author		HarryWang
*	@date  		2024/11/11
**************************************************************************
*/
int32_t bsp_CheckRunTime(int32_t _LastTime)
{
    int32_t now_time;
    int32_t time_diff;

    DISABLE_INT(); /* ���ж� */

    now_time = g_iRunTime; /* ���������Systick�ж��б���д�������Ҫ���жϽ��б��� */

    ENABLE_INT(); /* ���ж� */

    if (now_time >= _LastTime) {
        time_diff = now_time - _LastTime;
    } else {
        time_diff = 0x7FFFFFFF - _LastTime + now_time;
    }

    return time_diff;
}

/*************************************************************************
*	@brief 		sysTick�жϷ������
*	@param 		None
*	@retval		None
*	@note  		None
*	@author		HarryWang
*	@date  		2024/11/11
**************************************************************************
*/
void SysTick_Handler(void)
{
    HAL_IncTick(); /* ST HAL��ĵδ�ʱ�жϷ������ */

    if (g_ucEnableSystickISR == 0) {
        return;
    }

    SysTick_ISR(); /* bsp��ĵδ�ʱ�жϷ������ */
}
