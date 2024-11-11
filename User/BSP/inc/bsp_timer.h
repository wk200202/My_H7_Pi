#ifndef _BSP_TIMER_H_
#define _BSP_TIMER_H_

#include "bsp.h"

/***************************************************************************
*	                           �궨��
***************************************************************************/

#define TMR_COUNT 4 /* �����ʱ���ĸ��� */

/***************************************************************************
*	                          ��������
***************************************************************************/

/* ��ʱ���ṹ�壬��Ա���������� volatile, ����C�������Ż�ʱ���������� */

/* ��ʱ������ģʽ */
typedef enum {
    TMR_ONCE_MODE = 0, /* һ�ι���ģʽ */
    TMR_AUTO_MODE = 1  /* �Զ���ʱ����ģʽ */
} TMR_MODE_E;

/* ��ʱ�����ýṹ�� */
typedef struct
{
    volatile uint8_t Mode;     /* ������ģʽ��1���� */
    volatile uint8_t Flag;     /* ��ʱ�����־  */
    volatile uint32_t Count;   /* ������ */
    volatile uint32_t PreLoad; /* ������Ԥװֵ */
} SOFT_TMR;

/***************************************************************************
*	                          ��������
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
