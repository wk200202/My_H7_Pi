#include "bsp.h"

/**
 * 	【串口1】 DAPLink Debug芯片第1路。
		PB6/USART1_TX	  --- 打印调试口
		PB7/USART1_RX
 */
#define USART1_CLK_ENABLE() __HAL_RCC_USART1_CLK_ENABLE()

#define USART1_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define USART1_TX_GPIO_PORT         GPIOB
#define USART1_TX_PIN               GPIO_PIN_6
#define USART1_TX_AF                GPIO_AF7_USART1

#define USART1_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define USART1_RX_GPIO_PORT         GPIOB
#define USART1_RX_PIN               GPIO_PIN_7
#define USART1_RX_AF                GPIO_AF7_USART1

/**
 * 	【串口2】 RS485 Modbus通信
		PA15/UART7_TX
		PA8 /UART7_RX
 */
#define UART7_CLK_ENABLE() __HAL_RCC_UART7_CLK_ENABLE()

#define UART7_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define UART7_TX_GPIO_PORT         GPIOA
#define UART7_TX_PIN               GPIO_PIN_15
#define UART7_TX_AF                GPIO_AF11_UART7

#define UART7_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define UART7_RX_GPIO_PORT         GPIOA
#define UART7_RX_PIN               GPIO_PIN_8
#define UART7_RX_AF                GPIO_AF11_UART7

/**
 * 	【串口3】 ESP32模块通信
        PA0_C/UART4_TX
        PA1_C/UART4_RX
 */
#define UART4_CLK_ENABLE() __HAL_RCC_UART4_CLK_ENABLE()

#define UART4_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define UART4_TX_GPIO_PORT         GPIOA
#define UART4_TX_PIN               GPIO_PIN_0
#define UART4_TX_AF                GPIO_AF8_UART4

#define UART4_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define UART4_RX_GPIO_PORT         GPIOA
#define UART4_RX_PIN               GPIO_PIN_1
#define UART4_RX_AF                GPIO_AF8_UART4

#if UART1_FIFO_EN == 1
static UART_T g_tUart1;
static uint8_t g_TxBuf1[UART1_TX_BUF_SIZE]; /* 发送缓冲区 */
static uint8_t g_RxBuf1[UART1_RX_BUF_SIZE]; /* 接收缓冲区 */
#endif

#if UART2_FIFO_EN == 1
static UART_T g_tUart2;
static uint8_t g_TxBuf2[UART2_TX_BUF_SIZE]; /* 发送缓冲区 */
static uint8_t g_RxBuf2[UART2_RX_BUF_SIZE]; /* 接收缓冲区 */
#endif

#if UART3_FIFO_EN == 1
static UART_T g_tUart3;
static uint8_t g_TxBuf3[UART3_TX_BUF_SIZE]; /* 发送缓冲区 */
static uint8_t g_RxBuf3[UART3_RX_BUF_SIZE]; /* 接收缓冲区 */
#endif

/*********函数声明*********/
static void UartVarInit(void);
static void InitHardUart(void);
static void UartSend(UART_T *_pUart, uint8_t *_ucaBuf, uint16_t _usLen);
static uint8_t UartGetChar(UART_T *_pUart, uint8_t *_pByte);
static void UartIRQ(UART_T *_pUart);
void RS485_InitTXE(void);

/*************************************************************************
*	@brief :	串口初始化
*	@param :	None
*	@retval:	None
*	@note  :	None
*	@author:	HarryWang
*	@date  :	2024/09/21
**************************************************************************
*/
void bsp_Uart_Init(void)
{
    UartVarInit(); /* 必须先初始化全局结构体变量,再配置硬件 */

    InitHardUart(); /* 配置串口的硬件参数(波特率等) */

    RS485_InitTXE(); /* 配置RS485芯片的发送使能硬件，配置为推挽输出 */
}

/*************************************************************************
*	@brief :	初始化串口相关结构体变量
*	@param :	None
*	@retval:	None
*	@note  :	None
*	@author:	HarryWang
*	@date  :	2024/09/21
**************************************************************************
*/
static void UartVarInit(void)
{
#if UART1_FIFO_EN == 1
    g_tUart1.uart = USART1;                   /* STM32 串口设备 */
    g_tUart1.pTxBuf = g_TxBuf1;               /* 发送缓冲区指针 */
    g_tUart1.pRxBuf = g_RxBuf1;               /* 接收缓冲区指针 */
    g_tUart1.usTxBufSize = UART1_TX_BUF_SIZE; /* 发送缓冲区大小 */
    g_tUart1.usRxBufSize = UART1_RX_BUF_SIZE; /* 接收缓冲区大小 */
    g_tUart1.usTxWrite = 0;                   /* 发送FIFO写索引 */
    g_tUart1.usTxRead = 0;                    /* 发送FIFO读索引 */
    g_tUart1.usRxWrite = 0;                   /* 接收FIFO写索引 */
    g_tUart1.usRxRead = 0;                    /* 接收FIFO读索引 */
    g_tUart1.usRxCount = 0;                   /* 接收到的新数据个数 */
    g_tUart1.usTxCount = 0;                   /* 待发送的数据个数 */
    g_tUart1.SendBefor = 0;                   /* 发送数据前的回调函数 */
    g_tUart1.SendOver = 0;                    /* 发送完毕后的回调函数 */
    g_tUart1.ReciveNew = 0;                   /* 接收到新数据后的回调函数 */
    g_tUart1.Sending = 0;                     /* 正在发送中标志 */
#endif

#if UART2_FIFO_EN == 1
    g_tUart2.uart = USART2;                   /* STM32 串口设备 */
    g_tUart2.pTxBuf = g_TxBuf2;               /* 发送缓冲区指针 */
    g_tUart2.pRxBuf = g_RxBuf2;               /* 接收缓冲区指针 */
    g_tUart2.usTxBufSize = UART2_TX_BUF_SIZE; /* 发送缓冲区大小 */
    g_tUart2.usRxBufSize = UART2_RX_BUF_SIZE; /* 接收缓冲区大小 */
    g_tUart2.usTxWrite = 0;                   /* 发送FIFO写索引 */
    g_tUart2.usTxRead = 0;                    /* 发送FIFO读索引 */
    g_tUart2.usRxWrite = 0;                   /* 接收FIFO写索引 */
    g_tUart2.usRxRead = 0;                    /* 接收FIFO读索引 */
    g_tUart2.usRxCount = 0;                   /* 接收到的新数据个数 */
    g_tUart2.usTxCount = 0;                   /* 待发送的数据个数 */
    g_tUart2.SendBefor = RS485_SendBefor;     /* 发送数据前的回调函数 */
    g_tUart2.SendOver = RS485_SendOver;       /* 发送完毕后的回调函数 */
    g_tUart2.ReciveNew = RS485_ReciveNew;     /* 接收到新数据后的回调函数 */
    g_tUart2.Sending = 0;                     /* 正在发送中标志 */
#endif

#if UART3_FIFO_EN == 1
    g_tUart3.uart = USART3;                   /* STM32 串口设备 */
    g_tUart3.pTxBuf = g_TxBuf3;               /* 发送缓冲区指针 */
    g_tUart3.pRxBuf = g_RxBuf3;               /* 接收缓冲区指针 */
    g_tUart3.usTxBufSize = UART3_TX_BUF_SIZE; /* 发送缓冲区大小 */
    g_tUart3.usRxBufSize = UART3_RX_BUF_SIZE; /* 接收缓冲区大小 */
    g_tUart3.usTxWrite = 0;                   /* 发送FIFO写索引 */
    g_tUart3.usTxRead = 0;                    /* 发送FIFO读索引 */
    g_tUart3.usRxWrite = 0;                   /* 接收FIFO写索引 */
    g_tUart3.usRxRead = 0;                    /* 接收FIFO读索引 */
    g_tUart3.usRxCount = 0;                   /* 接收到的新数据个数 */
    g_tUart3.usTxCount = 0;                   /* 待发送的数据个数 */
    g_tUart3.SendBefor = 0;                   /* 发送数据前的回调函数 */
    g_tUart3.SendOver = 0;                    /* 发送完毕后的回调函数 */
    g_tUart3.ReciveNew = 0;                   /* 接收到新数据后的回调函数 */
    g_tUart3.Sending = 0;                     /* 正在发送中标志 */
#endif
}

void bsp_SetUartParam(USART_TypeDef *Instance, uint32_t BaudRate, uint32_t Parity, uint32_t Mode)
{
    UART_HandleTypeDef UartHandle;

    /*##-1- 配置串口硬件参数 ######################################*/
    /* 异步串口模式 (UART Mode) */
    /* 配置如下:
	  - 字长    = 8 位
	  - 停止位  = 1 个停止位
	  - 校验    = 参数Parity
	  - 波特率  = 参数BaudRate
	  - 硬件流控制关闭 (RTS and CTS signals) */

    UartHandle.Instance = Instance;

    UartHandle.Init.BaudRate = BaudRate;
    UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits = UART_STOPBITS_1;
    UartHandle.Init.Parity = Parity;
    UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode = Mode;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    UartHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    UartHandle.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    UartHandle.FifoMode = UART_FIFOMODE_DISABLE;
    UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    HAL_UARTEx_SetTxFifoThreshold(&UartHandle, UART_TXFIFO_THRESHOLD_1_8);
    HAL_UARTEx_SetRxFifoThreshold(&UartHandle, UART_RXFIFO_THRESHOLD_1_8);

    if (HAL_UART_Init(&UartHandle) != HAL_OK) {
        Error_Handler(__FILE__, __LINE__);
    }
}

static void InitHardUart(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

    /* 
       下面这个配置可以注释掉，预留下来是为了方便以后选择其它时钟使用 
       默认情况下，USART1和USART6选择的PCLK2，时钟100MHz。
       USART2，USART3，UART4，UART5，UART6，UART7和UART8选择的时钟是PLCK1，时钟100MHz。
    */
    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART16;
    RCC_PeriphClkInit.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

#if UART1_FIFO_EN == 1 /* 串口1 */
    /* 使能 GPIO TX/RX 时钟 */
    USART1_TX_GPIO_CLK_ENABLE();
    USART1_RX_GPIO_CLK_ENABLE();

    /* 使能 USARTx 时钟 */
    USART1_CLK_ENABLE();

    /* 配置TX引脚 */
    GPIO_InitStruct.Pin = USART1_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = USART1_TX_AF;
    HAL_GPIO_Init(USART1_TX_GPIO_PORT, &GPIO_InitStruct);

    /* 配置RX引脚 */
    GPIO_InitStruct.Pin = USART1_RX_PIN;
    GPIO_InitStruct.Alternate = USART1_RX_AF;
    HAL_GPIO_Init(USART1_RX_GPIO_PORT, &GPIO_InitStruct);

    /* 配置NVIC the NVIC for UART */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* 配置波特率、奇偶校验 */
    bsp_SetUartParam(USART1, UART1_BAUD, UART_PARITY_NONE, UART_MODE_TX_RX);

    SET_BIT(USART1->ICR, USART_ICR_TCCF);  /* 清除TC发送完成标志 */
    SET_BIT(USART1->RQR, USART_RQR_RXFRQ); /* 清除RXNE接收标志 */
    // USART_CR1_PEIE | USART_CR1_RXNEIE
    SET_BIT(USART1->CR1, USART_CR1_RXNEIE); /* 使能PE. RX接受中断 */
#endif

#if UART2_FIFO_EN == 1 /* 串口2 */
    /* 使能 GPIO TX/RX 时钟 */
    USART2_TX_GPIO_CLK_ENABLE();
    USART2_RX_GPIO_CLK_ENABLE();

    /* 使能 USARTx 时钟 */
    USART2_CLK_ENABLE();

    /* 配置TX引脚 */
    GPIO_InitStruct.Pin = USART2_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = USART2_TX_AF;
    HAL_GPIO_Init(USART2_TX_GPIO_PORT, &GPIO_InitStruct);

    /* 配置RX引脚 */
    GPIO_InitStruct.Pin = USART2_RX_PIN;
    GPIO_InitStruct.Alternate = USART2_RX_AF;
    HAL_GPIO_Init(USART2_RX_GPIO_PORT, &GPIO_InitStruct);

    /* 配置NVIC the NVIC for UART */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    /* 配置波特率、奇偶校验 */
    bsp_SetUartParam(USART2, UART2_BAUD, UART_PARITY_NONE, UART_MODE_RX);   // UART_MODE_TX_RX

    SET_BIT(USART2->ICR, USART_ICR_TCCF);   /* 清除TC发送完成标志 */
    SET_BIT(USART2->RQR, USART_RQR_RXFRQ);  /* 清除RXNE接收标志 */
    SET_BIT(USART2->CR1, USART_CR1_RXNEIE); /* 使能PE. RX接受中断 */
#endif

#if UART3_FIFO_EN == 1 /* 串口3 */
    /* 使能 GPIO TX/RX 时钟 */
    USART3_TX_GPIO_CLK_ENABLE();
    USART3_RX_GPIO_CLK_ENABLE();

    /* 使能 USARTx 时钟 */
    USART3_CLK_ENABLE();

    /* 配置TX引脚 */
    GPIO_InitStruct.Pin = USART3_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = USART3_TX_AF;
    HAL_GPIO_Init(USART3_TX_GPIO_PORT, &GPIO_InitStruct);

    /* 配置RX引脚 */
    GPIO_InitStruct.Pin = USART3_RX_PIN;
    GPIO_InitStruct.Alternate = USART3_RX_AF;
    HAL_GPIO_Init(USART3_RX_GPIO_PORT, &GPIO_InitStruct);

    /* 配置NVIC the NVIC for UART */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 3);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    /* 配置波特率、奇偶校验 */
    bsp_SetUartParam(USART3, UART3_BAUD, UART_PARITY_NONE, UART_MODE_TX_RX);

    SET_BIT(USART3->ICR, USART_ICR_TCCF);   /* 清除TC发送完成标志 */
    SET_BIT(USART3->RQR, USART_RQR_RXFRQ);  /* 清除RXNE接收标志 */
    SET_BIT(USART3->CR1, USART_CR1_RXNEIE); /* 使能PE. RX接受中断 */
#endif
}

/*************************************************************************
*	@brief :	RS485的发送使能引脚初始化
*	@param :	None
*	@retval:	None
*	@note  :	None
*	@author:	HarryWang
*	@date  :	2024/09/22
**************************************************************************
*/
void RS485_InitTXE(void)
{
    GPIO_InitTypeDef gpio_init;

    /* 打开GPIO时钟 */
    RS485_TXEN_GPIO_CLK_ENABLE();

    /* 配置引脚为推挽输出 */
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;        /* 推挽输出 */
    gpio_init.Pull = GPIO_NOPULL;                /* 上下拉电阻不使能 */
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* GPIO速度等级 */
    gpio_init.Pin = RS485_TXEN_PIN;
    HAL_GPIO_Init(RS485_TXEN_GPIO_PORT, &gpio_init);
}

/*
*********************************************************************************************************
*	函 数 名: RS485_SendBefor
*	功能说明: 发送数据前的准备工作。对于RS485通信，请设置RS485芯片为发送状态，
*			  并修改 UartVarInit()中的函数指针等于本函数名，比如 g_tUart2.SendBefor = RS485_SendBefor
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void RS485_SendBefor(void)
{
    RS485_TX_EN(); /* 切换RS485收发芯片为发送模式 */
}

/*
*********************************************************************************************************
*	函 数 名: RS485_SendOver
*	功能说明: 发送一串数据结束后的善后处理。对于RS485通信，请设置RS485芯片为接收状态，
*			  并修改 UartVarInit()中的函数指针等于本函数名，比如 g_tUart2.SendOver = RS485_SendOver
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void RS485_SendOver(void)
{
    RS485_RX_EN(); /* 切换RS485收发芯片为接收模式 */
}

/*
*********************************************************************************************************
*	函 数 名: RS485_SendBuf
*	功能说明: 通过RS485芯片发送一串数据。注意，本函数不等待发送完毕。
*	形    参: _ucaBuf : 数据缓冲区
*			  _usLen : 数据长度
*	返 回 值: 无
*********************************************************************************************************
*/
void RS485_SendBuf(uint8_t *_ucaBuf, uint16_t _usLen)
{
    comSendBuf(COM2, _ucaBuf, _usLen);
}

/*
*********************************************************************************************************
*	函 数 名: RS485_SendStr
*	功能说明: 向485总线发送一个字符串，0结束。
*	形    参: _pBuf 字符串，0结束
*	返 回 值: 无
*********************************************************************************************************
*/
void RS485_SendStr(char *_pBuf)
{
    RS485_SendBuf((uint8_t *)_pBuf, strlen(_pBuf));
}

/*
*********************************************************************************************************
*	函 数 名: ComToUart
*	功能说明: 将COM端口号转换为UART指针
*	形    参: _ucPort: 端口号(COM1 - COM8)
*	返 回 值: uart指针
*********************************************************************************************************
*/
UART_T *ComToUart(COM_PORT_E _ucPort)
{
    if (_ucPort == COM1) {
#if UART1_FIFO_EN == 1
        return &g_tUart1;
#else
        return 0;
#endif
    } else if (_ucPort == COM2) {
#if UART2_FIFO_EN == 1
        return &g_tUart2;
#else
        return 0;
#endif
    } else if (_ucPort == COM3) {
#if UART3_FIFO_EN == 1
        return &g_tUart3;
#else
        return 0;
#endif
    } else {
        Error_Handler(__FILE__, __LINE__);
        return 0;
    }
}

/*
*********************************************************************************************************
*	函 数 名: ComToUart
*	功能说明: 将COM端口号转换为 USART_TypeDef* USARTx
*	形    参: _ucPort: 端口号(COM1 - COM8)
*	返 回 值: USART_TypeDef*,  USART1, USART2, USART3, UART4, UART5，USART6，UART7，UART8。
*********************************************************************************************************
*/
USART_TypeDef *ComToUSARTx(COM_PORT_E _ucPort)
{
    if (_ucPort == COM1) {
#if UART1_FIFO_EN == 1
        return USART1;
#else
        return 0;
#endif
    } else if (_ucPort == COM2) {
#if UART2_FIFO_EN == 1
        return USART2;
#else
        return 0;
#endif
    } else if (_ucPort == COM3) {
#if UART3_FIFO_EN == 1
        return USART3;
#else
        return 0;
#endif
    }

    else {
        /* 不做任何处理 */
        return 0;
    }
}
/*
*********************************************************************************************************
*	函 数 名: comSendBuf
*	功能说明: 向串口发送一组数据。数据放到发送缓冲区后立即返回，由中断服务程序在后台完成发送
*	形    参: _ucPort: 端口号(COM1 - COM8)
*			  _ucaBuf: 待发送的数据缓冲区
*			  _usLen : 数据长度
*	返 回 值: 无
*********************************************************************************************************
*/
void comSendBuf(COM_PORT_E _ucPort, uint8_t *_ucaBuf, uint16_t _usLen)
{
    UART_T *pUart;

    pUart = ComToUart(_ucPort);
    if (pUart == 0) {
        return;
    }

    if (pUart->SendBefor != 0) {
        pUart->SendBefor(); /* 如果是RS485通信，可以在这个函数中将RS485设置为发送模式 */
    }

    UartSend(pUart, _ucaBuf, _usLen);
}

/*
*********************************************************************************************************
*	函 数 名: comSendChar
*	功能说明: 向串口发送1个字节。数据放到发送缓冲区后立即返回，由中断服务程序在后台完成发送
*	形    参: _ucPort: 端口号(COM1 - COM8)
*			  _ucByte: 待发送的数据
*	返 回 值: 无
*********************************************************************************************************
*/
void comSendChar(COM_PORT_E _ucPort, uint8_t _ucByte)
{
    comSendBuf(_ucPort, &_ucByte, 1);
}

/*
*********************************************************************************************************
*	函 数 名: comGetChar
*	功能说明: 从接收缓冲区读取1字节，非阻塞。无论有无数据均立即返回。
*	形    参: _ucPort: 端口号(COM1 - COM8)
*			  _pByte: 接收到的数据存放在这个地址
*	返 回 值: 0 表示无数据, 1 表示读取到有效字节
*********************************************************************************************************
*/
uint8_t comGetChar(COM_PORT_E _ucPort, uint8_t *_pByte)
{
    UART_T *pUart;

    pUart = ComToUart(_ucPort);
    if (pUart == 0) {
        return 0;
    }

    return UartGetChar(pUart, _pByte);
}

/*
*********************************************************************************************************
*	函 数 名: comClearTxFifo
*	功能说明: 清零串口发送缓冲区
*	形    参: _ucPort: 端口号(COM1 - COM8)
*	返 回 值: 无
*********************************************************************************************************
*/
void comClearTxFifo(COM_PORT_E _ucPort)
{
    UART_T *pUart;

    pUart = ComToUart(_ucPort);
    if (pUart == 0) {
        return;
    }

    pUart->usTxWrite = 0;
    pUart->usTxRead = 0;
    pUart->usTxCount = 0;
}

/*
*********************************************************************************************************
*	函 数 名: comClearRxFifo
*	功能说明: 清零串口接收缓冲区
*	形    参: _ucPort: 端口号(COM1 - COM8)
*	返 回 值: 无
*********************************************************************************************************
*/
void comClearRxFifo(COM_PORT_E _ucPort)
{
    UART_T *pUart;

    pUart = ComToUart(_ucPort);
    if (pUart == 0) {
        return;
    }

    pUart->usRxWrite = 0;
    pUart->usRxRead = 0;
    pUart->usRxCount = 0;
}

/*
*********************************************************************************************************
*	函 数 名: comSetBaud
*	功能说明: 设置串口的波特率. 本函数固定设置为无校验，收发都使能模式
*	形    参: _ucPort: 端口号(COM1 - COM8)
*			  _BaudRate: 波特率，8倍过采样  波特率.0-12.5Mbps
*                                16倍过采样 波特率.0-6.25Mbps
*	返 回 值: 无
*********************************************************************************************************
*/
void comSetBaud(COM_PORT_E _ucPort, uint32_t _BaudRate)
{
    USART_TypeDef *USARTx;

    USARTx = ComToUSARTx(_ucPort);
    if (USARTx == 0) {
        return;
    }

    bsp_SetUartParam(USARTx, _BaudRate, UART_PARITY_NONE, UART_MODE_TX_RX);
}

/*************************************************************************
*	@brief :	填写数据到UART发送缓冲区,并启动发送中断
*	@param :	_pUart : 串口设备
*	@param :	_ucaBuf : 待发送数据缓冲区
*	@param :	_usLen : 待发送数据长度
*	@retval:	None
*	@note  :	None
*	@author:	HarryWang
*	@date  :	2024/09/22
**************************************************************************
*/
static void UartSend(UART_T *_pUart, uint8_t *_ucaBuf, uint16_t _usLen)
{
    uint16_t i;

    for (i = 0; i < _usLen; i++) {
        /* 如果发送缓冲区已经满了，则等待缓冲区空 */
        while (1) {
            __IO uint16_t usCount;

            DISABLE_INT();
            usCount = _pUart->usTxCount;
            ENABLE_INT();

            if (usCount < _pUart->usTxBufSize) {
                break;
            } else if (usCount == _pUart->usTxBufSize) /* 数据已填满缓冲区 */
            {
                if ((_pUart->uart->CR1 & USART_CR1_TXEIE) == 0) {
                    SET_BIT(_pUart->uart->CR1, USART_CR1_TXEIE);
                }
            }
        }

        /* 将新数据填入发送缓冲区 */
        _pUart->pTxBuf[_pUart->usTxWrite] = _ucaBuf[i];

        DISABLE_INT();
        if (++_pUart->usTxWrite >= _pUart->usTxBufSize) {
            _pUart->usTxWrite = 0;
        }
        _pUart->usTxCount++;
        ENABLE_INT();
    }

    SET_BIT(_pUart->uart->CR1, USART_CR1_TXEIE); /* 使能发送中断（缓冲区空） */
}

/*************************************************************************
*	@brief :	从串口接收缓冲区读取1字节数据
*	@param :	_pUart : 串口设备
*	@param :	_pByte : 存放读取数据的指针
*	@retval:	0 表示无数据  1表示读取到数据
*	@note  :	None
*	@author:	HarryWang
*	@date  :	2024/09/22
**************************************************************************
*/
static uint8_t UartGetChar(UART_T *_pUart, uint8_t *_pByte)
{
    uint16_t usCount;

    /* usRxWrite 变量在中断函数中被改写，主程序读取该变量时，必须进行临界区保护 */
    DISABLE_INT();
    usCount = _pUart->usRxCount;
    ENABLE_INT();

    /* 如果读和写索引相同，则返回0 */
    //if (_pUart->usRxRead == usRxWrite)
    if (usCount == 0) /* 已经没有数据 */
    {
        return 0;
    } else {
        *_pByte = _pUart->pRxBuf[_pUart->usRxRead]; /* 从串口接收FIFO取1个数据 */

        /* 改写FIFO读索引 */
        DISABLE_INT();
        if (++_pUart->usRxRead >= _pUart->usRxBufSize) {
            _pUart->usRxRead = 0;
        }
        _pUart->usRxCount--;
        ENABLE_INT();
        return 1;
    }
}

/*************************************************************************
*	@brief :	判断发送缓冲区是否为空
*	@param :	_ucPort : 串口号
*	@retval:	1为空。0为不空。
*	@note  :	None
*	@author:	HarryWang
*	@date  :	2024/09/22
**************************************************************************
*/
uint8_t UartTxEmpty(COM_PORT_E _ucPort)
{
    UART_T *pUart;
    uint8_t Sending;

    pUart = ComToUart(_ucPort);
    if (pUart == 0) {
        return 0;
    }

    Sending = pUart->Sending;

    if (Sending != 0) {
        return 0;
    }
    return 1;
}

/*************************************************************************
*	@brief :	串口中断处理函数
*	@param :	_pUart : 串口结构体
*	@retval:	None
*	@note  :	None
*	@author:	HarryWang
*	@date  :	2024/09/22
**************************************************************************
*/
static void UartIRQ(UART_T *_pUart)
{
    uint32_t isrflags = READ_REG(_pUart->uart->ISR);
    uint32_t cr1its = READ_REG(_pUart->uart->CR1);
    uint32_t cr3its = READ_REG(_pUart->uart->CR3);

    /* 处理接收中断  */
    if ((isrflags & USART_ISR_RXNE_RXFNE) != RESET) {
        /* 从串口接收数据寄存器读取数据存放到接收FIFO */
        uint8_t ch;

        ch = READ_REG(_pUart->uart->RDR);
        _pUart->pRxBuf[_pUart->usRxWrite] = ch;
        if (++_pUart->usRxWrite >= _pUart->usRxBufSize) {
            _pUart->usRxWrite = 0;
        }
        if (_pUart->usRxCount < _pUart->usRxBufSize) {
            _pUart->usRxCount++;
        }

        /* 回调函数,通知应用程序收到新数据,一般是发送1个消息或者设置一个标记 */
        //if (_pUart->usRxWrite == _pUart->usRxRead)
        //if (_pUart->usRxCount == 1)
        {
            if (_pUart->ReciveNew) {
                _pUart->ReciveNew(ch); /* 比如，交给MODBUS解码程序处理字节流 */
            }
        }
    }

    /* 处理发送缓冲区空中断 */
    if (((isrflags & USART_ISR_TXE_TXFNF) != RESET) && (cr1its & USART_CR1_TXEIE) != RESET) {
        //if (_pUart->usTxRead == _pUart->usTxWrite)
        if (_pUart->usTxCount == 0) {
            /* 发送缓冲区的数据已取完时， 禁止发送缓冲区空中断 （注意：此时最后1个数据还未真正发送完毕）*/
            //USART_ITConfig(_pUart->uart, USART_IT_TXE, DISABLE);
            CLEAR_BIT(_pUart->uart->CR1, USART_CR1_TXEIE);

            /* 使能数据发送完毕中断 */
            //USART_ITConfig(_pUart->uart, USART_IT_TC, ENABLE);
            SET_BIT(_pUart->uart->CR1, USART_CR1_TCIE);
        } else {
            _pUart->Sending = 1;

            /* 从发送FIFO取1个字节写入串口发送数据寄存器 */
            //USART_SendData(_pUart->uart, _pUart->pTxBuf[_pUart->usTxRead]);
            _pUart->uart->TDR = _pUart->pTxBuf[_pUart->usTxRead];
            if (++_pUart->usTxRead >= _pUart->usTxBufSize) {
                _pUart->usTxRead = 0;
            }
            _pUart->usTxCount--;
        }
    }
    /* 数据bit位全部发送完毕的中断 */
    if (((isrflags & USART_ISR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET)) {
        //if (_pUart->usTxRead == _pUart->usTxWrite)
        if (_pUart->usTxCount == 0) {
            /* 如果发送FIFO的数据全部发送完毕，禁止数据发送完毕中断 */
            //USART_ITConfig(_pUart->uart, USART_IT_TC, DISABLE);
            CLEAR_BIT(_pUart->uart->CR1, USART_CR1_TCIE);

            /* 回调函数, 一般用来处理RS485通信，将RS485芯片设置为接收模式，避免抢占总线 */
            if (_pUart->SendOver) {
                _pUart->SendOver();
            }

            _pUart->Sending = 0;
        } else {
            /* 正常情况下，不会进入此分支 */

            /* 如果发送FIFO的数据还未完毕，则从发送FIFO取1个数据写入发送数据寄存器 */
            //USART_SendData(_pUart->uart, _pUart->pTxBuf[_pUart->usTxRead]);
            _pUart->uart->TDR = _pUart->pTxBuf[_pUart->usTxRead];
            if (++_pUart->usTxRead >= _pUart->usTxBufSize) {
                _pUart->usTxRead = 0;
            }
            _pUart->usTxCount--;
        }
    }

    /* 清除中断标志 */
    SET_BIT(_pUart->uart->ICR, UART_CLEAR_PEF);
    SET_BIT(_pUart->uart->ICR, UART_CLEAR_FEF);
    SET_BIT(_pUart->uart->ICR, UART_CLEAR_NEF);
    SET_BIT(_pUart->uart->ICR, UART_CLEAR_OREF);
    SET_BIT(_pUart->uart->ICR, UART_CLEAR_IDLEF);
    SET_BIT(_pUart->uart->ICR, UART_CLEAR_TCF);
    SET_BIT(_pUart->uart->ICR, UART_CLEAR_LBDF);
    SET_BIT(_pUart->uart->ICR, UART_CLEAR_CTSF);
    SET_BIT(_pUart->uart->ICR, UART_CLEAR_CMF);
    SET_BIT(_pUart->uart->ICR, UART_CLEAR_WUF);
    SET_BIT(_pUart->uart->ICR, UART_CLEAR_TXFECF);

    //	*            @arg UART_CLEAR_PEF: Parity Error Clear Flag
    //  *            @arg UART_CLEAR_FEF: Framing Error Clear Flag
    //  *            @arg UART_CLEAR_NEF: Noise detected Clear Flag
    //  *            @arg UART_CLEAR_OREF: OverRun Error Clear Flag
    //  *            @arg UART_CLEAR_IDLEF: IDLE line detected Clear Flag
    //  *            @arg UART_CLEAR_TCF: Transmission Complete Clear Flag
    //  *            @arg UART_CLEAR_LBDF: LIN Break Detection Clear Flag
    //  *            @arg UART_CLEAR_CTSF: CTS Interrupt Clear Flag
    //  *            @arg UART_CLEAR_RTOF: Receiver Time Out Clear Flag
    //  *            @arg UART_CLEAR_CMF: Character Match Clear Flag
    //  *            @arg.UART_CLEAR_WUF:  Wake Up from stop mode Clear Flag
    //  *            @arg UART_CLEAR_TXFECF: TXFIFO empty Clear Flag
}

/*************************************************************************
*	@brief :	串口中断处理函数
*	@param :	None
*	@retval:	None
*	@note  :	None
*	@author:	HarryWang
*	@date  :	2024/09/22
**************************************************************************
*/
#if UART1_FIFO_EN == 1
void USART1_IRQHandler(void)
{
    UartIRQ(&g_tUart1);
}
#endif

#if UART2_FIFO_EN == 1
void UART7_IRQHandler(void)
{
    UartIRQ(&g_tUart2);
}
#endif

#if UART3_FIFO_EN == 1
void UART4_IRQHandler(void)
{
    UartIRQ(&g_tUart3);
}
#endif

/*
*********************************************************************************************************
*	函 数 名: fputc
*	功能说明: 重定义putc函数，这样可以使用printf函数从串口1打印输出
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
int fputc(int ch, FILE *f)
{
#if 0 /* 将需要printf的字符通过串口中断FIFO发送出去，printf函数会立即返回 */
	comSendChar(COM1, ch);
	
	return ch;
#else /* 采用阻塞方式发送每个字符,等待数据发送完毕 */
    /* 写一个字节到USART1 */
    USART1->TDR = ch;

    /* 等待发送结束 */
    while ((USART1->ISR & USART_ISR_TC) == 0) {
    }

    return ch;
#endif
}
