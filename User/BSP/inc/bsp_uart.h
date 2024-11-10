#ifndef __BSP_UART__H
#define __BSP_UART__H

/***************************************************************************
*	                           宏定义
***************************************************************************/

/*
	串口分配：
	【串口1】 DAPLink Debug芯片第1路。
		PB6/USART1_TX	  --- 打印调试口
		PB7/USART1_RX

	【串口2】 RS485 Modbus通信
		PA15/UART7_TX
		PA8 /UART7_RX

	【串口3】 ESP32模块通信
		PA0_C/UART4_TX
		PA1_C/UART4_RX
*/

#define UART1_FIFO_EN 1
#define UART2_FIFO_EN 0
#define UART3_FIFO_EN 0

/* PH13 控制RS485芯片的发送使能 */
#define RS485_TXEN_GPIO_CLK_ENABLE() __HAL_RCC_GPIOH_CLK_ENABLE()
#define RS485_TXEN_GPIO_PORT         GPIOH
#define RS485_TXEN_PIN               GPIO_PIN_13

#define RS485_RX_EN() RS485_TXEN_GPIO_PORT->BSRR = RS485_TXEN_PIN
#define RS485_TX_EN() RS485_TXEN_GPIO_PORT->BSRR = RS485_TXEN_PIN << 16U

/* 定义串口波特率和FIFO缓冲区大小，分为发送缓冲区和接收缓冲区, 支持全双工 */
#if UART1_FIFO_EN == 1
#define UART1_BAUD        115200
#define UART1_TX_BUF_SIZE 1 * 1024
#define UART1_RX_BUF_SIZE 1 * 1024
#endif

#if UART2_FIFO_EN == 1
#define UART2_BAUD        9600
#define UART2_TX_BUF_SIZE 10
#define UART2_RX_BUF_SIZE 2 * 1024
#endif

#if UART3_FIFO_EN == 1
#define UART3_BAUD        9600
#define UART3_TX_BUF_SIZE 1 * 1024
#define UART3_RX_BUF_SIZE 1 * 1024
#endif

/***************************************************************************
*	                          变量声明
***************************************************************************/

/* 定义端口号 */
typedef enum {
    COM1 = 0,
    COM2 = 1,
    COM3 = 2,
} COM_PORT_E;

/* 串口设备结构体 */
typedef struct
{
    USART_TypeDef *uart;     /* STM32内部串口设备指针 */
    uint8_t *pTxBuf;         /* 发送缓冲区 */
    uint8_t *pRxBuf;         /* 接收缓冲区 */
    uint16_t usTxBufSize;    /* 发送缓冲区大小 */
    uint16_t usRxBufSize;    /* 接收缓冲区大小 */
    __IO uint16_t usTxWrite; /* 发送缓冲区写指针 */
    __IO uint16_t usTxRead;  /* 发送缓冲区读指针 */
    __IO uint16_t usTxCount; /* 等待发送的数据个数 */

    __IO uint16_t usRxWrite; /* 接收缓冲区写指针 */
    __IO uint16_t usRxRead;  /* 接收缓冲区读指针 */
    __IO uint16_t usRxCount; /* 还未读取的新数据个数 */

    void (*SendBefor)(void);          /* 开始发送之前的回调函数指针（主要用于RS485切换到发送模式） */
    void (*SendOver)(void);           /* 发送完毕的回调函数指针（主要用于RS485将发送模式切换为接收模式） */
    void (*ReciveNew)(uint8_t _byte); /* 串口收到数据的回调函数指针 */
    uint8_t Sending;                  /* 正在发送中 */
} UART_T;

/***************************************************************************
*	                          函数声明
***************************************************************************/

void bsp_Uart_Init(void);
void comSendBuf(COM_PORT_E _ucPort, uint8_t *_ucaBuf, uint16_t _usLen);
void comSendChar(COM_PORT_E _ucPort, uint8_t _ucByte);
uint8_t comGetChar(COM_PORT_E _ucPort, uint8_t *_pByte);
void comSendBuf(COM_PORT_E _ucPort, uint8_t *_ucaBuf, uint16_t _usLen);
void comClearTxFifo(COM_PORT_E _ucPort);
void comClearRxFifo(COM_PORT_E _ucPort);
void comSetBaud(COM_PORT_E _ucPort, uint32_t _BaudRate);

void USART_SetBaudRate(USART_TypeDef *USARTx, uint32_t BaudRate);
void bsp_SetUartParam(USART_TypeDef *Instance, uint32_t BaudRate, uint32_t Parity, uint32_t Mode);

void RS485_SendBuf(uint8_t *_ucaBuf, uint16_t _usLen);
void RS485_SendStr(char *_pBuf);
void RS485_SetBaud(uint32_t _baud);
uint8_t UartTxEmpty(COM_PORT_E _ucPort);

#endif   // _BSP_UART_H_
