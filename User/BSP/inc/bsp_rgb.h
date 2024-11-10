#ifndef _BSP_RGB_H_
#define _BSP_RGB_H_

/***************************************************************************
*	                           宏定义
***************************************************************************/

#define RGB_GPIO_CLK_ENABLE() __HAL_RCC_GPIOI_CLK_ENABLE()
#define RGB_LED_GPIO_Port     GPIOI
#define RGB_LED_Pin           GPIO_PIN_9

#define RGB_SDA_SET   (GPIOI->BSRR = (1 << 9))          // 设置GPIOI的第9号引脚
#define RGB_SDA_RESET (GPIOI->BSRR = (1 << (9 + 16)))   // 重置GPIOI的第9号引脚

#define WS2811_T0H 12   //220ns~380ns
#define WS2811_T0L 45   //580ns~1us

#define WS2811_T1H 45   //580ns~1us
#define WS2811_T1L 12   //220ns~420ns

/***************************************************************************
*	                          变量声明
***************************************************************************/

/***************************************************************************
*	                          函数声明
***************************************************************************/

void bsp_RGB_Init(void);
void bsp_RGB_Change_Color(uint8_t R, uint8_t G, uint8_t B);

#endif   // _BSP_RGB_H_
