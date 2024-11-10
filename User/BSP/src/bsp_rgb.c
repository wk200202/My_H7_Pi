#include "bsp.h"

/*************************************************************************
*	@brief :	WS2811发送一个字节的数据
*	@param :	dat:数据
*	@retval:	None
*	@note  :	None
*	@author:	HarryWang
*	@date  :	2024/09/21
**************************************************************************
*/
void WS2811_SendByte(uint8_t dat)   //发送1BIT的数据
{
    uint8_t i;
    uint16_t j;
    for (i = 0; i < 8; i++) {
        if (dat & 0x80)   //发送数据1
        {
            RGB_SDA_SET;
            for (j = 0; j < WS2811_T1H; j++) {
                __nop();
            }
            RGB_SDA_RESET;
            for (j = 0; j < WS2811_T1L; j++) {
                __nop();
            }
        } else   //发送数据0
        {
            RGB_SDA_SET;   // 0
            for (j = 0; j < WS2811_T0H; j++) {
                __nop();
            }
            RGB_SDA_RESET;
            for (j = 0; j < WS2811_T0L; j++) {
                __nop();
            }
        }
        dat <<= 1;
    }
}

/*************************************************************************
*	@brief :	WS2811复位
*	@param :	None
*	@retval:	None
*	@note  :	None
*	@author:	HarryWang
*	@date  :	2024/09/21
**************************************************************************
*/
void WS2811_Reset(void)
{
    RGB_SDA_RESET;
    HAL_Delay(1);
}

/*************************************************************************
*	@brief :	bsp RGB改变颜色函数
*	@param :	R:红色
*	@param :	G:绿色
*	@param :	B:蓝色
*	@retval:	None
*	@note  :	None
*	@author:	HarryWang
*	@date  :	2024/09/21
**************************************************************************
*/
void bsp_RGB_Change_Color(uint8_t R, uint8_t G, uint8_t B)
{
    WS2811_SendByte(R);
    WS2811_SendByte(G);
    WS2811_SendByte(B);
    WS2811_Reset();
}

/*
**************************************************************************
*   @brief :    Initialize RGB LED GPIO Peripherals
*   @param :    void
*   @retval:    void
*   @note  :    RGB : PI9
*   @author:    HarryWang
*   @date  :    2024/07/14
**************************************************************************
*/
void bsp_RGB_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    RGB_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Pin = RGB_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RGB_LED_GPIO_Port, &GPIO_InitStruct);
}