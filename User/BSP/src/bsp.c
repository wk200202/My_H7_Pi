#include "bsp.h"
#include "stm32h743xx.h"

/*
**************************************************************************
*	                          函数声明
**************************************************************************
*/
static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);
static void MPU_Config(void);

/*************************************************************************
*	@brief  	初始化所有的硬件设备
*	@param  	None
*	@retval 	None
*	@note   	None
*	@author 	HarryWang
*	@date   	2024/11/10
**************************************************************************
*/
void bsp_Init(void)
{
    /* 配置MPU */
    MPU_Config();

    /* 使能L1 Cache */
    CPU_CACHE_Enable();

    /* 
       STM32H7xx HAL 库初始化，此时系统用的还是H7自带的64MHz，HSI时钟:
	   - 调用函数HAL_InitTick，初始化滴答时钟中断1ms。
	   - 设置NVIV优先级分组为4。
	 */
    HAL_Init();

    /* 
       配置系统时钟到400MHz
       - 切换使用HSE。
       - 此函数会更新全局变量SystemCoreClock，并重新配置HAL_InitTick。
    */
    SystemClock_Config();

    bsp_RGB_Init();
    bsp_Key_Init();
    bsp_InitTimer();
    bsp_Uart_Init();
    bsp_iic_init();
}

/*
*********************************************************************************************************
*	函 数 名: bsp_RunPer10ms
*	功能说明: 该函数每隔10ms被Systick中断调用1次。详见 bsp_timer.c的定时中断服务程序。一些处理时间要求不严格的
*			任务可以放在此函数。比如：按键扫描、蜂鸣器鸣叫控制等。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_RunPer10ms(void)
{
    bsp_KeyScan10ms();
}

/*
*********************************************************************************************************
*	函 数 名: bsp_RunPer1ms
*	功能说明: 该函数每隔1ms被Systick中断调用1次。详见 bsp_timer.c的定时中断服务程序。一些需要周期性处理的事务
*			 可以放在此函数。比如：触摸坐标扫描。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_RunPer1ms(void)
{
}

/*
*********************************************************************************************************
*	函 数 名: bsp_Idle
*	功能说明: 空闲时执行的函数。一般主程序在for和while循环程序体中需要插入 CPU_IDLE() 宏来调用本函数。
*			 本函数缺省为空操作。用户可以添加喂狗、设置CPU进入休眠模式的功能。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_Idle(void)
{
    /* --- 喂狗 */
}

/*
**************************************************************************
*	@brief  	Initialize the system clock for 480MHz
*	@param  	None
*	@retval 	None
*	@note   	System Clock source            = PLL (HSE)
*            	SYSCLK(Hz)                     = 480000000 (CPU Clock)
*           	HCLK(Hz)                       = 240000000 (AXI and AHBs Clock)
*            	AHB Prescaler                  = 2
*            	D1 APB3 Prescaler              = 2 (APB3 Clock  100MHz)
*            	D2 APB1 Prescaler              = 2 (APB1 Clock  100MHz)
*            	D2 APB2 Prescaler              = 2 (APB2 Clock  100MHz)
*            	D3 APB4 Prescaler              = 2 (APB4 Clock  100MHz)
*            	HSE Frequency(Hz)              = 25000000
*           	PLL_M                          = 5
*            	PLL_N                          = 192
*            	PLL_P                          = 2
*            	PLL_Q                          = 4
*            	PLL_R                          = 2
*            	VDD(V)                         = 3.3
*            	Flash Latency(WS)              = 4
*	@author 	HarryWang
*	@date   	2024/07/14
**************************************************************************
*/
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;
    HAL_StatusTypeDef ret = HAL_OK;

    /* 锁住SCU(Supply configuration update) */
    MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

    /*
	  1、芯片内部的LDO稳压器输出的电压范围，可选VOS1，VOS2和VOS3，不同范围对应不同的Flash读速度，
		 详情看参考手册的Table 12的表格。
	  2、这里选择使用VOS1，电压范围1.15V - 1.26V。
	*/
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    /* 使能HSE，并选择HSE作为PLL时钟源 */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 5;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;

    ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
    if (ret != HAL_OK) {
        while (1)
            ;
    }

    /*
	   选择PLL的输出作为系统时钟
	   配置RCC_CLOCKTYPE_SYSCLK系统时钟
	   配置RCC_CLOCKTYPE_HCLK 时钟，对应AHB1，AHB2，AHB3和AHB4总线
	   配置RCC_CLOCKTYPE_PCLK1时钟，对应APB1总线
	   配置RCC_CLOCKTYPE_PCLK2时钟，对应APB2总线
	   配置RCC_CLOCKTYPE_D1PCLK1时钟，对应APB3总线
	   配置RCC_CLOCKTYPE_D3PCLK1时钟，对应APB4总线
	*/
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1);

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    /* 此函数会更新SystemCoreClock，并重新配置HAL_InitTick */
    ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
    if (ret != HAL_OK) {
        while (1)
            ;
    }

    /*
	  使用IO的高速模式，要使能IO补偿，即调用下面三个函数
	  （1）使能CSI clock
	  （2）使能SYSCFG clock
	  （3）使能I/O补偿单元， 设置SYSCFG_CCCSR寄存器的bit0
	*/
    __HAL_RCC_CSI_ENABLE();

    __HAL_RCC_SYSCFG_CLK_ENABLE();

    HAL_EnableCompensationCell();

    /* AXI SRAM的时钟是上电自动使能的，而D2域的SRAM1，SRAM2和SRAM3要单独使能 */
#if 1
    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    __HAL_RCC_D2SRAM2_CLK_ENABLE();
    __HAL_RCC_D2SRAM3_CLK_ENABLE();
#endif
}

/*************************************************************************
*	@brief  	配置MPU
*	@param  	None
*	@retval 	None
*	@note   	None
*	@author 	HarryWang
*	@date   	2024/11/10
**************************************************************************
*/
static void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct;

    /* 禁止 MPU */
    HAL_MPU_Disable();

#if 0
   	/* 配置AXI SRAM的MPU属性为Write back, Read allocate，Write allocate */
	MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
	MPU_InitStruct.BaseAddress      = 0x24000000;
	MPU_InitStruct.Size             = MPU_REGION_SIZE_512KB;
	MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
	MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
	MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
	MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
	MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
	MPU_InitStruct.SubRegionDisable = 0x00;
	MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);

#else
    /* 配置AXI SRAM的MPU属性为NORMAL, NO Read allocate，NO Write allocate */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x24000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
#endif

    /* 配置FMC扩展IO的MPU属性为Device或者Strongly Ordered */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x60000000;
    MPU_InitStruct.Size = ARM_MPU_REGION_SIZE_64KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE; /* 不能用MPU_ACCESS_CACHEABLE，会出现2次CS、WE信号 */
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER1;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /*使能 MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/*************************************************************************
*	@brief  	使能L1 Cache
*	@param  	None
*	@retval 	None
*	@note   	None
*	@author 	HarryWang
*	@date   	2024/11/10
**************************************************************************
*/
static void CPU_CACHE_Enable(void)
{
    /* 使能 I-Cache */
    SCB_EnableICache();

    /* 使能 D-Cache */
    SCB_EnableDCache();
}

/*************************************************************************
*	@brief  	错误处理函数
*	@param  	file:		错误文件名
*	@param  	line:		错误行号
*	@retval 	None
*	@note   	None
*	@author 	HarryWang
*	@date   	2024/09/22
**************************************************************************
*/
void Error_Handler(char *file, uint32_t line)
{
    printf("Wrong parameters value: file %s on line %d\r\n", file, line);

    /* 这是一个死循环，断言失败时程序会在此处死机，以便于用户查错 */
    if (line == 0) {
        return;
    }

    while (1) {
    }
}