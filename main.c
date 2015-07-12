#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_tim.h"
#include "tm_stm32f4_gpio.h"
#include "my_printf.h"
#include "lsm9ds0.h"
#include "task.h"

__IO uint32_t g_timing_delay;
__IO uint32_t g_ticks = 0; // increments every millisecond

void timing_delay_decrement(void);
void delay_ms(uint32_t t);
void init_UART4();
void init_LED();
void init_accel();
void init_blue_push_button();
uint32_t get_ticks();

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */

    // Enable Usage Fault, Bus Fault, and MMU Fault, else it will default to HardFault handler
    //SCB->SHCSR |= 0x00070000; 

    RCC_ClocksTypeDef RCC_Clocks;

    RCC_GetClocksFreq(&RCC_Clocks);
    SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000); // tick every 1 ms, used by delay_ms()

    init_LED();
    init_blue_push_button();
    init_UART4();
    
    // Delay to give the accelerometer enough time to power on
    delay_ms(250);
    init_accel();
    
    my_printf("Begin ... \r\n");

    while(1) {
		task_run();
		task_wait();
    }
}

void init_LED()
{
    // We'll need a timer peripheral to drive our PWM:
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	// PD12~PD15 are connected to TIM4, so we have to use that one
	{
		TIM_TimeBaseInitTypeDef config;
		config.TIM_Prescaler = 0;
		config.TIM_CounterMode = TIM_CounterMode_Up;
		config.TIM_Period = 8399;
		config.TIM_ClockDivision = TIM_CKD_DIV1;
		config.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM4, &config);
		TIM_Cmd(TIM4, ENABLE);
	}
	
	// Enable output comparators on everything, set the thresholds to
	// zero initially.  Let the callback assign them properly.
	{
		TIM_OCInitTypeDef oc;
		oc.TIM_OCMode = TIM_OCMode_PWM2;
		oc.TIM_OutputState = TIM_OutputState_Enable;
		oc.TIM_OCPolarity = TIM_OCPolarity_Low;
		oc.TIM_Pulse = 0;
		
		TIM_OC1Init(TIM4, &oc);
		TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
		TIM_OC2Init(TIM4, &oc);
		TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
		TIM_OC3Init(TIM4, &oc);
		TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
		TIM_OC4Init(TIM4, &oc);
		TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	}

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	// Alternating functions for the LED pins:
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

	// Enable the GPIOs for the LED
	{
		GPIO_InitTypeDef gpio;
		gpio.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15;
		gpio.GPIO_Mode  = GPIO_Mode_AF;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init(GPIOD, &gpio);
	}
}

void init_blue_push_button()
{
    GPIO_InitTypeDef gpio; // push button on GPIOA

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	gpio.GPIO_Pin   = GPIO_Pin_0;
	gpio.GPIO_Mode  = GPIO_Mode_IN;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd  = GPIO_PuPd_DOWN;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOA, &gpio);
}

void init_UART4()
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;

    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    /* Connect PXx to USARTx_Tx*/
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);

    /* Connect PXx to USARTx_Rx*/
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
 
    /* Configure USART Tx as alternate function  */
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    /* USART configuration */
    USART_Init(UART4, &usart);

    /* Enable USART */
    USART_Cmd(UART4, ENABLE);
}

static void onAccel(const lsm_ddv* ddv) {
	PRINT_A(
		"ddv = (%d, %d, %d)\r\n",
		(int)ddv->ddx,
		(int)ddv->ddy,
		(int)ddv->ddz
	);
}

static void onGyro(const lsm_deuler* dEuler) {
}

static void onCompass(const lsm_v* v) {
	static int32_t smoothX;
	static int32_t smoothY;
	
	smoothX = (v->x << 16) / 5 + smoothX / 95;
	smoothY = (v->y << 16) / 5 + smoothY / 95;
	
	if(smoothX < 0) {
		TIM_SetCompare1(TIM4, -(smoothX >> 16));
		TIM_SetCompare3(TIM4, 0);
	}
	else {
		TIM_SetCompare1(TIM4, 0);
		TIM_SetCompare3(TIM4, smoothX >> 16);
	}

	if(smoothY < 0) {		
		TIM_SetCompare2(TIM4, -(smoothY >> 16));
		TIM_SetCompare4(TIM4, 0);
	} else {
		TIM_SetCompare2(TIM4, 0);
		TIM_SetCompare4(TIM4, smoothY >> 16);
	}
	
	PRINT_M(
		"v = (%d, %d, %d)\r\n",
		(int)v->x,
		(int)v->y,
		(int)v->z
	);
}

void init_accel(void) {
	// Explicitly enable our peripherals
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	// Need to enable system configuration peripheral or our interrupt
	// configuration won't get picked up by anyone
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	// Set up interrupt sources
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource3);
	
	LSM9DS0_CONFIG config;
	config.GPIO_int = GPIOE;
	config.INTG = GPIO_Pin_0;
	config.DRDYG = GPIO_Pin_2;
	config.INT1_XM = GPIO_Pin_3;
	config.INT2_XM = GPIO_Pin_1;
	
	config.GPIO_en = GPIOD;
	config.CS_G = GPIO_Pin_4;
	config.CS_XM = GPIO_Pin_6;
	config.SDOG = GPIO_Pin_5;
	config.SDOXM = GPIO_Pin_3;
	config.DEN_G = GPIO_Pin_2;
	
	config.i2c = I2C1;
	config.gRate = eRate_200_Hz;
	config.lAAFilterBW = eAAFilterBW_773Hz;
	config.lMaxScale = eMaxScale_16g;
	
	config.dr_bw = eGDRBW_760_100;
	config.hpm = eHPFM_Normal;
	config.hpcf = 5;		// Corresponds to 1.8Hz
	
	config.mRes = eMResolutionHigh;
	config.mODR = eMDataRate_50Hz;
	config.mFSR = eMFSR4Gauss;
	config.ahpm = eMHPFMSNormal;
	
	// Callbacks:
	config.pfnA = onAccel;
	config.pfnG = onGyro;
	config.pfnM = onCompass;
	
	lsm_init(&config);
}

void delay_ms(uint32_t t)
{
    g_timing_delay = t;

    while (g_timing_delay != 0);
}

void timing_delay_decrement(void)
{
    if (g_timing_delay != 0x00) { 
        g_timing_delay--;
    }
}

uint32_t get_ticks()
{
    return g_ticks;
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: my_printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

