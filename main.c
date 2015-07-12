#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
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
void init_cs();
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
    GPIO_InitTypeDef gpio; // LEDS on GPIOD

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |  GPIO_Pin_15;
	gpio.GPIO_Mode  = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOD, &gpio);
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

static void print_accel(const lsm_ddv* ddv) {
	my_printf(
		"ddv = (%d, %d, %d)\r\n",
		(int)ddv->ddx,
		(int)ddv->ddy,
		(int)ddv->ddz
	);
}

static void print_gyro(const lsm_deuler* dEuler) {
}

static void print_compass(const lsm_v* v) {
	my_printf(
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
	
	config.mRes = eMResolutionLow;
	config.mODR = eMDataRate_3_125Hz;
	config.mFSR = eMFSR4Gauss;
	config.ahpm = eMHPFMSNormal;
	config.pfnA = print_accel;
	config.pfnG = print_gyro;
	config.pfnM = print_compass;
	
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

