#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "lsm9ds0.h"
#include "tm_stm32f4_i2c.h"
#include "my_printf.h"
#include "lsm9ds0_regs.h"

static LSM9DS0_CONFIG g_config;

// Seven-bit slave addresses, shifted left one bit already
static uint32_t i2c_addr_am = 0x3A;
static uint32_t i2c_addr_g = 0xD6;

static void i2c_read_am(eI2CAddr_AM reg, void* value) { *(uint8_t*)value = TM_I2C_Read(g_config.i2c, i2c_addr_am, reg); }
static void i2c_write_am(eI2CAddr_AM reg, const void* value) { TM_I2C_Write(g_config.i2c, i2c_addr_am, reg, *(uint8_t*)value); }
static void i2c_read_g(eI2CAddr_G reg, void* value) { *(uint8_t*)value = TM_I2C_Read(g_config.i2c, i2c_addr_am, reg); }
static void i2c_write_g(eI2CAddr_G reg, const void* value) { TM_I2C_Write(g_config.i2c, i2c_addr_g, reg, *(uint8_t*)value); }

void lsm_init(const LSM9DS0_CONFIG* config) {
	// Copy over configuration block:
	g_config = *config;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// Configure chip select pins, we want to pull them all high so we
	// can use i2c everywhere
	{
		GPIO_InitTypeDef gpio;
		gpio.GPIO_Pin = g_config.CS_G | g_config.CS_XM | g_config.SDOG | g_config.SDOXM;
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;

		GPIO_Init(g_config.GPIO_en, &gpio);
		GPIO_SetBits(g_config.GPIO_en, gpio.GPIO_Pin);
	}

	// Perform i2c initialization:
	TM_I2C_Init(g_config.i2c, TM_I2C_PinsPack_1, 100000);
	
	if(!TM_I2C_IsDeviceConnected(g_config.i2c, i2c_addr_am)) {
		my_printf("i2c accelerometer not responding\r\n");
		return;
	}
	if(!TM_I2C_IsDeviceConnected(g_config.i2c, i2c_addr_g)) {
		my_printf("i2c gyroscope not responding\r\n");
		return;
	}
	
	// Trigger perpipheral reset:
	{
		CTRL_REG0_XM_VALUE reboot = {};
		reboot.boot = 1;
		i2c_write_am(CTRL_REG0_XM, &reboot);
	}
	
	// Configure accelerometer:
	{
		CTRL_REG1_XM_VALUE reg1;
		reg1.rate = g_config.gRate;
		reg1.bdu = 1;
		reg1.azen = 1;
		reg1.ayen = 1;
		reg1.axen = 1;
		i2c_write_am(CTRL_REG1_XM, &reg1);
		
		CTRL_REG2_XM_VALUE reg2;
		reg2.abw = g_config.lAAFilterBW;
		reg2.afs = g_config.lMaxScale;
		reg2.ast = 0;
		reg2.sim = 0;
		i2c_write_am(CTRL_REG2_XM, &reg2);
	}
	
	// Configure magnetometer:
	{
		CTRL_REG5_XM_VALUE reg5;
		reg5.temp_en  = 0;
		reg5.m_res = g_config.mRes;
		reg5.m_odr = g_config.mODR;
		reg5.lir2 = 0;
		reg5.lir1 = 0;
		i2c_write_am(CTRL_REG5_XM, &reg5);
		
		CTRL_REG6_XM_VALUE reg6 = {};
		reg6.mfs = g_config.mFSR;
		i2c_write_am(CTRL_REG6_XM, &reg6);
		
		CTRL_REG7_XM_VALUE reg7 = {};
		reg7.ahpm = g_config.ahpm;
		reg7.afds = 0;
		reg7.mlp = 0;
		reg7.md = 0;
		i2c_write_am(CTRL_REG7_XM, &reg7);
	}
	
	// Configure gyro:
	{
		CTRL_REG1_G_VALUE reg1;
		reg1.dr_bw = g_config.dr_bw;
		reg1.pd = 1;
		reg1.x_en = 1;
		reg1.y_en = 1;
		reg1.z_en = 1;
		i2c_write_g(CTRL_REG1_G, &reg1);
		
		CTRL_REG2_G_VALUE reg2 = {};
		reg2.hpm = g_config.hpm;
		reg2.hpcf = g_config.hpcf;
		i2c_write_g(CTRL_REG2_G, &reg2);
	}
	
	// Interrupt handlers on our side
	{
		GPIO_InitTypeDef gpio;
		gpio.GPIO_Pin = g_config.INTG | g_config.DRDYG | g_config.INT1_XM | g_config.INT2_XM;
		gpio.GPIO_Mode = GPIO_Mode_IN;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init(g_config.GPIO_int, &gpio);
		my_printf("GPIOs configured\r\n");
		
		int intPins[] = {g_config.INTG, g_config.DRDYG, g_config.INT1_XM, g_config.INT2_XM};
		for(int i = 0; i < 4; i++) {
			// Lines are the same as the corresponding interrupt pins.  We need to enable all four.
			EXTI_InitTypeDef exti;
			exti.EXTI_Line = intPins[i];
			exti.EXTI_LineCmd = ENABLE;
			exti.EXTI_Mode = EXTI_Mode_Interrupt;
			exti.EXTI_Trigger = EXTI_Trigger_Rising;
			EXTI_Init(&exti);
			
			// Add IRQ vector to NVIC
			NVIC_InitTypeDef nvic;
			nvic.NVIC_IRQChannel = EXTI0_IRQn + i;
			nvic.NVIC_IRQChannelPreemptionPriority = 0x00;
			nvic.NVIC_IRQChannelSubPriority = 0x00;
			nvic.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&nvic);
		}
	}
	
	// Interrupt generation for accelerometer/magnetometer:
	{
		CTRL_REG0_XM_VALUE reg0;
		reg0.boot = 0;
		reg0.fifo_en = 1;
		reg0.wtm_en = 0;
		reg0.hp_click = 0;
		reg0.hpis1 = 0;
		reg0.hpis2 = 0;
		i2c_write_am(CTRL_REG0_XM, &reg0);
		
		CTRL_REG3_XM_VALUE reg3;
		reg3.p1_empty = 0;
		reg3.p1_drdyM = 0;
		reg3.p1_drdyA = 1;
		reg3.p1_intm = 0;
		reg3.p1_int2 = 0;
		reg3.p1_int1 = 0;
		reg3.p1_tap = 0;
		reg3.p1_boot = 0;
		i2c_write_am(CTRL_REG3_XM, &reg3);
		my_printf("ctrl_reg3_xm =\t%d\r\n", *(uint8_t*)&reg3);
		
		CTRL_REG4_XM_VALUE reg4;
		reg4.p2_wtm = 0;
		reg4.p2_overrun = 0;
		reg4.p2_drdyM = 0;
		reg4.p2_drdyA = 0;
		reg4.p2_intm = 0;
		reg4.p2_int2 = 0;
		reg4.p2_int1 = 0;
		reg4.p2_tap = 0;
		i2c_write_am(CTRL_REG4_XM, &reg4);
		my_printf("ctrl_reg4_xm =\t%d\r\n", *(uint8_t*)&reg4);
		
		INT_CTRL_REG_M_VALUE iCtrl;
		iCtrl.mien = 0;
		iCtrl._4d = 0;
		iCtrl.iel = 0;
		iCtrl.iea = 0;
		iCtrl.pp_od = 0;
		iCtrl.zmien = 0;
		iCtrl.ymien = 0;
		iCtrl.xmien = 0;
		i2c_write_am(INT_CTRL_REG_M, &iCtrl);
		my_printf("int_ctrl_reg =\t%d\r\n", *(uint8_t*)&iCtrl);
		
		FIFO_CTRL_REG_VALUE fifoCtrl;
		fifoCtrl.fth = 5;
		fifoCtrl.fm = eFIFOModeFIFO;
		i2c_write_am(FIFO_CTRL_REG, &fifoCtrl);
		
		/*INT_GEN_1_REG_VALUE intGen1;
		intGen1.aoi = 0;
		intGen1._6d = 0;
		intGen1.xhie = 1;
		intGen1.xlie = 1;
		intGen1.yhie = 1;
		intGen1.ylie = 1;
		intGen1.zhie = 1;
		intGen1.zlie = 1;
		i2c_write_am(INT_GEN_1_REG, &intGen1);
		
		INT_GEN_2_REG_VALUE intGen2;
		intGen2.aoi = 0;
		intGen2._6d = 0;
		intGen2.xhie = 1;
		intGen2.xlie = 1;
		intGen2.yhie = 1;
		intGen2.ylie = 1;
		intGen2.zhie = 1;
		intGen2.zlie = 1;
		i2c_write_am(INT_GEN_2_REG, &intGen2);*/
	}
}

lsm_ddx lsm_read_ddx(void) {
	lsm_ddx ddx = {};
	
	ddx.ddx =
		TM_I2C_Read(g_config.i2c, i2c_addr_am, OUT_X_L_A) |
		(TM_I2C_Read(g_config.i2c, i2c_addr_am, OUT_X_H_A) << 8);
	ddx.ddy =
		TM_I2C_Read(g_config.i2c, i2c_addr_am, OUT_Y_L_A) |
		(TM_I2C_Read(g_config.i2c, i2c_addr_am, OUT_Y_H_A) << 8);
	ddx.ddz =
		TM_I2C_Read(g_config.i2c, i2c_addr_am, OUT_Z_L_A) |
		(TM_I2C_Read(g_config.i2c, i2c_addr_am, OUT_Z_H_A) << 8);
	return ddx;
}

lsm_deuler lsm_read_deuler(void) {
	lsm_deuler deuler = {};
	
	deuler.dx =
		TM_I2C_Read(g_config.i2c, i2c_addr_g, OUT_X_L_G) |
		(TM_I2C_Read(g_config.i2c, i2c_addr_g, OUT_X_H_G) << 8);
	deuler.dy =
		TM_I2C_Read(g_config.i2c, i2c_addr_g, OUT_Y_L_G) |
		(TM_I2C_Read(g_config.i2c, i2c_addr_g, OUT_Y_H_G) << 8);
	deuler.dz =
		TM_I2C_Read(g_config.i2c, i2c_addr_g, OUT_Z_L_G) |
		(TM_I2C_Read(g_config.i2c, i2c_addr_g, OUT_Z_H_G) << 8);
	return deuler;
}

lsm_v lsm_read_compass(void) {
	lsm_v v = {};
	
	v.x =
		TM_I2C_Read(g_config.i2c, i2c_addr_am, OUT_X_L_M) |
		(TM_I2C_Read(g_config.i2c, i2c_addr_am, OUT_X_L_M) << 8);
	v.y =
		TM_I2C_Read(g_config.i2c, i2c_addr_am, OUT_Y_L_M) |
		(TM_I2C_Read(g_config.i2c, i2c_addr_am, OUT_Y_H_M) << 8);
	v.z =
		TM_I2C_Read(g_config.i2c, i2c_addr_am, OUT_Z_L_M) |
		(TM_I2C_Read(g_config.i2c, i2c_addr_am, OUT_Z_H_M) << 8);
	return v;
}

int lsm_handle_interrupt_INTG(void) {
	my_printf("Interrupt INTG\r\n");
	return 0;
}

int lsm_handle_interrupt_DRDYG(void) {
	my_printf("Interrupt DRDYG\r\n");
	return 0;
}

int lsm_handle_interrupt_INT1_XM(void) {
	// Interrupt indicates accelerometer data ready, process it
	STATUS_REG_A_VALUE val;
	while(i2c_read_am(STATUS_REG_A, &val), val.zyxada) {
		lsm_ddx ddv = {};
		uint8_t* data = (uint8_t*)&ddv;
		i2c_read_am(OUT_X_L_A, data + 0);
		i2c_read_am(OUT_X_H_A, data + 1);
		i2c_read_am(OUT_Y_L_A, data + 2);
		i2c_read_am(OUT_Y_H_A, data + 3);
		i2c_read_am(OUT_Z_L_A, data + 4);
		i2c_read_am(OUT_Z_H_A, data + 5);
		
		my_printf(
			"ddv = (%d, %d, %d)\r\n",
			(int)ddv.ddx,
			(int)ddv.ddy,
			(int)ddv.ddz
		);
	}
	
	return 0;
}

int lsm_handle_interrupt_INT2_XM(void) {
	my_printf("Interrupt INT2\r\n");
	return 0;
}

void EXTI0_IRQHandler(void)
{
	lsm_handle_interrupt_INTG();
}

void EXTI1_IRQHandler(void)
{
	lsm_handle_interrupt_DRDYG();
}

void EXTI2_IRQHandler(void)
{
	lsm_handle_interrupt_INT1_XM();
	EXTI_ClearITPendingBit(EXTI_Line0);
}

void EXTI3_IRQHandler(void)
{
	lsm_handle_interrupt_INT2_XM();
}
