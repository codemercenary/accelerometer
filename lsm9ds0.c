#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "lsm9ds0.h"
#include "tm_stm32f4_i2c.h"
#include "my_printf.h"

static LSM9DS0_CONFIG g_config;

// Seven-bit slave addresses, shifted left one bit already
static uint32_t i2c_addr_am = 0x3A;
static uint32_t i2c_addr_g = 0xD6;

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
	
	// Configure accelerometer:
	{
		CTRL_REG1_XM_VALUE reg1;
		reg1.rate = g_config.gRate;
		reg1.bdu = 1;
		reg1.azen = 1;
		reg1.ayen = 1;
		reg1.axen = 1;
		TM_I2C_Write(g_config.i2c, i2c_addr_am, CTRL_REG1_XM, *(uint8_t*)&reg1);
		
		CTRL_REG2_XM_VALUE reg2;
		reg2.abw = g_config.lAAFilterBW;
		reg2.afs = g_config.lMaxScale;
		reg2.ast = 0;
		reg2.sim = 0;
		TM_I2C_Write(g_config.i2c, i2c_addr_am, CTRL_REG2_XM, *(uint8_t*)&reg2);
	}
	
	// Configure magnetometer:
	{
		CTRL_REG5_XM_VALUE reg5;
		reg5.temp_en  = 0;
		reg5.m_res = g_config.mRes;
		reg5.m_odr = g_config.mODR;
		reg5.lir2 = 0;
		reg5.lir1 = 0;
		//TM_I2C_Write(g_config.i2c, i2c_addr_am, CTRL_REG5_XM, *(uint8_t*)&reg5);
		TM_I2C_Write(g_config.i2c, i2c_addr_am, CTRL_REG5_XM, *(uint8_t*)&reg5);
		
		CTRL_REG6_XM_VALUE reg6 = {};
		reg6.mfs = g_config.mFSR;
		TM_I2C_Write(g_config.i2c, i2c_addr_am, CTRL_REG6_XM, *(uint8_t*)&reg6);
		
		CTRL_REG7_XM_VALUE reg7 = {};
		reg7.ahpm = g_config.ahpm;
		reg7.afds = 0;
		reg7.mlp = 0;
		reg7.md = 0;
		TM_I2C_Write(g_config.i2c, i2c_addr_am, CTRL_REG7_XM, *(uint8_t*)&reg7);
	}
	
	// Configure gyro:
	{
		CTRL_REG1_G_VALUE reg1;
		reg1.dr_bw = g_config.dr_bw;
		reg1.pd = 1;
		reg1.x_en = 1;
		reg1.y_en = 1;
		reg1.z_en = 1;
		TM_I2C_Write(g_config.i2c, i2c_addr_g, CTRL_REG1_G, *(uint8_t*)&reg1);
		
		CTRL_REG2_G_VALUE reg2 = {};
		reg2.hpm = g_config.hpm;
		reg2.hpcf = g_config.hpcf;
		TM_I2C_Write(g_config.i2c, i2c_addr_g, CTRL_REG2_G, *(uint8_t*)&reg2);
	}
	
	// Interrupt configuration:
	{
		INT_GEN_1_REG_VALUE intGen;
		intGen.AOI = 0;
		intGen._6D = 0;
		intGen.XHIE = 1;
		intGen.XLIE = 1;
		intGen.YHIE = 1;
		intGen.YLIE = 1;
		intGen.ZHIE = 1;
		intGen.ZLIE = 1;
		TM_I2C_Write(g_config.i2c, i2c_addr_g, INT_GEN_1_REG, *(uint8_t*)&intGen);
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

int lsm_handle_interrupt(void) {
	my_printf("Interrupt\r\n");
	return 0;
}
