#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "lsm9ds0.h"
#include "tm_stm32f4_i2c.h"
#include "my_printf.h"

static LSM9DS0_CONFIG g_config;

// Seven-bit slave addresses, shifted left one bit already
static uint32_t i2c_addr_am = 0x3A;
static uint32_t i2c_addr_g = 0xD6;

// Names of all available registers
enum {
	OUT_X_L_M = 0x08,
	OUT_X_H_M = 0x09,
	OUT_Y_L_M = 0x0A,
	OUT_Y_H_M = 0x0B,
	OUT_Z_L_M = 0x0C,
	OUT_Z_H_M = 0x0D,
	
	CTRL_REG0_XM = 0x1F,
	CTRL_REG1_XM = 0x20,
	CTRL_REG2_XM = 0x21,
	CTRL_REG3_XM = 0x22,
	CTRL_REG4_XM = 0x23,
	
	OUT_X_L_A = 0x28,
	OUT_X_H_A = 0x29,
	OUT_Y_L_A = 0x3A,
	OUT_Y_H_A = 0x2B,
	OUT_Z_L_A = 0x2C,
	OUT_Z_H_A = 0x2D,
};

typedef struct _CTRL_REG0_XM_VALUE {
	unsigned char boot : 1;
	unsigned char fifo_en : 1;
	unsigned char wtm_en : 1;
	unsigned char : 2;
	unsigned char hp_click : 1;
	unsigned char hpis1 : 1;
	unsigned char hpis2 : 1;
} CTRL_REG0_XM_VALUE;

typedef struct _CTRL_REG1_XM_VALUE {
	unsigned char rate : 4;
	unsigned char bdu : 1;
	unsigned char azen : 1;
	unsigned char ayen : 1;
	unsigned char axen : 1;
} CTRL_REG1_XM_VALUE;

typedef struct _CTRL_REG2_XM_VALUE {
	unsigned char abw : 2;
	unsigned char afs : 3;
	unsigned char ast : 2;
	unsigned char sim : 1;
} CTRL_REG2_XM_VALUE;

typedef struct _CTRL_REG3_XM_VALUE {
	unsigned char p1_boot : 1;
	unsigned char p1_tap : 1;
	unsigned char p1_int1 : 1;
	unsigned char p1_int2 : 1;
	unsigned char p1_intm : 1;
	unsigned char p1_drdyA : 1;
	unsigned char p1_drdyM : 1;
	unsigned char p1_empty : 1;
} CTRL_REG3_XM_VALUE;

typedef struct _CTRL_REG4_XM_VALUE {
	unsigned char p2_tap : 1;
	unsigned char p2_int1 : 1;
	unsigned char p2_int2 : 1;
	unsigned char p2_intm : 1;
	unsigned char p2_drdyA : 1;
	unsigned char p2_drdyM : 1;
	unsigned char p2_overrun : 1;
	unsigned char p2_wtm : 1;
} CTRL_REG4_XM_VALUE;

enum {
	CTRL_REG1_G = 0x20,
	CTRL_REG2_G = 0x21,
	CTRL_REG3_G = 0x22,
	CTRL_REG4_G = 0x23,
	
	OUT_X_L_G = 0x28,
	OUT_X_H_G = 0x29,
	OUT_Y_L_G = 0x3A,
	OUT_Y_H_G = 0x2B,
	OUT_Z_L_G = 0x2C,
	OUT_Z_H_G = 0x2D
};

typedef struct _CTRL_REG1_G_VALUE {
	unsigned char dr_bw : 4;
	unsigned char pd : 1;
	unsigned char z_en : 1;
	unsigned char y_en : 1;
	unsigned char x_en : 1;
} CTRL_REG1_G_VALUE;

typedef struct _CTRL_REG2_G_VALUE {
	unsigned char : 2;
	unsigned char hpm : 2;
	unsigned char hpcf : 4;
} CTRL_REG2_G_VALUE;

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

int lsm_handle_interrupt(void) {
	return 0;
}
