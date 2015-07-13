#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "lsm9ds0.h"
#include "tm_stm32f4_i2c.h"
#include "my_printf.h"
#include "lsm9ds0_regs.h"
#include "task.h"
#include <stdlib.h>

extern void delay_ms(uint32_t t);

static LSM9DS0_CONFIG g_config;

// Dispatch table for interrupt lines:
static void (*pfnDispatch[4])(void*, void*) = {};

// Seven-bit slave addresses, shifted left one bit already
static uint32_t i2c_addr_am = 0x3A;
static uint32_t i2c_addr_g = 0xD6;

static uint8_t i2c_read_am_b(eI2CAddr_AM reg) { return TM_I2C_Read(g_config.i2c, i2c_addr_am, reg); }
static void i2c_read_am(eI2CAddr_AM reg, void* value) { *(uint8_t*)value = i2c_read_am_b(reg); }
static void i2c_read_am_multi(eI2CAddr_G reg, void* data, uint16_t count) { TM_I2C_ReadMulti(g_config.i2c, i2c_addr_am, 0x80 | reg, data, count); }
static void i2c_write_am(eI2CAddr_AM reg, const void* value) { TM_I2C_Write(g_config.i2c, i2c_addr_am, reg, *(uint8_t*)value); }

static uint8_t i2c_read_g_b(eI2CAddr_G reg) { return TM_I2C_Read(g_config.i2c, i2c_addr_g, reg); }
static void i2c_read_g(eI2CAddr_G reg, void* value) { *(uint8_t*)value = i2c_read_g_b(reg); }
static void i2c_read_g_multi(eI2CAddr_G reg, void* data, uint16_t count) { TM_I2C_ReadMulti(g_config.i2c, i2c_addr_g, 0x80 | reg, data, count); }
static void i2c_write_g(eI2CAddr_G reg, const void* value) { TM_I2C_Write(g_config.i2c, i2c_addr_g, reg, *(uint8_t*)value); }

static void lsm_handle_interrupt_INTG(void* arg1, void* arg2);
static void lsm_handle_interrupt_DRDYG(void* arg1, void* arg2);
static void lsm_handle_interrupt_INT1_XM(void* arg1, void* arg2);
static void lsm_handle_interrupt_INT2_XM(void* arg1, void* arg2);

uint8_t lsm_init(const LSM9DS0_CONFIG* config) {
	// Copy over configuration block:
	g_config = *config;

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

	// Delay to give the accelerometer enough time to power on
	delay_ms(250);
    
	// WHOAMI validation:
	if(i2c_read_am_b(WHO_AM_I_XM) != 0x49)
		my_printf("i2c accelerometer not responding\r\n");
	if(i2c_read_g_b(WHO_AM_I_G) != 0xD4)
		my_printf("i2c gyro not responding\r\n");
	
	// Interrupt handlers on our side
	{
		// Build up the dispatch table:
		int intPins[] = {g_config.INTG, g_config.DRDYG, g_config.INT1_XM, g_config.INT2_XM};
		void (*dispatchTab[])(void*, void*) = {
			lsm_handle_interrupt_INTG,
			lsm_handle_interrupt_DRDYG,
			lsm_handle_interrupt_INT1_XM,
			lsm_handle_interrupt_INT2_XM
		};
		for(int i = 0; i < 4; i++) {
			// We don't support interrupt pins other than 1-4
			if(!(GPIO_Pin_1 <= intPins[i] && intPins[i] <= GPIO_Pin_4)) {
				my_printf("An interrupt GPIO was requested that is out of bounds\r\n");
				return 1;
			}
			
			// We just enabled interrupt irqn, and the dispatcher is intDispatch[i]
			int irqn = (31 - __CLZ(intPins[i])) - 1;
			pfnDispatch[irqn] = dispatchTab[i];
		}
		
		// Enable all four NVIC and EXTI handlers.  Do not collapse this loop with the prior one,
		// the dispatch table has to be configured first before any interrupt can be correctly
		// handled.
		for(int i = 0; i < 4; i++) {
			EXTI_InitTypeDef exti;
			exti.EXTI_Line = EXTI_Line1 << i;
			exti.EXTI_LineCmd = ENABLE;
			exti.EXTI_Mode = EXTI_Mode_Interrupt;
			exti.EXTI_Trigger = EXTI_Trigger_Rising;
			EXTI_Init(&exti);
			
			NVIC_InitTypeDef nvic;
			nvic.NVIC_IRQChannel = EXTI1_IRQn + i;
			nvic.NVIC_IRQChannelPreemptionPriority = 0x00;
			nvic.NVIC_IRQChannelSubPriority = 0x00;
			nvic.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&nvic);
		}
		
		// We know for sure that the required GPIOs are pins 1~4
		GPIO_InitTypeDef gpio;
		gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
		gpio.GPIO_Mode = GPIO_Mode_IN;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init(g_config.GPIO_int, &gpio);
		my_printf("GPIOs configured\r\n");
		
		// Verify that we saturated all four dispatchers:
		for(int i = 0; i < 4; i++)
			if(!pfnDispatch[i]) {
				my_printf("Dispatch entry %d was not specified\r\n", i);
				return 2;
			}
	}
	
	// Trigger perpipheral reset:
	{
		CTRL_REG0_XM_VALUE rebootAM = {};
		rebootAM.boot = 1;
		i2c_write_am(CTRL_REG0_XM, &rebootAM);
		CTRL_REG5_G_VALUE rebootG = {};
		rebootG.boot = 1;
		i2c_write_g(CTRL_REG5_G, &rebootG);

		// Delay to give the accelerometer enough time to reset
		delay_ms(250);
		
		rebootAM.boot = 0;
		i2c_write_am(CTRL_REG0_XM, &rebootAM);
		rebootG.boot = 0;
		i2c_write_g(CTRL_REG5_G, &rebootG);
		delay_ms(250);
	}
	
	// Interrupt generation for accelerometer/magnetometer:
	{
		CTRL_REG0_XM_VALUE reg0 = {};
		reg0.boot = 0;
		reg0.fifo_en = 0;
		reg0.wtm_en = 1;
		reg0.hp_click = 0;
		reg0.hpis1 = 0;
		reg0.hpis2 = 0;
		i2c_write_am(CTRL_REG0_XM, &reg0);
		
		CTRL_REG3_XM_VALUE reg3 = {};
		reg3.p1_empty = 0;
		reg3.p1_drdyM = 0;
		reg3.p1_drdyA = 0;
		reg3.p1_intm = 0;
		reg3.p1_int2 = 0;
		reg3.p1_int1 = 0;
		reg3.p1_tap = 0;
		reg3.p1_boot = 0;
		i2c_write_am(CTRL_REG3_XM, &reg3);
		my_printf("ctrl_reg3_xm =\t%x\r\n", *(uint8_t*)&reg3);
		
		CTRL_REG4_XM_VALUE reg4 = {};
		reg4.p2_wtm = 1;
		reg4.p2_overrun = 0;
		reg4.p2_drdyM = 1;
		reg4.p2_drdyA = 1;
		reg4.p2_intm = 0;
		reg4.p2_int2 = 0;
		reg4.p2_int1 = 0;
		reg4.p2_tap = 0;
		i2c_write_am(CTRL_REG4_XM, &reg4);
		my_printf("ctrl_reg4_xm =\t%d\r\n", *(uint8_t*)&reg4);
		
		INT_CTRL_REG_M_VALUE iCtrl = {};
		iCtrl.mien = 0;
		iCtrl._4d = 0;
		iCtrl.iel = 0;
		iCtrl.iea = 1;
		iCtrl.pp_od = 0;
		iCtrl.zmien = 0;
		iCtrl.ymien = 0;
		iCtrl.xmien = 0;
		i2c_write_am(INT_CTRL_REG_M, &iCtrl);
		my_printf("ctrl_reg_m =\t%x\r\n", *(uint8_t*)&iCtrl);
		
		FIFO_CTRL_REG_VALUE fifoCtrl = {};
		fifoCtrl.fm = eFIFOModeStream;
		fifoCtrl.fth = 5;
		i2c_write_am(FIFO_CTRL_REG, &fifoCtrl);
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
		FIFO_CTRL_REG_G_VALUE fifo_ctl = {};
		fifo_ctl.fm = eFIFOModeStream;
		fifo_ctl.wtm = 5;
		i2c_write_g(FIFO_CTRL_REG_G, &fifo_ctl);
		my_printf("fifo_ctl_g =\t%x\r\n", *(uint8_t*)&fifo_ctl);
		
		CTRL_REG1_G_VALUE reg1;
		reg1.dr_bw = g_config.dr_bw;
		reg1.pd = 1;
		reg1.x_en = 1;
		reg1.y_en = 1;
		reg1.z_en = 1;
		i2c_write_g(CTRL_REG1_G, &reg1);
		
		CTRL_REG2_G_VALUE reg2 = {0};
		reg2.hpm = g_config.hpm;
		reg2.hpcf = g_config.hpcf;
		i2c_write_g(CTRL_REG2_G, &reg2);
		
		CTRL_REG3_G_VALUE reg3 = {};
		reg3.i1_int1 = 0;
		reg3.i1_boot = 0;
		reg3.h_lactive = 1;
		reg3.pp_od = 0;
		reg3.i2_drdy = 1;
		reg3.i2_wtm = 1;
		reg3.i2_orun  = 0;
		reg3.i2_empty = 0;
		i2c_write_g(CTRL_REG3_G, &reg3);
		
		CTRL_REG5_G_VALUE reg5 = {};
		reg5.fifo_en = 1;
		i2c_write_g(CTRL_REG5_G, &reg5);
	}
	
	return 0;
}

lsm_ddv lsm_read_ddv(void) {
	lsm_ddv ddv = {};
	i2c_read_am_multi(OUT_X_L_A, &ddv, sizeof(ddv));
	return ddv;
}

lsm_deuler lsm_read_deuler(void) {
	lsm_deuler euler;
	euler.dx = i2c_read_g_b(OUT_X_L_G) | (i2c_read_g_b(OUT_X_H_G) << 8);
	euler.dy = i2c_read_g_b(OUT_Y_L_G) | (i2c_read_g_b(OUT_Y_H_G) << 8);
	euler.dz = i2c_read_g_b(OUT_Z_L_G) | (i2c_read_g_b(OUT_Z_H_G) << 8);
	return euler;
}

lsm_v lsm_read_compass(void) {
	lsm_v v;
	i2c_read_am_multi(OUT_X_L_M, &v, sizeof(v));
	return v;
}

static void reverse_three(void* pThree) {
	uint16_t* pWord = (uint16_t*)pThree;
	pWord[0] = __REV16(pWord[0]);
	pWord[1] = __REV16(pWord[1]);
	pWord[2] = __REV16(pWord[2]);
}

void lsm_handle_interrupt_INTG(void* arg1, void* arg2) {
	my_printf("Interrupt INTG\r\n");
}

void lsm_handle_interrupt_DRDYG(void* arg1, void* arg2) {
	// Interrupt indicates accelerometer data ready, process it, data payloads
	// follow immediately (and conveniently) right after the status register
	for(
		struct {
			STATUS_REG_G_VALUE valG;
			lsm_deuler dv;
		} c;
		i2c_read_g_multi(STATUS_REG_G, &c, sizeof(c)), c.valG.zyxda;
	) {
		reverse_three(&c.dv);
		g_config.pfnG(&c.dv);
	}
}

void lsm_handle_interrupt_INT1_XM(void* arg1, void* arg2) {
	my_printf("Interrupt INT1\r\n");
}

void lsm_handle_interrupt_INT2_XM(void* arg1, void* arg2) {
	// Read FIFO status, decide how much to pull in
	
	// Interrupt indicates accelerometer data ready, process it, data payloads
	// follow immediately (and conveniently) right after the status register
	for(
		struct {
			STATUS_REG_A_VALUE valA;
			lsm_ddv ddv;
		} c;
		i2c_read_am_multi(STATUS_REG_A, &c, sizeof(c)), c.valA.zyxada;
	) {
		reverse_three(&c.ddv);
		g_config.pfnA(&c.ddv);
	}
	
	// Same situation here, status register followed immediately by the payload
	for(
		struct {
			STATUS_REG_M_VALUE valM;
			lsm_v v;
		} c = {};
		i2c_read_am_multi(STATUS_REG_M, &c, sizeof(c)),
		c.valM.zyxmda;
	) {
		reverse_three(&c.v);
		g_config.pfnM(&c.v);
	}
}

void EXTI1_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line1) != RESET)
		if(!task_add(pfnDispatch[0], NULL, NULL))
			my_printf("Overrun\r\n");
	EXTI_ClearITPendingBit(EXTI_Line1);
}

void EXTI2_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line2) != RESET)
		task_add(pfnDispatch[1], NULL, NULL);
	EXTI_ClearITPendingBit(EXTI_Line2);
}

void EXTI3_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line3) != RESET)
		task_add(pfnDispatch[2], NULL, NULL);
	EXTI_ClearITPendingBit(EXTI_Line3);
}

void EXTI4_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line4) != RESET)
		task_add(pfnDispatch[3], NULL, NULL);
	EXTI_ClearITPendingBit(EXTI_Line4);
}
