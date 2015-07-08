#pragma once

typedef enum _eSampleRate {
	eRate_Off = 0,
	eRate_3_125Hz = 1,
	eRate_6_25Hz = 2,
	eRate_12_5Hz = 3,
	eRate_25Hz = 4,
	eRate_50_Hz = 5,
	eRate_100_Hz = 6,
	eRate_200_Hz = 7,
	eRate_400Hz = 8,
	eRate_800Hz = 9,
	eRate_1600Hz = 10
} eSampleRate;

typedef enum _eAAFilterBW {
	eAAFilterBW_773Hz = 0,
	eAAFilterBW_194Hz = 1,
	eAAFilterBW_362Hz = 2,
	eAAFilterBW_50Hz = 3
} eAAFilterBW;

typedef enum _eMaxScale {
	eMaxScale_2g,
	eMaxScale_4g,
	eMaxScale_6g,
	eMaxScale_8g,
	eMaxScale_16g
} eMaxScale;

typedef enum _eGDRBW {
	eGDRBW_95Hz_1_25 = 0,
	eGDRBW_95Hz_25_A = 1,
	eGDRBW_95Hz_25_B = 2,
	eGDRBW_95Hz_25_C = 3,
	eGDRBW_190_12_5 = 4,
	eGDRBW_190_25 = 5,
	eGDRBW_190_50 = 6,
	eGDRBW_190_70 = 7,
	eGDRBW_380_20 = 8,
	eGDRBW_380_25 = 9,
	eGDRBW_380_50 = 10,
	eGDRBW_380_100 = 11,
	eGDRBW_760_30 = 12,
	eGDRBW_760_35 = 13,
	eGDRBW_760_50 = 14,
	eGDRBW_760_100 = 15
} eGDRBW;

typedef enum _eHPFM {
	eHPFM_NormalReset = 0,
	eHPFM_Reference = 1,
	eHPFM_Normal = 2,
	eHPFM_AutoReset = 3
} eHPFM;

typedef struct LSM9DS0_CONFIG {
	// Interrupt block and lines.
	GPIO_TypeDef* GPIO_int;
	uint32_t INTG;
	uint32_t DRDYG;
	uint32_t INT1_XM;
	uint32_t INT2_XM;
	
	// Chip select/enable lines:
	GPIO_TypeDef* GPIO_en;
	uint32_t CS_G;
	uint32_t CS_XM;
	uint32_t SDOG;
	uint32_t SDOXM;
	uint32_t DEN_G;
	
	// i2c block addresses:
	I2C_TypeDef* i2c;

	// Accelerometer traits:
	eSampleRate gRate;
	eAAFilterBW lAAFilterBW;
	eMaxScale lMaxScale;
	
	// Gyro traits:
	eGDRBW dr_bw;
	eHPFM hpm;
	uint8_t hpcf; // Cutoff frequency, table 26
	
} LSM9DS0_CONFIG;

// @summary Initializes behavior based on the specified GPIO interrupt
// @param config The pin configuration block
// @remarks The caller is responsible for enabling the clock block
void lsm_init(const LSM9DS0_CONFIG* config);

// @summary Reads linear accelerometer information
typedef struct _lsm_ddx {
	int16_t ddx;
	int16_t ddy;
	int16_t ddz;
} lsm_ddx;
lsm_ddx lsm_read_ddx(void);

typedef struct _lsm_deuler {
	int16_t dx;
	int16_t dy;
	int16_t dz;
} lsm_deuler;
lsm_deuler lsm_read_deuler(void);

// @summary Callback to be invoked when the line interrupt has been asserted
// @returns Nonzero to indicate that an interrupt was processed
int lsm_handle_interrupt(void);


