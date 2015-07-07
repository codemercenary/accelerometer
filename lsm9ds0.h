#pragma once

typedef struct LSM9DS0_CONFIG {
	// Interrupt line addresses
	int INT1_XM;
	int INT2_XM;
} LSM9DS0_CONFIG;

// @summary Initializes behavior based on the specified GPIO interrupt
// @param config The pin configuration block
void lsm_init(const LSM9DS0_CONFIG* config);

// @summary Callback to be invoked when the line interrupt has been asserted
// @returns Nonzero to indicate that an interrupt was processed
int lsm_handle_interrupt(void);


