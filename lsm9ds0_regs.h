#ifndef _LSM9DS0_REGS_H
#define _LSM9DS0_REGS_H

// Names of all available registers
typedef enum _eI2CAddr_AM {
	STATUS_REG_M = 0x07,
	OUT_X_L_M = 0x08,
	OUT_X_H_M = 0x09,
	OUT_Y_L_M = 0x0A,
	OUT_Y_H_M = 0x0B,
	OUT_Z_L_M = 0x0C,
	OUT_Z_H_M = 0x0D,
	
	INT_CTRL_REG_M = 0x12,
	INT_SRC_REG_M = 0x13,
	INT_THS_L_M = 0x14,
	INT_THS_H_M = 0x15,
	
	OFFSET_X_L_M = 0x16,
	OFFSET_X_H_M = 0x17,
	OFFSET_Y_L_M = 0x18,
	OFFSET_Y_H_M = 0x19,
	OFFSET_Z_L_M = 0x1A,
	OFFSET_Z_H_M = 0x1B,
	
	CTRL_REG0_XM = 0x1F,
	CTRL_REG1_XM = 0x20,
	CTRL_REG2_XM = 0x21,
	CTRL_REG3_XM = 0x22,
	CTRL_REG4_XM = 0x23,
	CTRL_REG5_XM = 0x24,
	CTRL_REG6_XM = 0x25,
	CTRL_REG7_XM = 0x26,
	
	STATUS_REG_A = 0x27,
	
	OUT_X_L_A = 0x28,
	OUT_X_H_A = 0x29,
	OUT_Y_L_A = 0x2A,
	OUT_Y_H_A = 0x2B,
	OUT_Z_L_A = 0x2C,
	OUT_Z_H_A = 0x2D,
	
	FIFO_CTRL_REG = 0x2E,
	FIFO_SRC_REG = 0x2F,
	INT_GEN_1_REG = 0x30,
	INT_GEN_1_SRC = 0x31,
	INT_GEN_1_THS = 0x32,
	INT_GEN_1_DURATION = 0x33,
	INT_GEN_2_REG = 0x34,
	INT_GEN_2_SRC = 0x35,
	INT_GEN_2_THS = 0x36,
	INT_GEN_2_DURATION = 0x37
} eI2CAddr_AM;

typedef struct _STATUS_REG_M_VALUE {
	unsigned xmda : 1;
	unsigned ymda : 1;
	unsigned zmda : 1;
	unsigned zyxmda : 1;
	unsigned xmor : 1;
	unsigned ymor : 1;
	unsigned zmor : 1;
	unsigned zyxmor : 1;
} STATUS_REG_M_VALUE;

typedef struct _INT_CTRL_REG_M_VALUE {
	unsigned char mien : 1;
	unsigned char _4d : 1;
	unsigned char iel : 1;
	unsigned char iea : 1;
	unsigned char pp_od : 1;
	unsigned char zmien : 1;
	unsigned char ymien : 1;
	unsigned char xmien : 1;
} INT_CTRL_REG_M_VALUE;

typedef struct _CTRL_REG0_XM_VALUE {
	
	unsigned char hpis2 : 1;
	unsigned char hpis1 : 1;
	unsigned char hp_click : 1;
	unsigned char : 2;
	unsigned char wtm_en : 1;
	unsigned char fifo_en : 1;
	unsigned char boot : 1;
} CTRL_REG0_XM_VALUE;

typedef struct _CTRL_REG1_XM_VALUE {
	unsigned char axen : 1;
	unsigned char ayen : 1;
	unsigned char azen : 1;
	unsigned char bdu : 1;
	unsigned char rate : 4;
} CTRL_REG1_XM_VALUE;

typedef struct _CTRL_REG2_XM_VALUE {
	unsigned char sim : 1;
	unsigned char ast : 2;
	unsigned char afs : 3;
	unsigned char abw : 2;
} CTRL_REG2_XM_VALUE;

typedef struct _CTRL_REG3_XM_VALUE {
	unsigned char p1_empty : 1;
	unsigned char p1_drdyM : 1;
	unsigned char p1_drdyA : 1;
	unsigned char p1_intm : 1;
	unsigned char p1_int2 : 1;
	unsigned char p1_int1 : 1;
	unsigned char p1_tap : 1;
	unsigned char p1_boot : 1;
} CTRL_REG3_XM_VALUE;

typedef struct _CTRL_REG4_XM_VALUE {
	unsigned char p2_wtm : 1;
	unsigned char p2_overrun : 1;
	unsigned char p2_drdyM : 1;
	unsigned char p2_drdyA : 1;
	
	unsigned char p2_intm : 1;
	unsigned char p2_int2 : 1;
	unsigned char p2_int1 : 1;
	unsigned char p2_tap : 1;
} CTRL_REG4_XM_VALUE;

typedef struct _CTRL_REG5_XM_VALUE {
	unsigned char lir1 : 1;
	unsigned char lir2 : 1;
	unsigned char m_odr : 3;
	unsigned char m_res : 2;
	unsigned char temp_en : 1;
} CTRL_REG5_XM_VALUE;

typedef struct _CTRL_REG6_XM_VALUE {
	unsigned char : 5;
	unsigned char mfs : 2;
	unsigned char : 1;
} CTRL_REG6_XM_VALUE;

typedef struct _CTRL_REG7_XM_VALUE {
	unsigned char md : 2;
	unsigned char mlp : 1;
	unsigned char : 2;
	unsigned char afds : 1;
	unsigned char ahpm : 2;
} CTRL_REG7_XM_VALUE;

typedef struct _STATUS_REG_A_VALUE {
	unsigned xada : 1;
	unsigned yada : 1;
	unsigned zada : 1;
	unsigned zyxada : 1;
	unsigned xaor : 1;
	unsigned yaor : 1;
	unsigned zaor : 1;
	unsigned zyxaor : 1;
} STATUS_REG_A_VALUE;

typedef struct _FIFO_CTRL_REG_VALUE {
	unsigned char fth : 5;
	unsigned char fm : 3;
} FIFO_CTRL_REG_VALUE;

typedef struct _FIFO_SRC_REG_VALUE {
	unsigned char ffs : 5;
	unsigned char empty : 1;
	unsigned char ovrn : 1;
	unsigned char wtm : 1;
} FIFO_SRC_REG_VALUE;

typedef struct _INT_GEN_1_REG_VALUE {
	unsigned char xlie : 1;
	unsigned char xhie : 1;
	unsigned char ylie : 1;
	unsigned char yhie : 1;
	unsigned char zlie : 1;
	unsigned char zhie : 1;
	unsigned char _6d : 1;
	unsigned char aoi : 1;
} INT_GEN_1_REG_VALUE;

typedef struct _INT_GEN_1_SRC_VALUE {
	unsigned char xl : 1;
	unsigned char xh : 1;
	unsigned char yl : 1;
	unsigned char yh : 1;
	unsigned char zl : 1;
	unsigned char zh : 1;
	unsigned char ia : 1;
	unsigned char : 1;
} INT_GEN_1_SRC_VALUE;

typedef struct _INT_GEN_1_THS_VALUE {
	unsigned char THS : 7;
	unsigned char : 1;
} INT_GEN_1_THS_VALUE;

typedef struct _INT_GEN_1_DURATION_VALUE {
	unsigned char D : 7;
	unsigned char : 1;
} INT_GEN_1_DURATION_VALUE;

// Interrupt 2 structures have the same layout as their interrupt 1
// counterparts
typedef INT_GEN_1_REG_VALUE INT_GEN_2_REG_VALUE;
typedef INT_GEN_1_SRC_VALUE INT_GEN_2_SRC_VALUE;
typedef INT_GEN_1_THS_VALUE INT_GEN_2_THS_VALUE;
typedef INT_GEN_1_DURATION_VALUE INT_GEN_2_DURATION_VALUE;

typedef enum _eI2CAddr_G {
	CTRL_REG1_G = 0x20,
	CTRL_REG2_G = 0x21,
	CTRL_REG3_G = 0x22,
	CTRL_REG4_G = 0x23,
	CTRL_REG5_G = 0x24,
	
	STATUS_REG_G = 0x27,
	
	OUT_X_L_G = 0x28,
	OUT_X_H_G = 0x29,
	OUT_Y_L_G = 0x2A,
	OUT_Y_H_G = 0x2B,
	OUT_Z_L_G = 0x2C,
	OUT_Z_H_G = 0x2D,
	
	FIFO_CTRL_REG_G = 0x2E
} eI2CAddr_G;

typedef struct _CTRL_REG1_G_VALUE {
	unsigned char x_en : 1;
	unsigned char y_en : 1;
	unsigned char z_en : 1;
	unsigned char pd : 1;
	unsigned char dr_bw : 4;
} CTRL_REG1_G_VALUE;

typedef struct _CTRL_REG2_G_VALUE {
	unsigned char hpcf : 4;
	unsigned char hpm : 2;
	unsigned char : 2;
} CTRL_REG2_G_VALUE;

typedef struct _CTRL_REG5_G_VALUE {
	unsigned char out_sel : 2;
	unsigned char int_sel : 2;
	unsigned char hpen : 1;
	unsigned char : 1;
	unsigned char fifo_en : 1;
	unsigned char boot : 1;
} CTRL_REG5_G_VALUE;

typedef struct _STATUS_REG_G_VALUE {
	unsigned char zyxor : 1;
	unsigned char zor : 1;
	unsigned char yor : 1;
	unsigned char xor : 1;
	unsigned char zyxda : 1;
	unsigned char zda : 1;
	unsigned char yda : 1;
	unsigned char xda : 1;
} STATUS_REG_G_VALUE;

#endif
