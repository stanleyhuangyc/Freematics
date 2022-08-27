#ifndef _ICM_42627_H
#define _ICM_42627_H


#define ICM_42627_DeviceID			0x20

// PWR_MGMT0
#define GYRO_MODE_OFF						0<<2
#define GYRO_MODE_LN						3<<2
#define GYRO_MODE_Standby				1<<2

#define ACCEL_MODE_OFF					1<<0
#define ACCEL_MODE_LP						2<<0		//LOW POWER
#define ACCEL_MODE_LN						3<<0		//LOW Noise

#define IDLE_OFF								1<<4
#define IDLE_ON								  0<<4

#define TEMP_DIS_ON							0<<5
#define TEMP_DIS_OFF						1<<5

//GYRO_CONFIG0
#define GYRO_ODR_1KHZ						6<0
#define GYRO_FS_SEL_250dps			3<<5

//CONFIG0_REG
#define ACCEL_ODR_1KHZ					6<<0
#define ACCEL_FS_SEL_2G					3<<5

//INT_STATUS
#define UI_FSYNC_INT						1<<6
#define PLL_RDY_INT							1<<5
#define RESET_DONE_INT					1<<4
#define DATA_RDY_INT						1<<3
#define FIFO_THS_INT						1<<2
#define FIFO_FULL_INT						1<<1
#define AGC_RDY_INT							1<<0

#define ICM_42627_ADDRESS 0x68 //I2C Address
#define WHO_AM_I_ICM42627 0x75 // Should return 0x20

//ICM_42627 REG
#define PWR_MGMT0_REG						0x4E
#define GYRO_CONFIG0_REG				0x4F
#define ACCEL_CONFIG0_REG				0x50
#define SELF_TEST_CONFIG_REG		0x70
#define INT_STATUS_REG					0x2D

#define TEMP_OUT_H_REG       		0x1D
#define TEMP_OUT_L_REG       		0x1E

#define ACCEL_XOUT_H_REG    	 	0x1F
#define ACCEL_XOUT_L_REG     		0x20
#define ACCEL_YOUT_H_REG     		0x21
#define ACCEL_YOUT_L_REG     		0x22
#define ACCEL_ZOUT_H_REG     		0x23
#define ACCEL_ZOUT_L_REG     		0x24

#define GYRO_XOUT_H_REG      		0x25
#define GYRO_XOUT_L_REG      		0x26
#define GYRO_YOUT_H_REG      		0x27
#define GYRO_YOUT_L_REG      		0x28
#define GYRO_ZOUT_H_REG      		0x29
#define GYRO_ZOUT_L_REG      		0x30

#endif