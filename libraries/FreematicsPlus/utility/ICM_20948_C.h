/*

This is a C-compatible interface to the features presented by the ICM 20948 9-axis device
The imementation of the interface is flexible

*/ 


#ifndef _ICM_20948_C_H_
#define _ICM_20948_C_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "ICM_20948_REGISTERS.h"
#include "ICM_20948_ENUMERATIONS.h"	// This is to give users access to usable value definiitons
#include "AK09916_ENUMERATIONS.h"

#ifdef __cplusplus
extern "C" {
#endif	/* __cplusplus */

#define ICM_20948_I2C_ADDR_AD0 	0x68 // Or 0x69 when AD0 is high
#define ICM_20948_I2C_ADDR_AD1	0x69 //
#define ICM_20948_WHOAMI		0xEA

#define MAG_AK09916_I2C_ADDR 	0x0C
#define MAG_AK09916_WHO_AM_I	0x4809
#define MAG_REG_WHO_AM_I		0x00

typedef enum{
	ICM_20948_Stat_Ok = 0x00,	// The only return code that means all is well
	ICM_20948_Stat_Err,			// A general error
	ICM_20948_Stat_NotImpl,		// Returned by virtual functions that are not implemented
	ICM_20948_Stat_ParamErr,
	ICM_20948_Stat_WrongID,
	ICM_20948_Stat_InvalSensor,	// Tried to apply a function to a sensor that does not support it (e.g. DLPF to the temperature sensor)
	ICM_20948_Stat_NoData,
	ICM_20948_Stat_SensorNotSupported,

	ICM_20948_Stat_NUM,
	ICM_20948_Stat_Unknown,
}ICM_20948_Status_e;

typedef enum{
	ICM_20948_Internal_Acc = (1 << 0),
	ICM_20948_Internal_Gyr = (1 << 1),
	ICM_20948_Internal_Mag = (1 << 2),
	ICM_20948_Internal_Tmp = (1 << 3),
	ICM_20948_Internal_Mst = (1 << 4), 		// I2C Master Ineternal
}ICM_20948_InternalSensorID_bm;			// A bitmask of internal sensor IDs

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
}ICM_20948_axis3bit16_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
}ICM_20948_axis1bit16_t;

typedef struct{								
	uint8_t				a			: 2;
	uint8_t 			g			: 2;
	uint8_t 			reserved_0	: 4;
}ICM_20948_fss_t;							// Holds full-scale settings to be able to extract measurements with units

typedef struct{								
	uint8_t	a;
	uint8_t	g;
}ICM_20948_dlpcfg_t;						// Holds digital low pass filter settings. Members are type ICM_20948_ACCEL_CONFIG_DLPCFG_e

typedef struct{
	uint16_t a;
	uint8_t g;
}ICM_20948_smplrt_t;

typedef struct{
	uint8_t		I2C_MST_INT_EN		: 1;
	uint8_t 	DMP_INT1_EN			: 1;
	uint8_t		PLL_RDY_EN			: 1;
	uint8_t 	WOM_INT_EN			: 1;
	uint8_t		REG_WOF_EN			: 1;
	uint8_t		RAW_DATA_0_RDY_EN	: 1;
	uint8_t		FIFO_OVERFLOW_EN_4	: 1;
	uint8_t		FIFO_OVERFLOW_EN_3	: 1;
	uint8_t		FIFO_OVERFLOW_EN_2	: 1;
	uint8_t		FIFO_OVERFLOW_EN_1	: 1;
	uint8_t		FIFO_OVERFLOW_EN_0	: 1;
	uint8_t 	FIFO_WM_EN_4		: 1;
	uint8_t 	FIFO_WM_EN_3		: 1;
	uint8_t 	FIFO_WM_EN_2		: 1;
	uint8_t 	FIFO_WM_EN_1		: 1;
	uint8_t 	FIFO_WM_EN_0		: 1;
}ICM_20948_INT_enable_t;

typedef union{
	ICM_20948_axis3bit16_t	raw;
	struct{
		int16_t x;
		int16_t y;
		int16_t z;
	}axes;
}ICM_20948_axis3named_t;

typedef struct{
	ICM_20948_axis3named_t 	acc;
	ICM_20948_axis3named_t 	gyr;
	ICM_20948_axis3named_t 	mag;
	union{
		ICM_20948_axis1bit16_t	raw;
		int16_t 				val;
	}tmp;
	ICM_20948_fss_t			fss;			// Full-scale range settings for this measurement
}ICM_20948_AGMT_t;

typedef struct{
	ICM_20948_Status_e	(*write)( uint8_t regaddr, uint8_t* pdata, uint32_t len, void* user);
	ICM_20948_Status_e	(*read)( uint8_t regaddr, uint8_t* pdata, uint32_t len, void* user);
	// void				(*delay)(uint32_t ms);
	void*				user;
}ICM_20948_Serif_t;							// This is the vtable of serial interface functions
extern const ICM_20948_Serif_t NullSerif;	// Here is a default for initialization (NULL)

typedef struct{
	const ICM_20948_Serif_t*	_serif;		// Pointer to the assigned Serif (Serial Interface) vtable
}ICM_20948_Device_t;						// Definition of device struct type


// Here's the list of what I want to be able to do:
/*

perform a generic startup routine that sets most things in the optimal performance range
Read / check against Who Am I
Add magnetometer to auxillary I2C bus and read it's values from the sensor values locations, configure ODR when accelerometer and gyro are both disabled
read raw accel and gyro values
configure accel/gyro update rates and dlpf's
read raw temp values
configure temperature sensor
load DMP firmware into the device
read DMP results from the device
configure interrupts 
	- configure interrupt and FSYNC pins 
	- configure which interrupts activate the interrupt pin
respond to interrupts on INT
configure FIFO (and use it)



callbacks for the user to respond to interrupt events


*/

// ICM_20948_Status_e ICM_20948_Startup( ICM_20948_Device_t* pdev ); // For the time being this performs a standardized startup routine


ICM_20948_Status_e	ICM_20948_link_serif( ICM_20948_Device_t* pdev, const ICM_20948_Serif_t* s );	// Links a SERIF structure to the device

// use the device's serif to perform a read or write
ICM_20948_Status_e	ICM_20948_execute_r( ICM_20948_Device_t* pdev, uint8_t regaddr, uint8_t* pdata, uint32_t len ); // Executes a R or W witht he serif vt as long as the pointers are not null
ICM_20948_Status_e	ICM_20948_execute_w( ICM_20948_Device_t* pdev, uint8_t regaddr, uint8_t* pdata, uint32_t len );


// Single-shot I2C on Master IF
ICM_20948_Status_e 	ICM_20948_i2c_master_slv4_txn( ICM_20948_Device_t* pdev, uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len, bool Rw, bool send_reg_addr );
ICM_20948_Status_e	ICM_20948_i2c_master_single_w( ICM_20948_Device_t* pdev, uint8_t addr, uint8_t reg, uint8_t* data );
ICM_20948_Status_e	ICM_20948_i2c_master_single_r( ICM_20948_Device_t* pdev, uint8_t addr, uint8_t reg, uint8_t* data );


// Device Level
ICM_20948_Status_e	ICM_20948_set_bank			( ICM_20948_Device_t* pdev, uint8_t bank );									// Sets the bank
ICM_20948_Status_e	ICM_20948_sw_reset			( ICM_20948_Device_t* pdev );												// Performs a SW reset
ICM_20948_Status_e	ICM_20948_sleep				( ICM_20948_Device_t* pdev, bool on );										// Set sleep mode for the chip
ICM_20948_Status_e	ICM_20948_low_power			( ICM_20948_Device_t* pdev, bool on );										// Set low power mode for the chip
ICM_20948_Status_e	ICM_20948_set_clock_source	( ICM_20948_Device_t* pdev, ICM_20948_PWR_MGMT_1_CLKSEL_e source ); 		// Choose clock source
ICM_20948_Status_e	ICM_20948_get_who_am_i		( ICM_20948_Device_t* pdev, uint8_t* whoami );								// Return whoami in out prarmeter
ICM_20948_Status_e	ICM_20948_check_id			( ICM_20948_Device_t* pdev );												// Return 'ICM_20948_Stat_Ok' if whoami matches ICM_20948_WHOAMI
ICM_20948_Status_e	ICM_20948_data_ready		( ICM_20948_Device_t* pdev );									// Returns 'Ok' if data is ready

// Interrupt Configuration
ICM_20948_Status_e	ICM_20948_int_pin_cfg		( ICM_20948_Device_t* pdev, ICM_20948_INT_PIN_CFG_t* write, ICM_20948_INT_PIN_CFG_t* read );	// Set the INT pin configuration
ICM_20948_Status_e	ICM_20948_int_enable 		( ICM_20948_Device_t* pdev, ICM_20948_INT_enable_t* write, ICM_20948_INT_enable_t* read );		// Write and or read the interrupt enable information. If non-null the write operation occurs before the read, so as to verify that the write was successful

// Internal Sensor Options
ICM_20948_Status_e	ICM_20948_set_sample_mode	( ICM_20948_Device_t* pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_LP_CONFIG_CYCLE_e mode );	// Use to set accel, gyro, and I2C master into cycled or continuous modes
ICM_20948_Status_e	ICM_20948_set_full_scale 	( ICM_20948_Device_t* pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_fss_t fss );
ICM_20948_Status_e	ICM_20948_set_dlpf_cfg		( ICM_20948_Device_t* pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_dlpcfg_t cfg );			
ICM_20948_Status_e	ICM_20948_enable_dlpf		( ICM_20948_Device_t* pdev, ICM_20948_InternalSensorID_bm sensors, bool enable );
ICM_20948_Status_e	ICM_20948_set_sample_rate	( ICM_20948_Device_t* pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_smplrt_t smplrt );

// Interface Things
ICM_20948_Status_e	ICM_20948_i2c_master_passthrough 			( ICM_20948_Device_t* pdev, bool passthrough );
ICM_20948_Status_e	ICM_20948_i2c_master_enable 				( ICM_20948_Device_t* pdev, bool enable );
ICM_20948_Status_e	ICM_20948_i2c_master_configure_slave 		( ICM_20948_Device_t* pdev, uint8_t slave, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap );



// Higher Level
ICM_20948_Status_e  ICM_20948_get_agmt          ( ICM_20948_Device_t* pdev, ICM_20948_AGMT_t* p );







// ToDo:

/* 
	Want to access magnetometer throught the I2C master interface...

  // If using the I2C master to read from the magnetometer
  // Enable the I2C master to talk to the magnetometer through the ICM 20948
  myICM.i2cMasterEnable( true ); 
  SERIAL_PORT.print(F("Enabling the I2C master returned ")); SERIAL_PORT.println(myICM.statusString());
  myICM.i2cMasterConfigureSlave ( 0, MAG_AK09916_I2C_ADDR, REG_ST1, 9, true, true, false, false, false );
  SERIAL_PORT.print(F("Configuring the magnetometer slave returned ")); SERIAL_PORT.println(myICM.statusString());

  // Operate the I2C master in duty-cycled mode
  myICM.setSampleMode( (ICM_20948_Internal_Mst | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Cycled ); // options: ICM_20948_Sample_Mode_Continuous or ICM_20948_Sample_Mode_Cycled
*/








#ifdef __cplusplus
}
#endif	/* __cplusplus */

#endif 	/* _ICM_20948_C_H_ */