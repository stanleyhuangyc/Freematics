#include "ICM_20948_C.h"
#include "ICM_20948_REGISTERS.h"
#include "AK09916_REGISTERS.h"



const ICM_20948_Serif_t NullSerif = {
	NULL,	// write
	NULL,	// read
	NULL,	// user
};

// Private function prototypes






// Function definitions
ICM_20948_Status_e	ICM_20948_link_serif( ICM_20948_Device_t* pdev, const ICM_20948_Serif_t* s ){
	if(s == NULL){ return ICM_20948_Stat_ParamErr; }
	if(pdev == NULL){ return ICM_20948_Stat_ParamErr; }
	pdev->_serif = s;
	return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e	ICM_20948_execute_w( ICM_20948_Device_t* pdev, uint8_t regaddr, uint8_t* pdata, uint32_t len ){
	if( pdev->_serif->write == NULL ){ return ICM_20948_Stat_NotImpl; }
	return (*pdev->_serif->write)( regaddr, pdata, len, pdev->_serif->user );
}

ICM_20948_Status_e	ICM_20948_execute_r( ICM_20948_Device_t* pdev, uint8_t regaddr, uint8_t* pdata, uint32_t len ){
	if( pdev->_serif->read == NULL ){ return ICM_20948_Stat_NotImpl; }
	return (*pdev->_serif->read)( regaddr, pdata, len, pdev->_serif->user );
}




// Single-shot I2C on Master IF
ICM_20948_Status_e	ICM_20948_i2c_master_slv4_txn( ICM_20948_Device_t* pdev, uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len, bool Rw, bool send_reg_addr ){
	// Thanks MikeFair! // https://github.com/kriswiner/MPU9250/issues/86
	
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	
	addr = (((Rw) ? 0x80 : 0x00) | addr );

	retval = ICM_20948_set_bank( pdev, 3 );
	retval = ICM_20948_execute_w( pdev, AGB3_REG_I2C_SLV4_ADDR, (uint8_t*)&addr, 1 );
	if( retval != ICM_20948_Stat_Ok ){ return retval; }

	retval = ICM_20948_set_bank( pdev, 3 );
	retval = ICM_20948_execute_w( pdev, AGB3_REG_I2C_SLV4_REG, (uint8_t*)&reg, 1 );
	if( retval != ICM_20948_Stat_Ok ){ return retval; }

	ICM_20948_I2C_SLV4_CTRL_t ctrl;
	ctrl.EN = 1;
	ctrl.INT_EN = false;
	ctrl.DLY = 0;
	ctrl.REG_DIS = !send_reg_addr;

	// ICM_20948_I2C_MST_STATUS_t i2c_mst_status;
	// bool txn_failed = false;
	uint16_t nByte = 0;

	while( nByte < len ){
		if( !Rw ){
			retval = ICM_20948_set_bank( pdev, 3 );
			retval = ICM_20948_execute_w( pdev, AGB3_REG_I2C_SLV4_DO, (uint8_t*)&(data[nByte]), 1 );
			if( retval != ICM_20948_Stat_Ok ){ return retval; }
		}

		// Kick off txn
		retval = ICM_20948_set_bank( pdev, 3 );
		retval = ICM_20948_execute_w( pdev, AGB3_REG_I2C_SLV4_CTRL, (uint8_t*)&ctrl, sizeof(ICM_20948_I2C_SLV4_CTRL_t) );
		if( retval != ICM_20948_Stat_Ok ){ return retval; }

	// 	// long tsTimeout = millis() + 3000;  // Emergency timeout for txn (hard coded to 3 secs)
	// 	uint32_t max_cycles = 1000;
	// 	uint32_t count = 0;
	// 	bool slave4Done = false;
	// 	while (!slave4Done) { 
	// 		retval = ICM_20948_set_bank( pdev, 0 );
	// 		retval = ICM_20948_execute_r( pdev, AGB0_REG_I2C_MST_STATUS, &i2c_mst_status, 1 );

	// 		slave4Done = ( i2c_mst_status.I2C_SLV4_DONE /*| (millis() > tsTimeout) */ ); // todo: avoid forever-loops
	// 		slave4Done |= (count >= max_cycles);
	// 		count++;
	// 	}
	// 	txn_failed = (i2c_mst_status.I2C_SLV4_NACK /* & (1 << I2C_SLV4_NACK_BIT)) | (millis() > tsTimeout) */);
	// 	txn_failed |= (count >= max_cycles);
	// 	if (txn_failed) break;

	// 	if ( Rw ){ 
	// 		retval = ICM_20948_set_bank( pdev, 3 );
	// 		retval = ICM_20948_execute_r( pdev, AGB3_REG_I2C_SLV4_DI, &data[nByte], 1 );
	// 	}

		nByte++;
	}
	// if( txn_failed ){ return ICM_20948_Stat_Err; }
	return retval;
}

ICM_20948_Status_e	ICM_20948_i2c_master_single_w( ICM_20948_Device_t* pdev, uint8_t addr, uint8_t reg, uint8_t* data ){
	return ICM_20948_i2c_master_slv4_txn( pdev, addr, reg, data, 1, false, true );
}

ICM_20948_Status_e	ICM_20948_i2c_master_single_r( ICM_20948_Device_t* pdev, uint8_t addr, uint8_t reg, uint8_t* data ){
	return ICM_20948_i2c_master_slv4_txn( pdev, addr, reg, data, 1, true, true );
}




ICM_20948_Status_e	ICM_20948_set_bank( ICM_20948_Device_t* pdev, uint8_t bank ){
	if( bank > 3 ){ return ICM_20948_Stat_ParamErr; } // Only 4 possible banks
	bank = (bank << 4) & 0x30; // bits 5:4 of REG_BANK_SEL
	return ICM_20948_execute_w( pdev, REG_BANK_SEL, &bank, 1 );
}

ICM_20948_Status_e	ICM_20948_sw_reset( ICM_20948_Device_t* pdev ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	ICM_20948_PWR_MGMT_1_t reg;

	ICM_20948_set_bank(pdev, 0);	// Must be in the right bank

	retval = ICM_20948_execute_r( pdev, AGB0_REG_PWR_MGMT_1, (uint8_t*)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
	if( retval != ICM_20948_Stat_Ok ){ return retval; }

	reg.DEVICE_RESET = 1;

	retval = ICM_20948_execute_w( pdev, AGB0_REG_PWR_MGMT_1, (uint8_t*)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
	if( retval != ICM_20948_Stat_Ok ){ return retval; }
	return retval;
}

ICM_20948_Status_e	ICM_20948_sleep				( ICM_20948_Device_t* pdev, bool on ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	ICM_20948_PWR_MGMT_1_t reg;

	ICM_20948_set_bank(pdev, 0);	// Must be in the right bank

	retval = ICM_20948_execute_r( pdev, AGB0_REG_PWR_MGMT_1, (uint8_t*)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
	if( retval != ICM_20948_Stat_Ok ){ return retval; }

	if(on){ reg.SLEEP = 1; }
	else{ reg.SLEEP = 0; }

	retval = ICM_20948_execute_w( pdev, AGB0_REG_PWR_MGMT_1, (uint8_t*)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
	if( retval != ICM_20948_Stat_Ok ){ return retval; }
	return retval;
}

ICM_20948_Status_e	ICM_20948_low_power			( ICM_20948_Device_t* pdev, bool on ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	ICM_20948_PWR_MGMT_1_t reg;

	ICM_20948_set_bank(pdev, 0);	// Must be in the right bank

	retval = ICM_20948_execute_r( pdev, AGB0_REG_PWR_MGMT_1, (uint8_t*)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
	if( retval != ICM_20948_Stat_Ok ){ return retval; }

	if(on){ reg.LP_EN = 1; }
	else{ reg.LP_EN = 0; }

	retval = ICM_20948_execute_w( pdev, AGB0_REG_PWR_MGMT_1, (uint8_t*)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
	if( retval != ICM_20948_Stat_Ok ){ return retval; }
	return retval;
}

ICM_20948_Status_e	ICM_20948_set_clock_source	( ICM_20948_Device_t* pdev, ICM_20948_PWR_MGMT_1_CLKSEL_e source ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	ICM_20948_PWR_MGMT_1_t reg;

	ICM_20948_set_bank(pdev, 0);	// Must be in the right bank
	
	retval = ICM_20948_execute_r( pdev, AGB0_REG_PWR_MGMT_1, (uint8_t*)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
	if( retval != ICM_20948_Stat_Ok ){ return retval; }

	reg.CLKSEL = source;

	retval = ICM_20948_execute_w( pdev, AGB0_REG_PWR_MGMT_1, (uint8_t*)&reg, sizeof(ICM_20948_PWR_MGMT_1_t));
	if( retval != ICM_20948_Stat_Ok ){ return retval; }
	return retval;
}



ICM_20948_Status_e ICM_20948_get_who_am_i( ICM_20948_Device_t* pdev, uint8_t* whoami ){
	if( whoami == NULL ){ return ICM_20948_Stat_ParamErr; }
	ICM_20948_set_bank(pdev, 0);	// Must be in the right bank
	return ICM_20948_execute_r( pdev, AGB0_REG_WHO_AM_I, whoami, 1 );
}

ICM_20948_Status_e	ICM_20948_check_id( ICM_20948_Device_t* pdev ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	uint8_t whoami = 0x00;
	retval = ICM_20948_get_who_am_i( pdev, &whoami );
	if( retval != ICM_20948_Stat_Ok){ return retval; }
	if( whoami != ICM_20948_WHOAMI ){ return ICM_20948_Stat_WrongID; }
	return retval;
}

ICM_20948_Status_e	ICM_20948_data_ready( ICM_20948_Device_t* pdev ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	ICM_20948_INT_STATUS_1_t reg;
	retval = ICM_20948_set_bank(pdev, 0);	// Must be in the right bank
	if( retval != ICM_20948_Stat_Ok){ return retval; }
	retval = ICM_20948_execute_r( pdev, AGB0_REG_INT_STATUS_1, (uint8_t*)&reg, sizeof(ICM_20948_INT_STATUS_1_t));
	if( retval != ICM_20948_Stat_Ok){ return retval; }
	if( !reg.RAW_DATA_0_RDY_INT ){ retval = ICM_20948_Stat_NoData; }
	return retval;
}







// Interrupt Configuration
ICM_20948_Status_e	ICM_20948_int_pin_cfg		( ICM_20948_Device_t* pdev, ICM_20948_INT_PIN_CFG_t* write, ICM_20948_INT_PIN_CFG_t* read ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	retval = ICM_20948_set_bank(pdev, 0);						// Must be in the right bank
	if( write != NULL ){										// write first, if available
		retval = ICM_20948_execute_w( pdev, AGB0_REG_INT_PIN_CONFIG, (uint8_t*)write, sizeof(ICM_20948_INT_PIN_CFG_t));
		if( retval != ICM_20948_Stat_Ok ){ return retval; }
	}
	if( read != NULL ){											// then read, to allow for verification
		retval = ICM_20948_execute_r( pdev, AGB0_REG_INT_PIN_CONFIG, (uint8_t*)read, sizeof(ICM_20948_INT_PIN_CFG_t));
		if( retval != ICM_20948_Stat_Ok ){ return retval; }
	}
	return retval;
}

ICM_20948_Status_e	ICM_20948_int_enable 		( ICM_20948_Device_t* pdev, ICM_20948_INT_enable_t* write, ICM_20948_INT_enable_t* read ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

	ICM_20948_INT_ENABLE_t 		en_0;
	ICM_20948_INT_ENABLE_1_t 	en_1;
	ICM_20948_INT_ENABLE_2_t 	en_2;
	ICM_20948_INT_ENABLE_3_t 	en_3;

	retval = ICM_20948_set_bank(pdev, 0);	// Must be in the right bank

	if( write != NULL ){	// If the write pointer is not NULL then write to the registers BEFORE reading
		en_0.I2C_MST_INT_EN = write->I2C_MST_INT_EN;
		en_0.DMP_INT1_EN = write->DMP_INT1_EN;
		en_0.PLL_READY_EN = write->PLL_RDY_EN;
		en_0.WOM_INT_EN = write->WOM_INT_EN;
		en_0.REG_WOF_EN = write->REG_WOF_EN;
		en_1.RAW_DATA_0_RDY_EN = write->RAW_DATA_0_RDY_EN;
		en_2.individual.FIFO_OVERFLOW_EN_4 = write->FIFO_OVERFLOW_EN_4;
		en_2.individual.FIFO_OVERFLOW_EN_3 = write->FIFO_OVERFLOW_EN_3;
		en_2.individual.FIFO_OVERFLOW_EN_2 = write->FIFO_OVERFLOW_EN_2;
		en_2.individual.FIFO_OVERFLOW_EN_1 = write->FIFO_OVERFLOW_EN_1;
		en_2.individual.FIFO_OVERFLOW_EN_0 = write->FIFO_OVERFLOW_EN_0;
		en_3.individual.FIFO_WM_EN_4 = write->FIFO_WM_EN_4;
		en_3.individual.FIFO_WM_EN_3 = write->FIFO_WM_EN_3;
		en_3.individual.FIFO_WM_EN_2 = write->FIFO_WM_EN_2;
		en_3.individual.FIFO_WM_EN_1 = write->FIFO_WM_EN_1;
		en_3.individual.FIFO_WM_EN_0 = write->FIFO_WM_EN_0;

		retval = ICM_20948_execute_w( pdev, AGB0_REG_INT_ENABLE, (uint8_t*)&en_0, sizeof(ICM_20948_INT_ENABLE_t)); if( retval != ICM_20948_Stat_Ok ){ return retval; }
		retval = ICM_20948_execute_w( pdev, AGB0_REG_INT_ENABLE_1, (uint8_t*)&en_1, sizeof(ICM_20948_INT_ENABLE_1_t)); if( retval != ICM_20948_Stat_Ok ){ return retval; }
		retval = ICM_20948_execute_w( pdev, AGB0_REG_INT_ENABLE_2, (uint8_t*)&en_2, sizeof(ICM_20948_INT_ENABLE_2_t)); if( retval != ICM_20948_Stat_Ok ){ return retval; }
		retval = ICM_20948_execute_w( pdev, AGB0_REG_INT_ENABLE_3, (uint8_t*)&en_3, sizeof(ICM_20948_INT_ENABLE_3_t)); if( retval != ICM_20948_Stat_Ok ){ return retval; }
	}

	if( read != NULL ){	// If read pointer is not NULL then read the registers (if write is not NULL then this should read back the results of write into read)
		retval = ICM_20948_execute_r( pdev, AGB0_REG_INT_ENABLE, (uint8_t*)&en_0, sizeof(ICM_20948_INT_ENABLE_t)); if( retval != ICM_20948_Stat_Ok ){ return retval; }
		retval = ICM_20948_execute_r( pdev, AGB0_REG_INT_ENABLE_1, (uint8_t*)&en_1, sizeof(ICM_20948_INT_ENABLE_1_t)); if( retval != ICM_20948_Stat_Ok ){ return retval; }
		retval = ICM_20948_execute_r( pdev, AGB0_REG_INT_ENABLE_2, (uint8_t*)&en_2, sizeof(ICM_20948_INT_ENABLE_2_t)); if( retval != ICM_20948_Stat_Ok ){ return retval; }
		retval = ICM_20948_execute_r( pdev, AGB0_REG_INT_ENABLE_3, (uint8_t*)&en_3, sizeof(ICM_20948_INT_ENABLE_3_t)); if( retval != ICM_20948_Stat_Ok ){ return retval; }
	
		read->I2C_MST_INT_EN = en_0.I2C_MST_INT_EN;
		read->DMP_INT1_EN = en_0.DMP_INT1_EN;
		read->PLL_RDY_EN = en_0.PLL_READY_EN;
		read->WOM_INT_EN = en_0.WOM_INT_EN;
		read->REG_WOF_EN = en_0.REG_WOF_EN;
		read->RAW_DATA_0_RDY_EN = en_1.RAW_DATA_0_RDY_EN;
		read->FIFO_OVERFLOW_EN_4 = en_2.individual.FIFO_OVERFLOW_EN_4;
		read->FIFO_OVERFLOW_EN_3 = en_2.individual.FIFO_OVERFLOW_EN_3;
		read->FIFO_OVERFLOW_EN_2 = en_2.individual.FIFO_OVERFLOW_EN_2;
		read->FIFO_OVERFLOW_EN_1 = en_2.individual.FIFO_OVERFLOW_EN_1;
		read->FIFO_OVERFLOW_EN_0 = en_2.individual.FIFO_OVERFLOW_EN_0;
		read->FIFO_WM_EN_4 = en_3.individual.FIFO_WM_EN_4;
		read->FIFO_WM_EN_3 = en_3.individual.FIFO_WM_EN_3;
		read->FIFO_WM_EN_2 = en_3.individual.FIFO_WM_EN_2;
		read->FIFO_WM_EN_1 = en_3.individual.FIFO_WM_EN_1;
		read->FIFO_WM_EN_0 = en_3.individual.FIFO_WM_EN_0;
	}

	return retval;
}









ICM_20948_Status_e	ICM_20948_set_sample_mode( ICM_20948_Device_t* pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_LP_CONFIG_CYCLE_e mode ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	ICM_20948_LP_CONFIG_t reg;

	if( !(sensors & ( ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr | ICM_20948_Internal_Mst ) ) ){ return ICM_20948_Stat_SensorNotSupported; }
	
	retval = ICM_20948_set_bank(pdev, 0);				// Must be in the right bank
	if( retval != ICM_20948_Stat_Ok){ return retval; }
	retval = ICM_20948_execute_r( pdev, AGB0_REG_LP_CONFIG, (uint8_t*)&reg, sizeof(ICM_20948_LP_CONFIG_t));
	if( retval != ICM_20948_Stat_Ok){ return retval; }
	
	if( sensors & ICM_20948_Internal_Acc ){ reg.ACCEL_CYCLE = mode; }		// Set all desired sensors to this setting
	if( sensors & ICM_20948_Internal_Gyr ){ reg.GYRO_CYCLE = mode; }
	if( sensors & ICM_20948_Internal_Mst ){ reg.I2C_MST_CYCLE = mode; }

	retval = ICM_20948_execute_w( pdev, AGB0_REG_LP_CONFIG, (uint8_t*)&reg, sizeof(ICM_20948_LP_CONFIG_t));
	if( retval != ICM_20948_Stat_Ok ){ return retval; }
	return retval;
}

ICM_20948_Status_e	ICM_20948_set_full_scale 	( ICM_20948_Device_t* pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_fss_t fss ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

	if( !(sensors & ( ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr ) ) ){ return ICM_20948_Stat_SensorNotSupported; }

	if( sensors & ICM_20948_Internal_Acc ){
		ICM_20948_ACCEL_CONFIG_t reg;
		retval |= ICM_20948_set_bank(pdev, 2);	// Must be in the right bank
		retval |= ICM_20948_execute_r( pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t*)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t));
		reg.ACCEL_FS_SEL = fss.a;
		retval |= ICM_20948_execute_w( pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t*)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t));
	}
	if( sensors & ICM_20948_Internal_Gyr ){
		ICM_20948_GYRO_CONFIG_1_t reg;
		retval |= ICM_20948_set_bank(pdev, 2);	// Must be in the right bank
		retval |= ICM_20948_execute_r( pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t*)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t));
		reg.GYRO_FS_SEL = fss.g;
		retval |= ICM_20948_execute_w( pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t*)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t));
	}
	return retval;
}

ICM_20948_Status_e	ICM_20948_set_dlpf_cfg		( ICM_20948_Device_t* pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_dlpcfg_t cfg ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

	if( !(sensors & ( ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr ) ) ){ return ICM_20948_Stat_SensorNotSupported; }

	if( sensors & ICM_20948_Internal_Acc ){
		ICM_20948_ACCEL_CONFIG_t reg;
		retval |= ICM_20948_set_bank(pdev, 2);	// Must be in the right bank
		retval |= ICM_20948_execute_r( pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t*)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t));
		reg.ACCEL_DLPFCFG = cfg.a;
		retval |= ICM_20948_execute_w( pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t*)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t));
	}
	if( sensors & ICM_20948_Internal_Gyr ){
		ICM_20948_GYRO_CONFIG_1_t reg;
		retval |= ICM_20948_set_bank(pdev, 2);	// Must be in the right bank
		retval |= ICM_20948_execute_r( pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t*)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t));
		reg.GYRO_DLPFCFG = cfg.g;
		retval |= ICM_20948_execute_w( pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t*)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t));
	}
	return retval;
}	

ICM_20948_Status_e	ICM_20948_enable_dlpf		( ICM_20948_Device_t* pdev, ICM_20948_InternalSensorID_bm sensors, bool enable ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

	if( !(sensors & ( ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr ) ) ){ return ICM_20948_Stat_SensorNotSupported; }

	if( sensors & ICM_20948_Internal_Acc ){
		ICM_20948_ACCEL_CONFIG_t reg;
		retval |= ICM_20948_set_bank(pdev, 2);	// Must be in the right bank
		retval |= ICM_20948_execute_r( pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t*)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t));
		if( enable ){ reg.ACCEL_FCHOICE = 1; }
		else{ reg.ACCEL_FCHOICE = 0; }
		retval |= ICM_20948_execute_w( pdev, AGB2_REG_ACCEL_CONFIG, (uint8_t*)&reg, sizeof(ICM_20948_ACCEL_CONFIG_t));
	}
	if( sensors & ICM_20948_Internal_Gyr ){
		ICM_20948_GYRO_CONFIG_1_t reg;
		retval |= ICM_20948_set_bank(pdev, 2);	// Must be in the right bank
		retval |= ICM_20948_execute_r( pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t*)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t));
		if( enable ){ reg.GYRO_FCHOICE = 1; }
		else{ reg.GYRO_FCHOICE = 0; }
		retval |= ICM_20948_execute_w( pdev, AGB2_REG_GYRO_CONFIG_1, (uint8_t*)&reg, sizeof(ICM_20948_GYRO_CONFIG_1_t));
	}
	return retval;
}

ICM_20948_Status_e	ICM_20948_set_sample_rate	( ICM_20948_Device_t* pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_smplrt_t smplrt ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

	if( !(sensors & ( ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr ) ) ){ return ICM_20948_Stat_SensorNotSupported; }

	if( sensors & ICM_20948_Internal_Acc ){
		retval |= ICM_20948_set_bank(pdev, 2);	// Must be in the right bank
		uint8_t div1 = (smplrt.a << 8);
		uint8_t div2 = (smplrt.a & 0xFF);
		retval |= ICM_20948_execute_w( pdev, AGB2_REG_ACCEL_SMPLRT_DIV_1, &div1, 1);
		retval |= ICM_20948_execute_w( pdev, AGB2_REG_ACCEL_SMPLRT_DIV_2, &div2, 1);
	}
	if( sensors & ICM_20948_Internal_Gyr ){
		retval |= ICM_20948_set_bank(pdev, 2);	// Must be in the right bank
		uint8_t div = (smplrt.g);
		retval |= ICM_20948_execute_w( pdev, AGB2_REG_GYRO_SMPLRT_DIV, &div, 1);
	}
	return retval;
}




// Interface Things
ICM_20948_Status_e	ICM_20948_i2c_master_passthrough 			( ICM_20948_Device_t* pdev, bool passthrough ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

	ICM_20948_INT_PIN_CFG_t reg;
	retval = ICM_20948_set_bank(pdev, 0);
	if( retval != ICM_20948_Stat_Ok ){ return retval; }
	retval = ICM_20948_execute_r( pdev, AGB0_REG_INT_PIN_CONFIG, (uint8_t*)&reg, sizeof(ICM_20948_INT_PIN_CFG_t) );
	if( retval != ICM_20948_Stat_Ok ){ return retval; }
	reg.BYPASS_EN = passthrough;
	retval = ICM_20948_execute_w( pdev, AGB0_REG_INT_PIN_CONFIG, (uint8_t*)&reg, sizeof(ICM_20948_INT_PIN_CFG_t) );
	if( retval != ICM_20948_Stat_Ok ){ return retval; }

	return retval;
}

ICM_20948_Status_e	ICM_20948_i2c_master_enable ( ICM_20948_Device_t* pdev, bool enable ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

	// Disable BYPASS_EN
	retval = ICM_20948_i2c_master_passthrough( pdev, false );
	if( retval != ICM_20948_Stat_Ok ){ return retval; }

	ICM_20948_I2C_MST_CTRL_t ctrl;
	retval = ICM_20948_set_bank(pdev, 3);
	if( retval != ICM_20948_Stat_Ok ){ return retval; }	
	retval = ICM_20948_execute_r( pdev, AGB3_REG_I2C_MST_CTRL, (uint8_t*)&ctrl, sizeof(ICM_20948_I2C_MST_CTRL_t) );
	if( retval != ICM_20948_Stat_Ok ){ return retval; }
	ctrl.I2C_MST_CLK = 0x07; // corresponds to 345.6 kHz, good for up to 400 kHz
	ctrl.I2C_MST_P_NSR = 1;
	retval = ICM_20948_execute_w( pdev, AGB3_REG_I2C_MST_CTRL, (uint8_t*)&ctrl, sizeof(ICM_20948_I2C_MST_CTRL_t) );
	if( retval != ICM_20948_Stat_Ok ){ return retval; }

	ICM_20948_USER_CTRL_t reg;
	retval = ICM_20948_set_bank(pdev, 0);
	if( retval != ICM_20948_Stat_Ok ){ return retval; }
	retval = ICM_20948_execute_r( pdev, AGB0_REG_USER_CTRL, (uint8_t*)&reg, sizeof(ICM_20948_USER_CTRL_t) );
	if( retval != ICM_20948_Stat_Ok ){ return retval; }
	if( enable ){ reg.I2C_MST_EN = 1; }
	else{ reg.I2C_MST_EN = 0; }
	retval = ICM_20948_execute_w( pdev, AGB0_REG_USER_CTRL, (uint8_t*)&reg, sizeof(ICM_20948_USER_CTRL_t) );
	if( retval != ICM_20948_Stat_Ok ){ return retval; }

	return retval;
}

ICM_20948_Status_e	ICM_20948_i2c_master_configure_slave 		( ICM_20948_Device_t* pdev, uint8_t slave, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap ){
	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	
	uint8_t slv_addr_reg;
	uint8_t slv_reg_reg;
	uint8_t slv_ctrl_reg;

	switch( slave ){
		case 0 : slv_addr_reg = AGB3_REG_I2C_SLV0_ADDR; slv_reg_reg = AGB3_REG_I2C_SLV0_REG; slv_ctrl_reg = AGB3_REG_I2C_SLV0_CTRL; break;
		case 1 : slv_addr_reg = AGB3_REG_I2C_SLV1_ADDR; slv_reg_reg = AGB3_REG_I2C_SLV1_REG; slv_ctrl_reg = AGB3_REG_I2C_SLV1_CTRL; break;
		case 2 : slv_addr_reg = AGB3_REG_I2C_SLV2_ADDR; slv_reg_reg = AGB3_REG_I2C_SLV2_REG; slv_ctrl_reg = AGB3_REG_I2C_SLV2_CTRL; break;
		case 3 : slv_addr_reg = AGB3_REG_I2C_SLV3_ADDR; slv_reg_reg = AGB3_REG_I2C_SLV3_REG; slv_ctrl_reg = AGB3_REG_I2C_SLV3_CTRL; break;
		default :
			return ICM_20948_Stat_ParamErr;
	}

	// Set the slave address and the Rw flag
	ICM_20948_I2C_SLVX_ADDR_t address;
	address.ID = addr;
	if( Rw ){ address.RNW = 1; }
	retval = ICM_20948_execute_w( pdev, slv_addr_reg, (uint8_t*)&address, sizeof(ICM_20948_I2C_SLVX_ADDR_t) );
	if( retval != ICM_20948_Stat_Ok ){ return retval; }

	// Set the slave sub-address (reg)
	ICM_20948_I2C_SLVX_REG_t subaddress;
	subaddress.REG = reg;
	retval = ICM_20948_execute_w( pdev, slv_reg_reg, (uint8_t*)&subaddress, sizeof(ICM_20948_I2C_SLVX_REG_t) );
	if( retval != ICM_20948_Stat_Ok ){ return retval; }

	// Set up the control info
	ICM_20948_I2C_SLVX_CTRL_t ctrl;
	ctrl.LENG = len;
	ctrl.EN = enable;
	ctrl.REG_DIS = data_only;
	ctrl.GRP = grp;
	ctrl.BYTE_SW = swap;
	retval = ICM_20948_execute_w( pdev, slv_ctrl_reg, (uint8_t*)&ctrl, sizeof(ICM_20948_I2C_SLVX_CTRL_t) );
	if( retval != ICM_20948_Stat_Ok ){ return retval; }

	return retval;
}









// Higher Level
ICM_20948_Status_e  ICM_20948_get_agmt          ( ICM_20948_Device_t* pdev, ICM_20948_AGMT_t* pagmt ){
	if( pagmt == NULL ){ return ICM_20948_Stat_ParamErr; }

	ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
	const uint8_t numbytes = 14;
	uint8_t buff[numbytes];

	// Get readings
	retval |= ICM_20948_set_bank( pdev, 0 ); 
	retval |= ICM_20948_execute_r( pdev, (uint8_t)AGB0_REG_ACCEL_XOUT_H, buff, numbytes );

	pagmt->acc.axes.x = ((buff[0] << 8) | (buff[1] & 0xFF));
	pagmt->acc.axes.y = ((buff[2] << 8) | (buff[3] & 0xFF));
	pagmt->acc.axes.z = ((buff[4] << 8) | (buff[5] & 0xFF));

	pagmt->gyr.axes.x = ((buff[6] << 8) | (buff[7] & 0xFF));
	pagmt->gyr.axes.y = ((buff[8] << 8) | (buff[9] & 0xFF));
	pagmt->gyr.axes.z = ((buff[10] << 8) | (buff[11] & 0xFF));

	pagmt->tmp.val = ((buff[12] << 8) | (buff[13] & 0xFF));

	// ToDo: get magnetometer readings
	//  pagmt->mag.axes.x =
	//  pagmt->mag.axes.y =
	//  pagmt->mag.axes.z =


	// Get settings to be able to compute scaled values
	retval |= ICM_20948_set_bank( pdev, 2 ); 
	ICM_20948_ACCEL_CONFIG_t acfg; 
	retval |= ICM_20948_execute_r( pdev, (uint8_t)AGB2_REG_ACCEL_CONFIG, (uint8_t*)&acfg, 1*sizeof(acfg) );
	pagmt->fss.a = acfg.ACCEL_FS_SEL; 	// Worth noting that without explicitly setting the FS range of the accelerometer it was showing the register value for +/- 2g but the reported values were actually scaled to the +/- 16g range 
										// Wait a minute... now it seems like this problem actually comes from the digital low-pass filter. When enabled the value is 1/8 what it should be...
	retval |= ICM_20948_set_bank( pdev, 2 ); 
	ICM_20948_GYRO_CONFIG_1_t gcfg1;
	retval |= ICM_20948_execute_r( pdev, (uint8_t)AGB2_REG_GYRO_CONFIG_1, (uint8_t*)&gcfg1, 1*sizeof(gcfg1) );
	pagmt->fss.g = gcfg1.GYRO_FS_SEL;
	ICM_20948_ACCEL_CONFIG_2_t acfg2;
	retval |= ICM_20948_execute_r( pdev, (uint8_t)AGB2_REG_ACCEL_CONFIG_2, (uint8_t*)&acfg2, 1*sizeof(acfg2) );
 
	return retval;
}



















