/*************************************************************************
* Freematics MPU6050 helper class
* Distributed under BSD license
* Visit http://freematics.com for more information
* (C)2016 Stanley Huang <support@freematics.com.au>
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include "FreematicsMPU6050.h"

bool CMPU6050::memsInit()
{
	// default at power-up:
	//    Gyro at 250 degrees second
	//    Acceleration at 2g
	//    Clock source at internal 8MHz
	//    The device is in sleep mode.
	//
	uint8_t c;
	bool success;
	success = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
	if (!success) return false;

	// According to the datasheet, the 'sleep' bit
	// should read a '1'. But I read a '0'.
	// That bit has to be cleared, since the sensor
	// is in sleep mode at power-up. Even if the
	// bit reads '0'.
	success = MPU6050_read (MPU6050_PWR_MGMT_2, &c, 1);
	if (!success) return false;

	// Clear the 'sleep' bit to start the sensor.
	MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
	return true;
}

bool CMPU6050::memsRead(int16_t* acc, int16_t* gyr, int16_t* mag, int16_t* temp)
{
	bool success;

	// Read the raw values.
	// Read 14 bytes at once,
	// containing acceleration, temperature and gyro.
	// With the default settings of the MPU-6050,
	// there is no filter enabled, and the values
	// are not very stable.
	
	MPU6050_READOUT_DATA accel_t_gyro;
	success = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *)&accel_t_gyro, sizeof(MPU6050_READOUT_DATA));
	if (!success) return false;
	
	if (temp) {
		// 340 per degrees Celsius, -512 at 35 degrees.
		*temp = ((int)(((uint16_t)accel_t_gyro.t_h << 8) | accel_t_gyro.t_l) + 512) / 34 + 350; 
	}

	if (acc) {
		MPU6050_store(acc, accel_t_gyro.x_accel_l, accel_t_gyro.x_accel_h);
		MPU6050_store(acc + 1, accel_t_gyro.y_accel_l, accel_t_gyro.y_accel_h);
		MPU6050_store(acc + 2, accel_t_gyro.z_accel_l, accel_t_gyro.z_accel_h);
	}

	if (gyr) {
		MPU6050_store(gyr, accel_t_gyro.x_gyro_l, accel_t_gyro.x_gyro_h);
		MPU6050_store(gyr + 1, accel_t_gyro.y_gyro_l, accel_t_gyro.y_gyro_h);
		MPU6050_store(gyr + 2, accel_t_gyro.z_gyro_l, accel_t_gyro.z_gyro_h);
	}
	
	if (mag) {
		// no magnetometer
		mag[0] = 0;
		mag[1] = 0;
		mag[2] = 0;
	}
	
	return true;
}

void CMPU6050::MPU6050_store(int16_t* pData, uint8_t data_l, uint8_t data_h)
{
	uint8_t* ptr = (uint8_t*)pData;
	*ptr = data_l;
	*(ptr + 1) = data_h;
}

bool CMPU6050::MPU6050_read(int start, uint8_t* buffer, int size)
{
	int i, n;

	Wire.beginTransmission(MPU6050_I2C_ADDRESS);
	Wire.write(start);
	Wire.endTransmission(false);    // hold the I2C-bus

	// Third parameter is true: relase I2C-bus after data is read.
	Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
	while(Wire.available() && i<size)
	{
		buffer[i++]=Wire.read();
	}
	return i == size;
}


bool CMPU6050::MPU6050_write(int start, const uint8_t* pData, int size)
{
	int n;

	Wire.beginTransmission(MPU6050_I2C_ADDRESS);
	Wire.write(start);        // write the start address
	n = Wire.write(pData, size);  // write data bytes
	if (n != size) return false;
	Wire.endTransmission(true); // release the I2C-bus
	return true;
}

bool CMPU6050::MPU6050_write_reg(int reg, uint8_t data)
{
	return MPU6050_write(reg, &data, 1);
}
