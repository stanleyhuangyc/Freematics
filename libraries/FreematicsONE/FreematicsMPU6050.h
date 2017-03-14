/*************************************************************************
* Freematics MPU6050 helper class
* Distributed under BSD license
* Visit http://freematics.com for more information
* (C)2016 Stanley Huang <support@freematics.com.au>
*************************************************************************/

#ifndef _MPU6050_H
#define _MPU6050_H

typedef struct
{
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
} MPU6050_READOUT_DATA;

#define MPU6050_I2C_ADDRESS 0x68
#define MPU6050_ACCEL_XOUT_H       0x3B   // R
#define MPU6050_ACCEL_XOUT_L       0x3C   // R
#define MPU6050_ACCEL_YOUT_H       0x3D   // R
#define MPU6050_ACCEL_YOUT_L       0x3E   // R
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R
#define MPU6050_ACCEL_ZOUT_L       0x40   // R
#define MPU6050_TEMP_OUT_H         0x41   // R
#define MPU6050_TEMP_OUT_L         0x42   // R
#define MPU6050_GYRO_XOUT_H        0x43   // R
#define MPU6050_GYRO_XOUT_L        0x44   // R
#define MPU6050_GYRO_YOUT_H        0x45   // R
#define MPU6050_GYRO_YOUT_L        0x46   // R
#define MPU6050_GYRO_ZOUT_H        0x47   // R
#define MPU6050_GYRO_ZOUT_L        0x48   // R
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_PWR_MGMT_2         0x6C   // R/W
#define MPU6050_WHO_AM_I           0x75   // R

class CMPU6050
{
public:
	// initialize MEMS
	bool memsInit();
	// read out MEMS data (acc for accelerometer x/y/z, gyr for gyroscope x/y/z, temp in 0.1 celcius degree)
	bool memsRead(int16_t* acc, int16_t* gyr = 0, int16_t* mag = 0, int16_t* temp = 0);
private:
	bool MPU6050_read(int start, uint8_t* buffer, int size);
	bool MPU6050_write(int start, const uint8_t* pData, int size);
	bool MPU6050_write_reg(int reg, uint8_t data);
	void MPU6050_store(int16_t* pData, uint8_t data_l, uint8_t data_h);
};

#endif
