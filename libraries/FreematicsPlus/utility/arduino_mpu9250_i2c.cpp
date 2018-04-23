/******************************************************************************
arduino_mpu9250_i2c.cpp - MPU-9250 Digital Motion Processor Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

This library implements motion processing functions of Invensense's MPU-9250.
It is based on their Emedded MotionDriver 6.12 library.
	https://www.invensense.com/developers/software-downloads/

******************************************************************************/
#include <Arduino.h>
#include "arduino_mpu9250_i2c.h"
#include <driver/i2c.h>

#define WRITE_BIT         I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT          I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN      0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS     0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL           (i2c_ack_type_t)0x0              /*!< I2C ack value */
#define NACK_VAL          (i2c_ack_type_t)0x1 /*!< I2C nack value */

int arduino_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, ( slave_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
	i2c_master_write(cmd, data, length, ACK_CHECK_DIS);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret == ESP_OK ? 0 : -1;
}

int arduino_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
	// write sub-address
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, ( slave_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) return -1;
	// read data
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, ( slave_addr << 1 ) | READ_BIT, ACK_CHECK_EN);
	if (length > 1) {
		i2c_master_read(cmd, data, length - 1, ACK_VAL);
	}
	i2c_master_read_byte(cmd, data + length - 1, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret == ESP_OK ? 0 : -1;
}

int arduino_get_clock_ms(unsigned long *count)
{
	*count = millis();
	return 0;
}

int arduino_delay_ms(unsigned long num_ms)
{
	delay(num_ms);
	return 0;
}
