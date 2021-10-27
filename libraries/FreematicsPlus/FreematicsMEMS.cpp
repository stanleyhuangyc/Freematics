/*************************************************************************
* Freematics MEMS motion sensor helper classes
* Distributed under BSD license
* Visit https://freematics.com for more information
* (C)2016-2020 Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#include "FreematicsMEMS.h"
#include <driver/i2c.h>
#include "utility/ICM_20948_REGISTERS.h"
#include "utility/AK09916_REGISTERS.h"

#define WRITE_BIT         I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT          I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN      0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS     0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL           (i2c_ack_type_t)0x0              /*!< I2C ack value */
#define NACK_VAL          (i2c_ack_type_t)0x1 /*!< I2C nack value */

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation
void CQuaterion::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  uint32_t now = millis();
  deltat = ((float)(now - lastUpdate)/1000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = now;

  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrtf(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f/norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

void CQuaterion::getOrientation(ORIENTATION* ori)
{
     ori->yaw  = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]) * 180.0f / PI;
     ori->pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2])) * 180.0f / PI;
     ori->roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * 180.0f / PI;
}

/*******************************************************************************
  Base I2C MEMS class
*******************************************************************************/

#define WRITE_BIT         I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT          I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN      0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS     0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL           (i2c_ack_type_t)0x0              /*!< I2C ack value */
#define NACK_VAL          (i2c_ack_type_t)0x1 /*!< I2C nack value */

bool MEMS_I2C::initI2C(unsigned long clock)
{
  i2c_port_t i2c_master_port = I2C_NUM_0;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)21;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = (gpio_num_t)22;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = clock;
  conf.clk_flags = 0;
  return i2c_param_config(i2c_master_port, &conf) == ESP_OK &&
    i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0) == ESP_OK;
}

void MEMS_I2C::uninitI2C()
{
  i2c_driver_delete((i2c_port_t)I2C_NUM_0);
}
/*******************************************************************************
  MPU-9250 class functions 
*******************************************************************************/

//==============================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer,
//====== and temperature data
//==============================================================================
void MPU9250::readAccelData(int16_t * destination)
{
  uint8_t rawData[6];   // x/y/z accel register data stored here
  readBytes(ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void MPU9250::readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void MPU9250::readMagData(int16_t * destination)
{
  if(readByteAK(AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
    uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    readBytesAK(AK8963_XOUT_L, 7, rawData);  // Read the six raw data and ST2 registers sequentially into data array
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
   }
 }
}

int16_t MPU9250::readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a 16-bit value
}

bool MPU9250::initAK8963(float * destination)
{
  if (readByteAK(WHO_AM_I_AK8963) != 0x48) {
      return false;
  }

  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByteAK(AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  writeByteAK(AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  // Read the x-, y-, and z-axis calibration values
  /*
  if (!readBytesAK(AK8963_ASAX, 3, &rawData[0], 3000)) {
    return false;
  }
  */
  rawData[0] = readByteAK(AK8963_ASAX);
  rawData[1] = readByteAK(AK8963_ASAY);
  rawData[2] = readByteAK(AK8963_ASAZ);
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.;
  writeByteAK(AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByteAK(AK8963_CNTL, MFS_16BITS << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
  return true;
}

// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.
void MPU9250::calibrateMPU9250(float * gyroBias, float * accelBias)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device
  // Write a one to bit 7 reset bit; toggle reset device
  writeByte(PWR_MGMT_1, 0x80);
  delay(100);

 // get stable time source; Auto select clock source to be PLL gyroscope
 // reference if ready else use the internal oscillator, bits 2:0 = 001
  writeByte(PWR_MGMT_1, 0x01);
  writeByte(PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  writeByte(INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(FIFO_EN, 0x00);      // Disable FIFO
  writeByte(PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(I2C_MST_CTRL, 0x00); // Disable master
  writeByte(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(USER_CTRL, 0x2C);    // Reset FIFO and DMP
  delay(15);

// Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(USER_CTRL, 0x40);   // Enable FIFO
  writeByte(FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(FIFO_COUNTH, 2, &data[0]); // read FIFO sample count

  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

// Push gyro biases to hardware registers
  writeByte(XG_OFFSET_H, data[0]);
  writeByte(XG_OFFSET_L, data[1]);
  writeByte(YG_OFFSET_H, data[2]);
  writeByte(YG_OFFSET_L, data[3]);
  writeByte(ZG_OFFSET_H, data[4]);
  writeByte(ZG_OFFSET_L, data[5]);

// Output scaled gyro biases for display in the main program
  gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
  writeByte(XA_OFFSET_H, data[0]);
  writeByte(XA_OFFSET_L, data[1]);
  writeByte(YA_OFFSET_H, data[2]);
  writeByte(YA_OFFSET_L, data[3]);
  writeByte(ZA_OFFSET_H, data[4]);
  writeByte(ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for display in the main program
   accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
   accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
   accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250::MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
  float factoryTrim[6];
  uint8_t FS = 0;

  writeByte(SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
  writeByte(ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

    readBytes(ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

// Configure the accelerometer for self-test
  writeByte(ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(25);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer

    readBytes(ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation
  writeByte(ACCEL_CONFIG, 0x00);
  writeByte(GYRO_CONFIG,  0x00);
  delay(25);  // Delay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = readByte(SELF_TEST_X_ACCEL); // X-axis accel self-test results
  selfTest[1] = readByte(SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
  selfTest[2] = readByte(SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
  selfTest[3] = readByte(SELF_TEST_X_GYRO);  // X-axis gyro self-test results
  selfTest[4] = readByte(SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
  selfTest[5] = readByte(SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++) {
    destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
    destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
  }
}

void MPU9250::writeByte(uint8_t subAddress, uint8_t data)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( MPU9250_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  // write sub-address and data
  uint8_t buf[2] = {subAddress, data};
  i2c_master_write(cmd, buf, sizeof(buf), ACK_CHECK_EN);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
}

uint8_t MPU9250::readByte(uint8_t subAddress)
{
  // write sub-address
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( MPU9250_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK_DIS);
  i2c_master_write_byte(cmd, subAddress, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  // read data
  uint8_t data = 0;
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( MPU9250_ADDRESS << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &data, NACK_VAL);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return data;
}

bool MPU9250::readBytes(uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  // write sub-address
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( MPU9250_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK_DIS);
  i2c_master_write_byte(cmd, subAddress, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) return false;
  // read data
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( MPU9250_ADDRESS << 1 ) | READ_BIT, ACK_CHECK_EN);
  if (count > 1) {
      i2c_master_read(cmd, dest, count - 1, ACK_VAL);
  }
  i2c_master_read_byte(cmd, dest + count - 1, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret == ESP_OK;
}

void MPU9250::writeByteAK(uint8_t subAddress, uint8_t data)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( AK8963_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  // write sub-address and data
  uint8_t buf[2] = {subAddress, data};
  i2c_master_write(cmd, buf, sizeof(buf), ACK_CHECK_EN);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
}

uint8_t MPU9250::readByteAK(uint8_t subAddress)
{
  // write sub-address
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( AK8963_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK_DIS);
  i2c_master_write_byte(cmd, subAddress, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  // read data
  uint8_t data = 0;
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( AK8963_ADDRESS << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &data, NACK_VAL);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return data;
}

bool MPU9250::readBytesAK(uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  // write sub-address
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( AK8963_ADDRESS << 1 ) | WRITE_BIT, ACK_CHECK_DIS);
  i2c_master_write_byte(cmd, subAddress, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) return false;
  // read data
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( AK8963_ADDRESS << 1 ) | READ_BIT, ACK_CHECK_EN);
  if (count > 1) {
      i2c_master_read(cmd, dest, count - 1, ACK_VAL);
  }
  i2c_master_read_byte(cmd, dest + count - 1, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret == ESP_OK;
}

void MPU9250::init()
{
 // wake up device
  writeByte(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

 // get stable time source
  writeByte(PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(CONFIG, 0x03);

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                    // determined inset in CONFIG above

 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(GYRO_CONFIG); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

 // Set accelerometer full-scale range configuration
  c = readByte(ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer
  writeByte(ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(INT_PIN_CFG, 0x22);
   writeByte(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   delay(100);

}

byte MPU9250::begin(bool fusion)
{
  if (!initI2C(100000)) return 0;
  byte ret = 0;
  for (byte attempt = 0; attempt < 2; attempt++) {
    //float SelfTest[6];
    //MPU9250SelfTest(SelfTest);
    byte c = readByte(WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
    if (c != 0x68 && c != 0x71) continue;
    calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
    init();
    if (c == 0x71 && initAK8963(magCalibration))
      ret = 2;
    else
      ret = 1;
    break;
  }
  if (ret && fusion && !quaterion) {
    quaterion = new CQuaterion;
  }
  return ret;
}

bool MPU9250::read(float* acc, float* gyr, float* mag, float* temp, ORIENTATION* ori)
{
  if (acc) {
    readAccelData(accelCount);
    acc[0] = (float)accelCount[0]*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
    acc[1] = (float)accelCount[1]*aRes; // - accelBias[1];
    acc[2] = (float)accelCount[2]*aRes; // - accelBias[2];
  }
  if (gyr) {
    readGyroData(gyroCount);
    gyr[0] = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
    gyr[1] = (float)gyroCount[1]*gRes;
    gyr[2] = (float)gyroCount[2]*gRes;
  }
  if (mag) {
    float magbias[3];
    magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
    magbias[2] = +125.;  // User environmental x-axis correction in milliGauss

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    readMagData(magCount);
    mag[0] = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
    mag[1] = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];
    mag[2] = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];
  }
  if (temp) {
    int t = readTempData();
    *temp = (float)t / 333.87 + 21;
  }

  if (quaterion && acc && gyr && mag) {
    quaterion->MadgwickQuaternionUpdate(acc[0], acc[1], acc[2], gyr[0]*PI/180.0f, gyr[1]*PI/180.0f, gyr[2]*PI/180.0f,  mag[0],  mag[1], mag[2]);
    quaterion->getOrientation(ori);
  }
  return true;
}

/*******************************************************************************
  ICM-20948 class functions 
*******************************************************************************/

// serif functions for the I2C and SPI classes
ICM_20948_Status_e ICM_20948_write_I2C(uint8_t reg, uint8_t* data, uint32_t len, void* user){
    if(user == NULL){ return ICM_20948_Stat_ParamErr; }
    uint8_t addr = ((ICM_20948_I2C*)user)->_addr;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write(cmd, data, len, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? ICM_20948_Stat_Ok : ICM_20948_Stat_Err;
}

ICM_20948_Status_e ICM_20948_read_I2C(uint8_t reg, uint8_t* buff, uint32_t len, void* user){
    if(user == NULL){ return ICM_20948_Stat_ParamErr; }
    uint8_t addr = ((ICM_20948_I2C*)user)->_addr;

    // write sub-address
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return ICM_20948_Stat_Err;
    // read data
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( addr << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (len > 1) {
      i2c_master_read(cmd, buff, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, buff + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? ICM_20948_Stat_Ok : ICM_20948_Stat_NoData;
}

ICM_20948_AGMT_t ICM_20948::getAGMT                 ( void ){
    status = ICM_20948_get_agmt( &_device, &agmt );
    
    if( _has_magnetometer ){
        getMagnetometerData( &agmt );
    }

    return agmt;
}

float             ICM_20948::magX                ( void ){
    return getMagUT(agmt.mag.axes.x);
}

float             ICM_20948::magY                ( void ){
    return getMagUT(agmt.mag.axes.y);
}

float             ICM_20948::magZ                ( void ){
    return getMagUT(agmt.mag.axes.z);
}

float               ICM_20948::getMagUT            ( int16_t axis_val ){
    return (((float)axis_val)*0.15);
}

float             ICM_20948::accX                ( void ){
    return getAccMG(agmt.acc.axes.x);
}

float             ICM_20948::accY                ( void ){
    return getAccMG(agmt.acc.axes.y);
}

float             ICM_20948::accZ                ( void ){
    return getAccMG(agmt.acc.axes.z);
}

float               ICM_20948::getAccMG            ( int16_t axis_val ){
    switch(agmt.fss.a){
        case 0 : return (((float)axis_val)/16.384); break;
        case 1 : return (((float)axis_val)/8.192); break;
        case 2 : return (((float)axis_val)/4.096); break;
        case 3 : return (((float)axis_val)/2.048); break;
        default : return 0; break;
    }
}

float             ICM_20948::gyrX                ( void ){
    return getGyrDPS(agmt.gyr.axes.x);
}

float             ICM_20948::gyrY                ( void ){
    return getGyrDPS(agmt.gyr.axes.y);
}

float             ICM_20948::gyrZ                ( void ){
    return getGyrDPS(agmt.gyr.axes.z);
}

float               ICM_20948::getGyrDPS            ( int16_t axis_val ){
    switch(agmt.fss.g){
        case 0 : return (((float)axis_val)/131); break;
        case 1 : return (((float)axis_val)/65.5); break;
        case 2 : return (((float)axis_val)/32.8); break;
        case 3 : return (((float)axis_val)/16.4); break;
        default : return 0; break;
    }
}

float             ICM_20948::temp                 ( void ){
    return getTempC(agmt.tmp.val);
}

float               ICM_20948::getTempC             ( int16_t val ){
    return (((float)val)/333.87) + 21;
}



const char* ICM_20948::statusString                 ( ICM_20948_Status_e stat ){
    ICM_20948_Status_e val;
    if( stat == ICM_20948_Stat_NUM){
        val = status;
    }else{
        val = stat;
    }

    switch(val){
        case ICM_20948_Stat_Ok : return "All is well."; break;
        case ICM_20948_Stat_Err : return "General Error"; break;
	    case ICM_20948_Stat_NotImpl : return "Not Implemented"; break;
        case ICM_20948_Stat_ParamErr : return "Parameter Error"; break;
        case ICM_20948_Stat_WrongID : return "Wrong ID"; break;
        case ICM_20948_Stat_InvalSensor : return "Invalid Sensor"; break;
        case ICM_20948_Stat_NoData : return "Data Underflow"; break;
        case ICM_20948_Stat_SensorNotSupported : return "Sensor Not Supported"; break;
        default :
            return "Unknown Status"; break;
        
    }
    return "None";
}



// Device Level
ICM_20948_Status_e	ICM_20948::setBank			    ( uint8_t bank ){
    status =  ICM_20948_set_bank( &_device, bank );
    return status;
}

ICM_20948_Status_e	ICM_20948::swReset			    ( void ){
    status = ICM_20948_sw_reset( &_device );
    return status;
}

ICM_20948_Status_e	ICM_20948::sleep				( bool on ){
    status = ICM_20948_sleep( &_device, on );
    return status;
}

ICM_20948_Status_e	ICM_20948::lowPower			( bool on ){
    status = ICM_20948_low_power( &_device, on );
    return status;
}

ICM_20948_Status_e	ICM_20948::setClockSource	    ( ICM_20948_PWR_MGMT_1_CLKSEL_e source ){
    status = ICM_20948_set_clock_source( &_device, source );
    return status;
}

ICM_20948_Status_e	ICM_20948::checkID			    ( void ){
    status = ICM_20948_check_id( &_device );
    return status;
}

bool	            ICM_20948::dataReady		    ( void ){
    status = ICM_20948_data_ready( &_device );
    if( status == ICM_20948_Stat_Ok ){ return true; }
    return false;
}

uint8_t	            ICM_20948::getWhoAmI		    ( void ){
    uint8_t retval = 0x00;
    status = ICM_20948_get_who_am_i( &_device, &retval );
    return retval;
}

bool                ICM_20948::isConnected         ( void ){
    status = checkID();
    if( status == ICM_20948_Stat_Ok ){ return true; }
    return false;
}


// Internal Sensor Options
ICM_20948_Status_e	ICM_20948::setSampleMode	    ( uint8_t sensor_id_bm, uint8_t lp_config_cycle_mode ){
    status = ICM_20948_set_sample_mode( &_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, (ICM_20948_LP_CONFIG_CYCLE_e)lp_config_cycle_mode );
    return status;
}

ICM_20948_Status_e	ICM_20948::setFullScale 	    ( uint8_t sensor_id_bm, ICM_20948_fss_t fss ){
    status = ICM_20948_set_full_scale( &_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, fss );
    return status;
}

ICM_20948_Status_e	ICM_20948::setDLPFcfg		    ( uint8_t sensor_id_bm, ICM_20948_dlpcfg_t cfg ){
    status = ICM_20948_set_dlpf_cfg( &_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, cfg );
    return status;
}

ICM_20948_Status_e	ICM_20948::enableDLPF		    ( uint8_t sensor_id_bm, bool enable ){
    status = ICM_20948_enable_dlpf( &_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, enable );
    return status;
}

ICM_20948_Status_e	ICM_20948::setSampleRate	    ( uint8_t sensor_id_bm, ICM_20948_smplrt_t smplrt ){
    status = ICM_20948_set_sample_rate( &_device, (ICM_20948_InternalSensorID_bm)sensor_id_bm, smplrt );
    return status;
}





// Interrupts on INT Pin
ICM_20948_Status_e  ICM_20948::clearInterrupts         ( void ){
    ICM_20948_INT_STATUS_t int_stat;
    ICM_20948_INT_STATUS_1_t int_stat_1;

    // read to clear interrupts
    status = ICM_20948_set_bank( &_device, 0 );                                                                                   if( status != ICM_20948_Stat_Ok ){ return status; }
    status = ICM_20948_execute_r( &_device, AGB0_REG_INT_STATUS, (uint8_t*)&int_stat, sizeof(ICM_20948_INT_STATUS_t) );           if( status != ICM_20948_Stat_Ok ){ return status; }
    status = ICM_20948_execute_r( &_device, AGB0_REG_INT_STATUS_1, (uint8_t*)&int_stat_1, sizeof(ICM_20948_INT_STATUS_1_t) );     if( status != ICM_20948_Stat_Ok ){ return status; }

    // todo: there may be additional interrupts that need to be cleared, like FIFO overflow/watermark

    return status;
}


ICM_20948_Status_e  ICM_20948::cfgIntActiveLow         ( bool active_low ){
    ICM_20948_INT_PIN_CFG_t reg;
    status = ICM_20948_int_pin_cfg		( &_device, NULL, &reg );       // read phase
    if(status != ICM_20948_Stat_Ok){ return status; }
    reg.INT1_ACTL = active_low;                                         // set the setting
    status = ICM_20948_int_pin_cfg		( &_device, &reg, NULL );       // write phase
    if(status != ICM_20948_Stat_Ok){ return status; }
    return status;
}

ICM_20948_Status_e  ICM_20948::cfgIntOpenDrain         ( bool open_drain ){
    ICM_20948_INT_PIN_CFG_t reg;
    status = ICM_20948_int_pin_cfg		( &_device, NULL, &reg );       // read phase
    if(status != ICM_20948_Stat_Ok){ return status; }
    reg.INT1_OPEN = open_drain;                                         // set the setting
    status = ICM_20948_int_pin_cfg		( &_device, &reg, NULL );       // write phase
    if(status != ICM_20948_Stat_Ok){ return status; }
    return status;
}

ICM_20948_Status_e  ICM_20948::cfgIntLatch             ( bool latching ){
    ICM_20948_INT_PIN_CFG_t reg;
    status = ICM_20948_int_pin_cfg		( &_device, NULL, &reg );       // read phase
    if(status != ICM_20948_Stat_Ok){ return status; }
    reg.INT1_LATCH_EN = latching;                                       // set the setting
    status = ICM_20948_int_pin_cfg		( &_device, &reg, NULL );       // write phase
    if(status != ICM_20948_Stat_Ok){ return status; }
    return status;
}

ICM_20948_Status_e  ICM_20948::cfgIntAnyReadToClear      ( bool enabled ){
    ICM_20948_INT_PIN_CFG_t reg;
    status = ICM_20948_int_pin_cfg		( &_device, NULL, &reg );       // read phase
    if(status != ICM_20948_Stat_Ok){ return status; }
    reg.INT_ANYRD_2CLEAR = enabled;                                     // set the setting
    status = ICM_20948_int_pin_cfg		( &_device, &reg, NULL );       // write phase
    if(status != ICM_20948_Stat_Ok){ return status; }
    return status;
}

ICM_20948_Status_e  ICM_20948::cfgFsyncActiveLow       ( bool active_low ){
    ICM_20948_INT_PIN_CFG_t reg;
    status = ICM_20948_int_pin_cfg		( &_device, NULL, &reg );       // read phase
    if(status != ICM_20948_Stat_Ok){ return status; }
    reg.ACTL_FSYNC = active_low;                                         // set the setting
    status = ICM_20948_int_pin_cfg		( &_device, &reg, NULL );       // write phase
    if(status != ICM_20948_Stat_Ok){ return status; }
    return status;
}

ICM_20948_Status_e  ICM_20948::cfgFsyncIntMode         ( bool interrupt_mode ){
    ICM_20948_INT_PIN_CFG_t reg;
    status = ICM_20948_int_pin_cfg		( &_device, NULL, &reg );       // read phase
    if(status != ICM_20948_Stat_Ok){ return status; }
    reg.FSYNC_INT_MODE_EN = interrupt_mode;                             // set the setting
    status = ICM_20948_int_pin_cfg		( &_device, &reg, NULL );       // write phase
    if(status != ICM_20948_Stat_Ok){ return status; }
    return status;
}


//      All these individual functions will use a read->set->write method to leave other settings untouched
ICM_20948_Status_e	ICM_20948::intEnableI2C        ( bool enable ){
    ICM_20948_INT_enable_t en;                              // storage
    status = ICM_20948_int_enable( &_device, NULL, &en );   // read phase
    if( status != ICM_20948_Stat_Ok ){ return status; }
    en.I2C_MST_INT_EN = enable;                             // change the setting
    status = ICM_20948_int_enable( &_device, &en, &en );    // write phase w/ readback
    if( status != ICM_20948_Stat_Ok ){ return status; }
    if( en.I2C_MST_INT_EN != enable ){
        status = ICM_20948_Stat_Err;
        return status; 
    }
    return status;
}

ICM_20948_Status_e	ICM_20948::intEnableDMP        ( bool enable ){
    ICM_20948_INT_enable_t en;                              // storage
    status = ICM_20948_int_enable( &_device, NULL, &en );   // read phase
    if( status != ICM_20948_Stat_Ok ){ return status; }
    en.DMP_INT1_EN = enable;                                // change the setting
    status = ICM_20948_int_enable( &_device, &en, &en );    // write phase w/ readback
    if( status != ICM_20948_Stat_Ok ){ return status; }
    if( en.DMP_INT1_EN != enable ){
        status = ICM_20948_Stat_Err;
        return status; 
    }
    return status;
}

ICM_20948_Status_e	ICM_20948::intEnablePLL        ( bool enable ){
    ICM_20948_INT_enable_t en;                              // storage
    status = ICM_20948_int_enable( &_device, NULL, &en );   // read phase
    if( status != ICM_20948_Stat_Ok ){ return status; }
    en.PLL_RDY_EN = enable;                                 // change the setting
    status = ICM_20948_int_enable( &_device, &en, &en );    // write phase w/ readback
    if( status != ICM_20948_Stat_Ok ){ return status; }
    if( en.PLL_RDY_EN != enable ){
        status = ICM_20948_Stat_Err;
        return status; 
    }
    return status;
}

ICM_20948_Status_e	ICM_20948::intEnableWOM        ( bool enable ){
    ICM_20948_INT_enable_t en;                              // storage
    status = ICM_20948_int_enable( &_device, NULL, &en );   // read phase
    if( status != ICM_20948_Stat_Ok ){ return status; }
    en.WOM_INT_EN = enable;                                 // change the setting
    status = ICM_20948_int_enable( &_device, &en, &en );    // write phase w/ readback
    if( status != ICM_20948_Stat_Ok ){ return status; }
    if( en.WOM_INT_EN != enable ){
        status = ICM_20948_Stat_Err;
        return status; 
    }
    return status;
}

ICM_20948_Status_e	ICM_20948::intEnableWOF        ( bool enable ){
    ICM_20948_INT_enable_t en;                              // storage
    status = ICM_20948_int_enable( &_device, NULL, &en );   // read phase
    if( status != ICM_20948_Stat_Ok ){ return status; }
    en.REG_WOF_EN = enable;                                 // change the setting
    status = ICM_20948_int_enable( &_device, &en, &en );    // write phase w/ readback
    if( status != ICM_20948_Stat_Ok ){ return status; }
    if( en.REG_WOF_EN != enable ){
        status = ICM_20948_Stat_Err;
        return status; 
    }
    return status;
}

ICM_20948_Status_e	ICM_20948::intEnableRawDataReady   ( bool enable ){
    ICM_20948_INT_enable_t en;                              // storage
    status = ICM_20948_int_enable( &_device, NULL, &en );   // read phase
    if( status != ICM_20948_Stat_Ok ){ return status; }
    en.RAW_DATA_0_RDY_EN = enable;                          // change the setting
    status = ICM_20948_int_enable( &_device, &en, &en );    // write phase w/ readback
    if( status != ICM_20948_Stat_Ok ){ return status; }
    if( en.RAW_DATA_0_RDY_EN != enable ){
        Serial.println("mismatch error");
        status = ICM_20948_Stat_Err;
        return status; 
    }
    return status;
}

ICM_20948_Status_e	ICM_20948::intEnableOverflowFIFO   ( uint8_t bm_enable ){
    ICM_20948_INT_enable_t en;                              // storage
    status = ICM_20948_int_enable( &_device, NULL, &en );   // read phase
    if( status != ICM_20948_Stat_Ok ){ return status; }
    en.FIFO_OVERFLOW_EN_0 = ((bm_enable >> 0) & 0x01);      // change the settings
    en.FIFO_OVERFLOW_EN_1 = ((bm_enable >> 1) & 0x01);
    en.FIFO_OVERFLOW_EN_2 = ((bm_enable >> 2) & 0x01);
    en.FIFO_OVERFLOW_EN_3 = ((bm_enable >> 3) & 0x01);
    en.FIFO_OVERFLOW_EN_4 = ((bm_enable >> 4) & 0x01);
    status = ICM_20948_int_enable( &_device, &en, &en );    // write phase w/ readback
    if( status != ICM_20948_Stat_Ok ){ return status; }
    return status;
}

ICM_20948_Status_e	ICM_20948::intEnableWatermarkFIFO  ( uint8_t bm_enable ){
    ICM_20948_INT_enable_t en;                              // storage
    status = ICM_20948_int_enable( &_device, NULL, &en );   // read phase
    if( status != ICM_20948_Stat_Ok ){ return status; }
    en.FIFO_WM_EN_0 = ((bm_enable >> 0) & 0x01);            // change the settings
    en.FIFO_WM_EN_1 = ((bm_enable >> 1) & 0x01);
    en.FIFO_WM_EN_2 = ((bm_enable >> 2) & 0x01);
    en.FIFO_WM_EN_3 = ((bm_enable >> 3) & 0x01);
    en.FIFO_WM_EN_4 = ((bm_enable >> 4) & 0x01);
    status = ICM_20948_int_enable( &_device, &en, &en );    // write phase w/ readback
    if( status != ICM_20948_Stat_Ok ){ return status; }
    return status;
}



// Interface Options
ICM_20948_Status_e	ICM_20948::i2cMasterPassthrough 	( bool passthrough ){
    status = ICM_20948_i2c_master_passthrough ( &_device, passthrough );
    return status;
}

ICM_20948_Status_e	ICM_20948::i2cMasterEnable          ( bool enable ){
    status = ICM_20948_i2c_master_enable( &_device, enable );
    return status;
}

ICM_20948_Status_e	ICM_20948::i2cMasterConfigureSlave  ( uint8_t slave, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap ){
    status = ICM_20948_i2c_master_configure_slave 		( &_device, slave, addr, reg, len, Rw, enable, data_only, grp, swap );
    return status;
}

ICM_20948_Status_e 	ICM_20948::i2cMasterSLV4Transaction( uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len, bool Rw, bool send_reg_addr ){
    status = ICM_20948_i2c_master_slv4_txn( &_device, addr, reg, data, len, Rw, send_reg_addr );
    return status;
}
ICM_20948_Status_e	ICM_20948::i2cMasterSingleW        ( uint8_t addr, uint8_t reg, uint8_t data ){
    status = ICM_20948_i2c_master_single_w( &_device, addr, reg, &data );
    return status;
}
uint8_t	ICM_20948::i2cMasterSingleR        ( uint8_t addr, uint8_t reg ){
    uint8_t data;
    status = ICM_20948_i2c_master_single_r( &_device, addr, reg, &data );
    return data;
}










ICM_20948_Status_e  ICM_20948::startupDefault          ( void ){
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

    retval = checkID();
    if( retval != ICM_20948_Stat_Ok ){ status = retval; return status; }

    retval = swReset();
    if( retval != ICM_20948_Stat_Ok ){ status = retval; return status; }
    delay(50);

    retval = sleep( false );
    if( retval != ICM_20948_Stat_Ok ){ status = retval; return status; }

    retval = lowPower( false );
    if( retval != ICM_20948_Stat_Ok ){ status = retval; return status; }
    
    retval = setSampleMode( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous );  // options: ICM_20948_Sample_Mode_Continuous or ICM_20948_Sample_Mode_Cycled
    if( retval != ICM_20948_Stat_Ok ){ status = retval; return status; }                                                                 // sensors: 	ICM_20948_Internal_Acc, ICM_20948_Internal_Gyr, ICM_20948_Internal_Mst

    ICM_20948_fss_t FSS;
    FSS.a = gpm2;       // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
    FSS.g = dps250;     // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
    retval = setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), FSS );  
    if( retval != ICM_20948_Stat_Ok ){ status = retval; return status; }

    ICM_20948_dlpcfg_t dlpcfg;
    dlpcfg.a = acc_d473bw_n499bw;
    dlpcfg.g = gyr_d361bw4_n376bw5;
    retval = setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg );
    if( retval != ICM_20948_Stat_Ok ){ status = retval; return status; }

    retval = enableDLPF( ICM_20948_Internal_Acc, false );
    if( retval != ICM_20948_Stat_Ok ){ status = retval; return status; }
    retval = enableDLPF( ICM_20948_Internal_Gyr, false );
    if( retval != ICM_20948_Stat_Ok ){ status = retval; return status; }

    _has_magnetometer = true;
    retval = startupMagnetometer();
    if(( retval != ICM_20948_Stat_Ok) && ( retval != ICM_20948_Stat_NotImpl )){ status = retval; return status; }
    if( retval == ICM_20948_Stat_NotImpl ){
        // This is a temporary fix. 
        // Ultimately we *should* be able to configure the I2C master to handle the 
        // magnetometer no matter what interface (SPI / I2C) we are using. 

        // Should try testing I2C master functionality on a bare ICM chip w/o TXS0108 level shifter...

        _has_magnetometer = false;
        retval = ICM_20948_Stat_Ok; // reset the retval because we handled it in this cases
    }

    status = retval;
    return status;
}

ICM_20948_Status_e  ICM_20948::startupMagnetometer    ( void ){
    return ICM_20948_Stat_NotImpl; // By default we assume that we cannot access the magnetometer
}

ICM_20948_Status_e  ICM_20948::getMagnetometerData     ( ICM_20948_AGMT_t* pagmt ){
    return ICM_20948_Stat_NotImpl; // By default we assume that we cannot access the magnetometer
}









// direct read/write
ICM_20948_Status_e  ICM_20948::read                 ( uint8_t reg, uint8_t* pdata, uint32_t len){
    status = ICM_20948_execute_r( &_device, reg, pdata, len );
    return status;
}

ICM_20948_Status_e  ICM_20948::write                ( uint8_t reg, uint8_t* pdata, uint32_t len){
    status = ICM_20948_execute_w( &_device, reg, pdata, len );
    return status;
}

byte ICM_20948_I2C::begin(bool fusion){
    // Associate 
	_ad0 = ICM_20948_ARD_UNUSED_PIN;
	_ad0val = false;

  _addr = ICM_20948_I2C_ADDR_AD0;
  if( _ad0val ){ _addr = ICM_20948_I2C_ADDR_AD1; }

    // Set pinmodes
	if(_ad0 != ICM_20948_ARD_UNUSED_PIN){ pinMode(_ad0, OUTPUT); }

    // Set pins to default positions
	if(_ad0 != ICM_20948_ARD_UNUSED_PIN){ digitalWrite(_ad0, _ad0val); }

    if (!initI2C(100000)) return 0;
    
    // Set up the serif
    _serif.write = ICM_20948_write_I2C;
    _serif.read = ICM_20948_read_I2C;
    _serif.user = (void*)this;              // refer to yourself in the user field

    // Link the serif
    _device._serif = &_serif;

    // Perform default startup
    status = startupDefault();
    if( status != ICM_20948_Stat_Ok ){
        return 0;
    }

  if (fusion && !quaterion) {
    quaterion = new CQuaterion;
  }

    return 2;
}

ICM_20948_Status_e  ICM_20948_I2C::startupMagnetometer    ( void ){
    // If using the magnetometer through passthrough:
    i2cMasterPassthrough( true ); // Set passthrough mode to try to access the magnetometer (by default I2C master is disabled but you still have to enable the passthrough)

    // Try to set up magnetometer
    AK09916_CNTL2_Reg_t reg;
    reg.MODE = AK09916_mode_cont_100hz;

    ICM_20948_Status_e retval = writeMag( AK09916_REG_CNTL2, (uint8_t*)&reg, sizeof(AK09916_CNTL2_Reg_t) );
    status = retval;
    if(status == ICM_20948_Stat_Ok){
        _has_magnetometer = true;
    }
    return status;
}


ICM_20948_Status_e ICM_20948_I2C::magWhoIAm( void ){
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

    const uint8_t len = 2;
    uint8_t whoiam[len];
    retval = readMag( AK09916_REG_WIA1, whoiam, len );
    status = retval;
    if( retval != ICM_20948_Stat_Ok ){ return retval; }

    if( (whoiam[0] == (MAG_AK09916_WHO_AM_I >> 8)) && ( whoiam[1] == (MAG_AK09916_WHO_AM_I & 0xFF)) ){
        retval = ICM_20948_Stat_Ok;
        status = retval;
        return status;
    }
    retval = ICM_20948_Stat_WrongID;
    status = retval;
    return status;
}

bool                ICM_20948_I2C::magIsConnected( void ){
    if( magWhoIAm() != ICM_20948_Stat_Ok ){
        return false;
    }
    return true;
}

ICM_20948_Status_e  ICM_20948_I2C::getMagnetometerData     ( ICM_20948_AGMT_t* pagmt ){

    const uint8_t reqd_len = 9; // you must read all the way through the status2 register to re-enable the next measurement
    uint8_t buff[reqd_len];
        
    status = readMag( AK09916_REG_ST1, buff, reqd_len );
    if( status != ICM_20948_Stat_Ok ){
        return status;
    }

    pagmt->mag.axes.x = ((buff[2] << 8) | (buff[1] & 0xFF));
    pagmt->mag.axes.y = ((buff[4] << 8) | (buff[3] & 0xFF));
    pagmt->mag.axes.z = ((buff[6] << 8) | (buff[5] & 0xFF));

    return status;
}

ICM_20948_Status_e ICM_20948_I2C::readMag( uint8_t reg, uint8_t* pdata, uint8_t len ){
	// write sub-address
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, ( MAG_AK09916_I2C_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) return ICM_20948_Stat_Err;
	// read data
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, ( MAG_AK09916_I2C_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
	if (len > 1) {
		i2c_master_read(cmd, pdata, len - 1, ACK_VAL);
	}
	i2c_master_read_byte(cmd, pdata + len - 1, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret == ESP_OK ? ICM_20948_Stat_Ok : ICM_20948_Stat_NoData;
}

ICM_20948_Status_e ICM_20948_I2C::writeMag( uint8_t reg, uint8_t* pdata, uint8_t len ){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( MAG_AK09916_I2C_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write(cmd, pdata, len, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? ICM_20948_Stat_Ok : ICM_20948_Stat_Err;
}

bool ICM_20948_I2C::read(float* acc, float* gyr, float* mag, float* tmp, ORIENTATION* ori)
{
  if(!dataReady() || ICM_20948_get_agmt( &_device, &agmt ) != ICM_20948_Stat_Ok){
    return false;
  }
  if( _has_magnetometer ){
    getMagnetometerData( &agmt );
  }
  if (acc) {
    acc[0] = accX() / 1000;
    acc[1] = accY() / 1000;
    acc[2] = accZ() / 1000;
  }
  if (gyr) {
    gyr[0] = gyrX();
    gyr[1] = gyrY();
    gyr[2] = gyrZ();
  }
  if (mag) {
    mag[0] = magX();
    mag[1] = magY();
    mag[2] = magZ();
  }
  if (tmp) {
    *tmp = temp();
  }
  if (quaterion && acc && gyr && mag) {
    quaterion->MadgwickQuaternionUpdate(acc[0], acc[1], acc[2], gyr[0]*PI/180.0f, gyr[1]*PI/180.0f, gyr[2]*PI/180.0f,  mag[0],  mag[1], mag[2]);
    quaterion->getOrientation(ori);
  }
  return true;
}
