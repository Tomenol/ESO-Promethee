/*	MPU6050 MEMS IMU Library :
*		This library is an optimized version of the arduino MPU6050 
*		library : https://github.com/ElectronicCats/mpu6050
*			
*	This version allows one to easily configure and retrieve accelerometer
*	and gyroscope measurements.

*	Dependencies :
*		- I2C library : used to send / receive data to and from the MPU6050
*		
*	Project : ESO Prométhée
*	Author : MAYNADIE Thomas
*
*	Version : 16-10-2021
*/

#include "MPU6050.h"

/*
*	default MPU6050 class constructor
*
*	Input values :
*		- device address : either 0x68 or 0x69
*/
MPU6050::MPU6050(uint8_t _dev_address)
{
	this->dev_address = _dev_address;
	
	for(uint8_t i = 0; i < 3; i++)
	{
		this->raw_accel[i] = 0;
		this->raw_angular_vel[i] = 0;
	}
}

/*
*	MPU6050 initialization :
*		- set power management clock selection to X_GYRO
*		- set gyroscope config to +/-500°/s
*		- set accelerometer config to +/-16g
*   	- disable sleep mode
*
*	TODO : option for the user to choose resolution ?
*/
void MPU6050::initialize()
{
  I2C::writeBits(this->dev_address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
  I2C::writeBits(this->dev_address, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, 0x01);
  I2C::writeBits(this->dev_address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, 0x03);
  I2C::writeBits(this->dev_address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 1, 0);
}

/*
*	Read accelerometer measurements
*
*	Return value : 
*		- 3x1 floating point array containing the X, Y and Z specific force measurements
*/
float* MPU6050::getRawAccelerationVector()
{
  Wire.beginTransmission(this->dev_address);
  Wire.write(MPU6050_RA_ACCEL_XOUT_H);
  Wire.endTransmission();
 
  Wire.beginTransmission(this->dev_address);
  Wire.requestFrom(this->dev_address, (uint8_t) 6);

  for (int j = 0; j < 3; j++)
    this->raw_accel[j] = (float)(Wire.read() << 8 | Wire.read()) / (float)LSB_ACCEL_MEASUREMENT; // get data on i2c Bus

  Wire.endTransmission();

  return this->raw_accel;
}

/*
*	Read gyroscope measurements
*
*	Return value : 
*		- 3x1 floating point array containing the X, Y and Z angular velocity measurements
*/
float* MPU6050::getRawAngularVelocityVector()
{
  Wire.beginTransmission(this->dev_address);
  Wire.write(MPU6050_RA_GYRO_XOUT_H);
  Wire.endTransmission();
 
  Wire.beginTransmission(this->dev_address);
  Wire.requestFrom(this->dev_address, (uint8_t) 6);

  for (int j = 0; j < 3; j++)
    this->raw_angular_vel[j] = (float)(Wire.read() << 8 | Wire.read()) / (float)LSB_GYRO_MEASUREMENT; // get data on i2c Bus

  Wire.endTransmission();

  return this->raw_angular_vel;
}
