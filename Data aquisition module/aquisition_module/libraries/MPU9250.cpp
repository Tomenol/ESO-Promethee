/*	MPU9250 MEMS IMU Library :
*		This library is an optimized version of the arduino MPU9250 
*		library : https://github.com/ElectronicCats/mpu6050
*			
*	This version allows one to easily configure and retrieve accelerometer
*	and gyroscope measurements.

*	Dependencies :
*		- I2C library : used to send / receive data to and from the MPU9250
*		
*	Project : ESO Prométhée
*	Author : MAYNADIE Thomas
*
*	Version : 16-10-2021
*/

#include "MPU9250.h"

/*
*	default MPU9250 class constructor
*
*	Input values :
*		- device address : either 0x68 or 0x69
*/
MPU9250::MPU9250(uint8_t _dev_address)
{
	this->dev_address = _dev_address;
}

/*
*	MPU9250 initialization :
*		- set power management clock selection to X_GYRO
*		- set gyroscope config to +/-500°/s
*		- set accelerometer config to +/-16g
*   	- disable sleep mode
*
*	TODO : option for the user to choose resolution ?
*/
uint8_t MPU9250::initialize()
{
	if(this->reset_device() == 0) return 0;

	if(I2C::write8(this->dev_address, MPU9250_RA_SMPLRT_DIV, MPU9250_RV_SMPLRT_DIV) == 0) return 0;
	if(I2C::write8(this->dev_address, MPU9250_RA_CONFIG, MPU9250_RV_CONFIG) == 0) return 0;
	if(I2C::write8(this->dev_address, MPU9250_RA_GYRO_CONFIG, MPU9250_RV_GYRO_CONFIG) == 0) return 0;
	if(I2C::write8(this->dev_address, MPU9250_RA_ACCEL_CONFIG, MPU9250_RV_ACCEL_CONFIG) == 0) return 0;
	if(I2C::write8(this->dev_address, MPU9250_RA_ACCEL_CONFIG_2, MPU9250_RV_ACCEL_CONFIG_2) == 0) return 0;
	if(I2C::write8(this->dev_address, MPU9250_RA_ACCEL_FIFO_EN, MPU9250_RV_ACCEL_FIFO_EN) == 0) return 0;
	if(I2C::write8(this->dev_address, MPU9250_RA_ACCEL_USER_CTRL, MPU9250_RV_ACCEL_USER_CTRL) == 0) return 0;
	if(I2C::write8(this->dev_address, MPU9250_RA_ACCEL_PWR_MGMT, MPU9250_RV_ACCEL_PWR_MGMT) == 0) return 0;
  if(I2C::write8(this->dev_address, MPU9250_RA_ACCEL_PWR_MGMT_2, MPU9250_RV_ACCEL_PWR_MGMT_2) == 0) return 0;
  if(I2C::write8(this->dev_address, MPU9250_RA_ACCEL_PWR_MGMT_2, MPU9250_RV_ACCEL_PWR_MGMT_2) == 0) return 0;

  if(I2C::write8(this->dev_address, MPU9250_RA_INT_PIN_CFG, MPU9250_RV_INT_PIN_CFG) == 0) return 0;

  if(I2C::write8(AK8963_SLAVE_ADDRESS, AK8963_CNTL1, 0x00) == 0) return 0;
  delay(1);

  // get magnetometer calibration coefficients
  if(this->get_magnetometer_coefficients() == 0) return 0;

  if(I2C::write8(AK8963_SLAVE_ADDRESS, AK8963_CNTL1, 0x16) == 0) return 0; // continuous @ 100Hz and 16bits
  
	return 1;
}

uint8_t MPU9250::get_magnetometer_coefficients()
{
  uint8_t _data[3];
  if(I2C::write8(AK8963_SLAVE_ADDRESS, AK8963_CNTL1, 0x0F) == 0) return 0;
  delay(1);
  
  if(I2C::readBurstRegValue(AK8963_SLAVE_ADDRESS, AK8963_ASAX, 3, _data) == 0) return 0;
  for(uint8_t i = 0; i < 3; i++)
    this->magCoeff[i] = (double)(_data[i] - 128) / 256.0 + 1.0;

  if(Wire.endTransmission() > 0) return 0;  

  if(I2C::write8(AK8963_SLAVE_ADDRESS, AK8963_CNTL1, 0x00) == 0) return 0;
  delay(1);

  return 1;
}

uint8_t MPU9250::reset_device()
{
	if(I2C::writeBits(this->dev_address, MPU9250_RA_ACCEL_PWR_MGMT, 0x07, 0x01, 0x01) == 0) return 0;

	delay(10);

	return 1;
}

/*
*	Read accelerometer measurements
*
*	Return value : 
*		- 3x1 floating point array containing the X, Y and Z specific force measurements
*/
uint8_t MPU9250::getRawAccelerationVector(double* _accel)
{
    Wire.beginTransmission(this->dev_address);
    Wire.write(MPU9250_RA_ACCEL_XOUT_H);
    if(Wire.endTransmission() > 0) return 0;

    Wire.requestFrom(this->dev_address, (uint8_t) 6);
    for(uint8_t i = 0; i < 3; i++)
    {
      int16_t data = static_cast<int16_t>(((uint16_t)Wire.read()) << 0x08 | (uint16_t)Wire.read());
      _accel[i] = (double)data * LSB_ACCEL_MEASUREMENT_16G;
    }
    
    if(Wire.endTransmission() > 0) return 0;

   return 1;
}

/*
*	Read gyroscope measurements
*
*	Return value : 
*		- 3x1 floating point array containing the X, Y and Z angular velocity measurements
*/
uint8_t MPU9250::getRawAngularVelocityVector(double* _angular_vel)
{
  Wire.beginTransmission(this->dev_address);
  Wire.write(MPU9250_RA_GYRO_XOUT_H);
  if(Wire.endTransmission() > 0) return 0;
  
  Wire.beginTransmission(this->dev_address);
  Wire.requestFrom(this->dev_address, (uint8_t) 6);
  
  for (uint8_t i = 0; i < 3; i++)
  {
    int16_t data = static_cast<int16_t>(((uint16_t)Wire.read()) << 0x08 | (uint16_t)Wire.read());
    _angular_vel[i] = (double)data * LSB_GYRO_MEASUREMENT_2000; 
  }
  
  if(Wire.endTransmission() > 0) return 0;
  
  return 1;
}

uint8_t MPU9250::getRawMagVector(double* _mag)
{
  uint8_t data[8];

  if(I2C::readBurstRegValue(AK8963_SLAVE_ADDRESS, AK8963_HXL-1, 8, data) == 0) return 0;

  for(uint8_t i = 0; i < 3; i++)
    _mag[i] = (double)(int16_t)((uint16_t)data[2*i+2] << 8 | data[2*i+1]) * this->magCoeff[i] * (double)LSB_MAG_MEASUREMENT; // get data on i2c Bus

 return 1;
}
