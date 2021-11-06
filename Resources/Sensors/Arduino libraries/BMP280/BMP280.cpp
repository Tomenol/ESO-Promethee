/*	BMP180 pressure sensor Library :
*		This library is an optimized version of the arduino BMP180 
*		library : 
*			
*	This version allows one to easily configure and retrieve pressure and temperature
*	measurements.

*	Dependencies :
*		- I2C library : used to send / receive data to and from the BMP180
*		
*	Project : ESO Prométhée
*	Author : MAYNADIE Thomas
*
*	Version : 16-10-2021
*/

#include "BMP280.h"

BMP280::BMP280() 
{
  this->oss = 0;
}


/*
*	default BMP180 class constructor
*	Retreives pressure / temperature calibration coefficients
*/
void BMP280::initialize()
{
	this->dig_T1 = I2C::readBurstRegValue(BMP280_DEVICE_ADDR, BMP280_REGISTER_DIG_T1, BMP180_COEFF_REG_SIZE);
	this->dig_T2 = static_cast<int16_t>(I2C::readBurstRegValue(BMP280_DEVICE_ADDR, BMP280_REGISTER_DIG_T2, BMP180_COEFF_REG_SIZE);
	this->dig_T3 = static_cast<int16_t>(I2C::readBurstRegValue(BMP280_DEVICE_ADDR, BMP280_REGISTER_DIG_T3, BMP180_COEFF_REG_SIZE);

	this->dig_P1 = I2C::readBurstRegValue(BMP280_DEVICE_ADDR, BMP280_REGISTER_DIG_P1, BMP180_COEFF_REG_SIZE);
	this->dig_P2 = static_cast<int16_t>(I2C::readBurstRegValue(BMP280_DEVICE_ADDR, BMP280_REGISTER_DIG_P2, BMP180_COEFF_REG_SIZE);
	this->dig_P3 = static_cast<int16_t>(I2C::readBurstRegValue(BMP280_DEVICE_ADDR, BMP280_REGISTER_DIG_P3, BMP180_COEFF_REG_SIZE);
	this->dig_P4 = static_cast<int16_t>(I2C::readBurstRegValue(BMP280_DEVICE_ADDR, BMP280_REGISTER_DIG_P4, BMP180_COEFF_REG_SIZE);
	this->dig_P5 = static_cast<int16_t>(I2C::readBurstRegValue(BMP280_DEVICE_ADDR, BMP280_REGISTER_DIG_P5, BMP180_COEFF_REG_SIZE);
	this->dig_P9 = static_cast<int16_t>(I2C::readBurstRegValue(BMP280_DEVICE_ADDR, BMP280_REGISTER_DIG_P9, BMP180_COEFF_REG_SIZE);
	this->dig_P6 = static_cast<int16_t>(I2C::readBurstRegValue(BMP280_DEVICE_ADDR, BMP280_REGISTER_DIG_P6, BMP180_COEFF_REG_SIZE);
	this->dig_P7 = static_cast<int16_t>(I2C::readBurstRegValue(BMP280_DEVICE_ADDR, BMP280_REGISTER_DIG_P7, BMP180_COEFF_REG_SIZE);
	this->dig_P8 = static_cast<int16_t>(I2C::readBurstRegValue(BMP280_DEVICE_ADDR, BMP280_REGISTER_DIG_P8, BMP180_COEFF_REG_SIZE);

	delay(100);
	
	// device full reset
	I2C::writeRegValue(BMP280_DEVICE_ADDR, BMP180_FACTORY_RESET_REG_ADDR, BMP180_FACTORY_RESET_REG_VALUE);

	// write values for config
	// oss = 
	// osrs_p = x16 -> 111
	// osrs_t = x2 -> 010
	// IIR filter coeff = 16 -> 111
	// Normal mode -> 10
	// t_stby = 0.05 -> 000
	I2C::writeRegValue(BMP280_DEVICE_ADDR, BMP280_REGISTER_CONFIG, 7, 6, 0x07);
	I2C::writeRegValue(BMP280_DEVICE_ADDR, BMP280_REGISTER_CONFIG, 0, 1, 0x00);
	I2C::writeRegValue(BMP280_DEVICE_ADDR, BMP280_REGISTER_CONTROL,0x5E);
}

/*
*	Retreives raw temperature measurement -> 5ms (oss = 0)
*
*	Return value : 
*		- (int32_t) raw temperature measurement
*/	
int32_t BMP180::readValues()
{
	
}
