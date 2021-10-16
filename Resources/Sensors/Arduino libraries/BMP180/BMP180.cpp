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

#include "BMP180.h"

BMP180::BMP180() 
{
  this->oss = 0;
}


/*
*	default BMP180 class constructor
*	Retreives pressure / temperature calibration coefficients
*/
void BMP180::initialize()
{
	this->AC1 = static_cast<int16_t>(I2C::readBurstRegValue(BMP180_DEVICE_ADDR, BMP180_COEFF_AC1_REG_ADDR, BMP180_COEFF_REG_SIZE));
	this->AC2 = static_cast<int16_t>(I2C::readBurstRegValue(BMP180_DEVICE_ADDR, BMP180_COEFF_AC2_REG_ADDR, BMP180_COEFF_REG_SIZE));
	this->AC3 = static_cast<int16_t>(I2C::readBurstRegValue(BMP180_DEVICE_ADDR, BMP180_COEFF_AC3_REG_ADDR, BMP180_COEFF_REG_SIZE));
	this->AC4 = I2C::readBurstRegValue(BMP180_DEVICE_ADDR, BMP180_COEFF_AC4_REG_ADDR, BMP180_COEFF_REG_SIZE);
	this->AC5 = I2C::readBurstRegValue(BMP180_DEVICE_ADDR, BMP180_COEFF_AC5_REG_ADDR, BMP180_COEFF_REG_SIZE);
	this->AC6 = I2C::readBurstRegValue(BMP180_DEVICE_ADDR, BMP180_COEFF_AC6_REG_ADDR, BMP180_COEFF_REG_SIZE);
	this->B_1 = static_cast<int16_t>(I2C::readBurstRegValue(BMP180_DEVICE_ADDR, BMP180_COEFF_B1_REG_ADDR, BMP180_COEFF_REG_SIZE));
	this->B_2 = static_cast<int16_t>(I2C::readBurstRegValue(BMP180_DEVICE_ADDR, BMP180_COEFF_B2_REG_ADDR, BMP180_COEFF_REG_SIZE));
	this->MB = static_cast<int16_t>(I2C::readBurstRegValue(BMP180_DEVICE_ADDR, BMP180_COEFF_MB_REG_ADDR, BMP180_COEFF_REG_SIZE));
	this->MC = static_cast<int16_t>(I2C::readBurstRegValue(BMP180_DEVICE_ADDR, BMP180_COEFF_MC_REG_ADDR, BMP180_COEFF_REG_SIZE));
	this->MD = static_cast<int16_t>(I2C::readBurstRegValue(BMP180_DEVICE_ADDR, BMP180_COEFF_MD_REG_ADDR, BMP180_COEFF_REG_SIZE));
	
	delay(100);
	
	// device full reset
	I2C::writeRegValue(BMP180_DEVICE_ADDR, BMP180_FACTORY_RESET_REG_ADDR, BMP180_FACTORY_RESET_REG_VALUE);
}

/*
*	Retreives raw temperature measurement -> 5ms (oss = 0)
*
*	Return value : 
*		- (int32_t) raw temperature measurement
*/	
int32_t BMP180::getRawTemperature()
{
	I2C::writeRegValue(BMP180_DEVICE_ADDR, BMP180_REQUEST_MEASUREMENTS_REG_ADDR, BMP180_REQUEST_MEASUREMENTS_TEMPERATURE);
	delay(5);

	return static_cast<int32_t>(I2C::readBurstRegValue(BMP180_DEVICE_ADDR, BMP180_MEASUREMENT_RESULTS_REG_ADDR, BMP180_TEMP_MEASUREMENT_REG_SIZE));
}

/*
*	Retreives raw pressure measurement -> 5ms (oss = 0)
*
*	Return value : 
*		- (int32_t) raw pressure measurement
*/	
int32_t BMP180::getRawPressure()
{
	I2C::writeRegValue(BMP180_DEVICE_ADDR, BMP180_REQUEST_MEASUREMENTS_REG_ADDR, BMP180_REQUEST_MEASUREMENTS_PRESSURE | (oss << 6));
	delay(5);

	return static_cast<int32_t>(I2C::readBurstRegValue(BMP180_DEVICE_ADDR, BMP180_MEASUREMENT_RESULTS_REG_ADDR, BMP180_PRESSURE_MEASUREMENT_REG_SIZE)) >> (8 - this->oss);
}

/*
*	Computes corrected temperature and pressure measurements
*
*	input values : 
*		- _temperature (int32_t*) : address of the corrected temperature measurement result
*		- _pressure (int32_t*) : address of the corrected pressure measurement result
*/	
void BMP180::get_temperature_and_pressure(int32_t* _temperature, int32_t* _pressure)
{
	int32_t UT = getRawTemperature();
	int32_t UP = getRawPressure();

	// temperature correction
	int32_t X1 = ((UT - static_cast<int32_t>(this->AC6)) * static_cast<int32_t>(this->AC5)) >> 15;
	int32_t X2 = (static_cast<int32_t>(this->MC) << 11) / (X1 + static_cast<int32_t>(this->MD));
	uint32_t B5 = X1 + X2;
	
	*_temperature = (B5 + 8) >> 4;
	
	// pressure correction
	int32_t p = 0;
	int32_t B6 = B5 - 4000;
	
	X1 = (static_cast<int32_t>(this->B_2) * ((B6 * B6) >> 12)) >> 11;
	X2 = (static_cast<int32_t>(this->AC2) * B6) >> 11;
	
	int32_t X3 = X1 + X2;
	int32_t B3 = ((((static_cast<int32_t>(this->AC1) << 2) + X3) << this->oss) + 2) >> 2;
	
	X1 = (static_cast<int32_t>(this->AC3) * B6) >> 13;
	X2 = (static_cast<int32_t>(this->B_1) * ((B6 * B6) >> 12)) >> 16;
	X3 = (X1 + X2 + 2) >> 2;
	
	uint32_t B4 = static_cast<uint32_t>(this->AC4) * (static_cast<uint32_t>(X3 + 32768)) >> 15;
	uint32_t B7 = static_cast<uint32_t>(UP - B3) * (50000 >> this->oss);
	
	if (B7 < 0x80000000)
		(*_pressure) = (B7 << 1) / B4;
	else
		(*_pressure) = (B7 / B4) << 1;
	
	X1 = ((*_pressure) >> 8) * ((*_pressure) >> 8);
	X1 = (X1 * 3038) >> 16;
	X2 = (-7357 * (*_pressure)) >> 16;
	
	(*_pressure) = (*_pressure) + ((X1 + X2 + 3791) >> 4);
}
