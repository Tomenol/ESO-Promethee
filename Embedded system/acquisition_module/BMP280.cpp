/**	BMP180 pressure sensor Library :
*		This library is an optimized version of the Adafruit BMP280 
*		library : https://github.com/adafruit/Adafruit_BMP280_Library
*			
*	This version allows one to easily configure and retrieve pressure and temperature
*	measurements.

*	Dependencies :
*		- I2C library : used to send / receive data to and from the BMP280
*		
*	Project : ESO Prométhée
*	Author : MAYNADIE Thomas
*
*	Version : 05-11-2021
**/

#include "BMP280.h"

/**
* Default BMP280 class constructor :
*   Retreives pressure / temperature calibration coefficients.
**/
BMP280::BMP280() {}

/**
*	Default BMP280 class initialization function :
*	  Retreives pressure / temperature calibration coefficients and configures the sensor to obtain optimal measurements.
*   
*   Input values :
*     - None
*     
*   Return value : 
*     - Status (uint8_t) : 1 success, 0 error
**/
uint8_t BMP280::initialize()
{  
  uint16_t coeffs[12];
  
  //retreive calibration coefficients
  for(uint8_t i = 0; i < 12; i++)
    if(I2C::read16_LE(BMP280_DEVICE_ADDR, (uint8_t)(BMP280_REGISTER_DIG_T1 + (uint8_t)(2*i)), &(coeffs[i])) == 0) return 0;

  //convert calibration coefficients
  this->dig_T1 = coeffs[0];
  this->dig_T2 = (int16_t)(coeffs[1]);
  this->dig_T3 = (int16_t)(coeffs[2]);
  
  this->dig_P1 = coeffs[3];
  this->dig_P2 = (int16_t)(coeffs[4]);
  this->dig_P3 = (int16_t)(coeffs[5]);
  this->dig_P4 = (int16_t)(coeffs[6]);
  this->dig_P5 = (int16_t)(coeffs[7]);
  this->dig_P6 = (int16_t)(coeffs[8]);
  this->dig_P7 = (int16_t)(coeffs[9]);
  this->dig_P8 = (int16_t)(coeffs[10]);
  this->dig_P9 = (int16_t)(coeffs[11]);

	// device full reset
  if(I2C::write8(BMP280_DEVICE_ADDR, BMP280_REGISTER_SOFTRESET, MODE_SOFT_RESET_CODE) == 0) return 0;

  // sensor configuration
  if(I2C::write8(BMP280_DEVICE_ADDR, BMP280_REGISTER_CONTROL, 0xB7) == 0) return 0;
  if(I2C::write8(BMP280_DEVICE_ADDR, BMP280_REGISTER_CONFIG, 0x01) == 0) return 0;
  
  return 1;
}

/**
*	Retreives pressure and temperature measurement :
* 
*   Input values :
*     - _temperature (double*) : corrected temperature measurement
*     - _pressure (double*) : corrected pressure measurement
*     
*   Return value : 
*     - Status (double) : 1 success, 0 error
**/
uint8_t BMP280::readValues(double *_temperature, double *_pressure)
{
  uint32_t _tmp;
  int32_t  adc_T, adc_P;
  
  int32_t vart_1, vart_2;
  int64_t var1, var2, p;

  uint8_t buff[3];
  
	// retreive temperature
  if(I2C::read32(BMP280_DEVICE_ADDR, BMP280_REGISTER_TEMPDATA, &_tmp) == 0) return 0;  
    
  adc_T = (int32_t)_tmp >> 4;
  
  // retreive temperature
  if(I2C::read32(BMP280_DEVICE_ADDR, BMP280_REGISTER_PRESSUREDATA , &_tmp) == 0) return 0;
  
  adc_P = (int32_t)_tmp >> 4;

  // compute temperature
  vart_1 = ((((adc_T >> 3) - ((int32_t)this->dig_T1 << 1))) * ((int32_t)this->dig_T2)) >> 11;
  vart_2 = (((((adc_T >> 4) - ((int32_t)this->dig_T1)) * ((adc_T >> 4) - ((int32_t)this->dig_T1))) >> 12) * ((int32_t)this->dig_T3)) >> 14;
  *_temperature = (double)(((vart_1 + vart_2) * 5 + 128) >> 8) / 100.0;

  // compute pressure
  var1 = ((int64_t)(vart_1 + vart_2)) - 128000;
  var2 = var1 * var1 * (int64_t)this->dig_P6;
  var2 = var2 + ((var1 * (int64_t)this->dig_P5) << 17);
  var2 = var2 + (((int64_t)this->dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)this->dig_P3) >> 8) + ((var1 * (int64_t)this->dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)this->dig_P1) >> 33;

  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)this->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)this->dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)this->dig_P7) << 4);
  *_pressure = (double)(p / 256.0);

  return 1;
}

/**
*  Retreives altitude from pressure measurements :
* 
*   Input values :
*     - _pressure (double*) : corrected pressure measurement
*     
*   Return value : 
*     - altitude (double) : altitude in meters from reference point
**/
double BMP280::getAltitude(double _P, double _T)
{
  return this->alt0 - (double)(R_PERFECT_GAZ * _T / (G_0 * M_AIR)) * log(_P/this->P0);
}

/**
*  Sets reference altitude from pressure measurements :
* 
*   Input values :
*     - None
*     
*   Return value : 
*     - Status (uint8_t) : 1 success, 0 error
**/
uint8_t BMP280::setReferenceAltitude(double _alt)
{
  double P, T;

  if(readValues(&T, &P) == 0) return 0;
  
  this->P0 = P;
  this->alt0 = _alt;
  
  return 1;
}
