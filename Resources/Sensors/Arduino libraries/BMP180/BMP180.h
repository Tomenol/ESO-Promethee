/*	BMP180 pressure sensor Library :
*		This library is an optimized version of the arduino BMP180 library 
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

#include "Arduino.h"
#include "I2C.h"

#define BMP180_DEVICE_ADDR						0x77	
	
#define BMP180_MEASUREMENT_RESULTS_REG_ADDR		0xF6	
#define BMP180_REQUEST_MEASUREMENTS_REG_ADDR	0xF4	

#define BMP180_REQUEST_MEASUREMENTS_TEMPERATURE	0x2E	
#define BMP180_TEMP_MEASUREMENT_REG_SIZE		0x02
	
#define BMP180_REQUEST_MEASUREMENTS_PRESSURE	0x34	
#define BMP180_PRESSURE_MEASUREMENT_REG_SIZE	0x02	

#define BMP180_COEFF_AC1_REG_ADDR				0xAA	
#define BMP180_COEFF_AC2_REG_ADDR				0xAC	
#define BMP180_COEFF_AC3_REG_ADDR				0xAE	
#define BMP180_COEFF_AC4_REG_ADDR				0xB0	
#define BMP180_COEFF_AC5_REG_ADDR				0xB2	
#define BMP180_COEFF_AC6_REG_ADDR				0xB4	
#define BMP180_COEFF_B1_REG_ADDR				0xB6	
#define BMP180_COEFF_B2_REG_ADDR				0xB8	
#define BMP180_COEFF_MB_REG_ADDR				0xBA	
#define BMP180_COEFF_MC_REG_ADDR				0xBC	
#define BMP180_COEFF_MD_REG_ADDR				0xBE	

#define BMP180_COEFF_REG_SIZE					0x02	

#define BMP180_FACTORY_RESET_REG_ADDR			0xE0	
#define BMP180_FACTORY_RESET_REG_VALUE			0xB6	
		

#ifndef __BMP180_H__
#define __BMP180_H__

class BMP180
{
	private:
		int16_t AC1;
		int16_t AC2;
		int16_t AC3;
		int16_t MB;
		int16_t MC;
		int16_t MD;
		int16_t B_1;
		int16_t B_2;
		
		uint16_t AC4;
		uint16_t AC5;
		uint16_t AC6;
		
		uint8_t oss;
		
	public:
		BMP180();

    void initialize();
		int32_t getRawTemperature();
		int32_t getRawPressure();
		
		void get_temperature_and_pressure(int32_t *_temperature, int32_t *_pressure);
	
	
};

#endif
