/*  BMP180 pressure sensor Library :
*   This library is an optimized version of the Adafruit BMP280 
*   library : https://github.com/adafruit/Adafruit_BMP280_Library
*     
* This version allows one to easily configure and retrieve pressure and temperature
* measurements.

* Dependencies :
*   - I2C library : used to send / receive data to and from the BMP280
*   
* Project : ESO Prométhée
* Author : MAYNADIE Thomas
*
* Version : 05-11-2021
*/

#include "Arduino.h"
#include "I2C.h"

#ifndef __BMP280_H__
#define __BMP280_H__

#define BMP280_DEVICE_ADDR						  0x76	
#define BMP280_DEVICE_ADDR_ALT					0x77	
#define BMP280_DEVICE_ID						    0x58	
	
#define BMP280_REGISTER_DIG_T1 					0x88
#define BMP280_REGISTER_DIG_T2 					0x8A
#define BMP280_REGISTER_DIG_T3 					0x8C
#define BMP280_REGISTER_DIG_P1 					0x8E
#define BMP280_REGISTER_DIG_P2 					0x90
#define BMP280_REGISTER_DIG_P3 					0x92
#define BMP280_REGISTER_DIG_P4 					0x94
#define BMP280_REGISTER_DIG_P5 					0x96
#define BMP280_REGISTER_DIG_P6 					0x98
#define BMP280_REGISTER_DIG_P7 					0x9A
#define BMP280_REGISTER_DIG_P8 					0x9C
#define BMP280_REGISTER_DIG_P9 					0x9E
#define BMP280_REGISTER_CHIPID 					0xD0
#define BMP280_REGISTER_VERSION 				0xD1
#define BMP280_REGISTER_SOFTRESET 			0xE0
#define BMP280_REGISTER_CAL26 					0xE1
#define BMP280_REGISTER_STATUS 					0xF3
#define BMP280_REGISTER_CONTROL 				0xF4
#define BMP280_REGISTER_CONFIG 					0xF5
#define BMP280_REGISTER_PRESSUREDATA 		0xF7
#define BMP280_REGISTER_TEMPDATA        0xFA

#define MODE_SOFT_RESET_CODE            0xB6

#define G_0                             9.8044          // m/s2
#define M_AIR                           28.965338e-3     // kg/mol
#define R_PERFECT_GAZ                   8.3144621       // J/kg/mol

class BMP280
{
	private:
	  // calibration coefficients 
        uint16_t dig_T1;
        int16_t dig_T2;
        int16_t dig_T3;
        
        uint16_t dig_P1;
        int16_t dig_P2;
        int16_t dig_P3;
        int16_t dig_P4;
        int16_t dig_P5;
        int16_t dig_P6;
        int16_t dig_P7;
        int16_t dig_P8;
        int16_t dig_P9;

        // altitude measurements
        double P0;
        double alt0;
		
	public:  
		BMP280();

		uint8_t initialize();
        uint8_t readValues(double *_temperature, double *_pressure);

        // altitude measurements
        double getAltitude(double _P, double _T);
        uint8_t setReferenceAltitude(double _alt);
};

#endif
