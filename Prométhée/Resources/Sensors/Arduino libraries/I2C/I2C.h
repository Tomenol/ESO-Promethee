/*	I2C Library :
*		This small library allows one to easily send / receive register 
*		commands / values. 
*		
*	Project : ESO Prométhée
*	Author : MAYNADIE Thomas
*
*	Version : 16-10-2021
*/

#ifndef I2C_H_
#define I2C_H_

#include <Wire.h>
#include "Arduino.h"

class I2C
{
	public:
		static void begin();
		
		static uint8_t readRegValue(uint8_t _dev_addr, uint8_t _reg_addr);
		static uint32_t readBurstRegValue(uint8_t _dev_addr, uint8_t _starting_reg_addr, uint8_t _len);
		
		static void writeRegValue(uint8_t _dev_addr, uint8_t _reg_addr, uint8_t _reg_value);
		static void writeBits(uint8_t _dev_addr, uint8_t _reg_addr, uint8_t _msb_start, uint8_t _len, uint8_t _bits);

	private:
		I2C() {}
};
#endif 