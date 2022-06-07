/**  I2C Library :
*   This small library allows one to easily send / receive register 
*   commands / values. 
*   
* Project : ESO Prométhée
* Author : MAYNADIE Thomas
*
* Version : 05-11-2021
**/

#ifndef I2C_H_
#define I2C_H_

#include <Wire.h>
#include "Arduino.h"

class I2C
{
	public:
		static void begin();
		
		static uint8_t read8(uint8_t _dev_addr, uint8_t _reg_addr, uint8_t *_data);
	    static uint8_t readBurstRegValue(uint8_t _dev_addr, uint8_t _starting_reg_addr, uint8_t _len, uint8_t* _buffer);
	    static uint8_t read16(uint8_t _dev_addr, uint8_t _reg_addr, uint16_t* _data);
	    static uint8_t read16_LE(uint8_t _dev_addr, uint8_t _reg_addr, uint16_t* _data);
	    static uint8_t read32(uint8_t _dev_addr, uint8_t _starting_reg_addr, uint32_t* _data);
		
		static uint8_t write8(uint8_t _dev_addr, uint8_t _reg_addr, uint8_t _reg_value);
		static uint8_t writeBits(uint8_t _dev_addr, uint8_t _reg_addr, uint8_t _msb_start, uint8_t _len, uint8_t _bits);

	private:
		I2C() {}
};
#endif 
