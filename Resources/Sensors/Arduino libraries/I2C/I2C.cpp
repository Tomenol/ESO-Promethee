/*	I2C Library :
*		This small library allows one to easily send / receive register 
*		commands / values. 
*		
*	Project : ESO Prométhée
*	Author : MAYNADIE Thomas
*
*	Version : 16-10-2021
*/

#include "I2C.h"

/*
*	Initializes the I2C bus
*/
void I2C::begin()
{
	Wire.begin();
}

/*
*	Retreives the register value at a given address.
*
*	Input values :
*		- _dev_addr (uint8_t) : target device address on the I2C bus
*		- _reg_addr (uint8_t) : target register address (refer to the device's register map)
*
*	Return value : 
*		- register value (uint8_t)
*/	
uint8_t I2C::readRegValue(uint8_t _dev_addr, uint8_t _reg_addr)
{
	uint32_t data = 0;

  Wire.beginTransmission(_dev_addr);
  Wire.write(_reg_addr);
  Wire.endTransmission();

  Wire.beginTransmission(_dev_addr);
  Wire.requestFrom(_dev_addr, (uint8_t) 1);
  data = Wire.read();
  Wire.endTransmission();
  return data;
}

/*
*	Retreives multiple register values.
*
*	Input values :
*		- _dev_addr (uint8_t) : target device address on the I2C bus
*		- _starting_reg_addr (uint8_t) : address of the first register to read (refer to the device's register map)
*		- _len (uint8_t) : number of registers to read
*
*	Return value : 
*		- register values (uint32_t)
*/	
uint32_t I2C::readBurstRegValue(uint8_t _dev_addr, uint8_t _starting_reg_addr, uint8_t _len)
{
	uint32_t data = 0;

	Wire.beginTransmission(_dev_addr);
	Wire.write(_starting_reg_addr);
	Wire.endTransmission();
  
  Wire.beginTransmission(_dev_addr);
	Wire.requestFrom(_dev_addr, _len);

	for (uint8_t i = 0; i < _len; i++)
	{
		data <<= 8;
		data |= Wire.read();
	}

	return data;
}

/*
*	Overwrites a register with a given value.
*
*	Input values :
*		- _dev_addr (uint8_t) : target device address on the I2C bus
*		- _reg_addr (uint8_t) : target register address (refer to the device's register map)
*		- _reg_value (uint8_t) : new register value
*/	
void I2C::writeRegValue(uint8_t _dev_addr, uint8_t _reg_addr, uint8_t _reg_value)
{
	Wire.beginTransmission(_dev_addr);
	
	Wire.write(_reg_addr);
	Wire.write(_reg_value);
	
	Wire.endTransmission();
}

/*
*	Modifies a given number of bits of a register.
*
*	Input values :
*		- _dev_addr (uint32_t) : target device address on the I2C bus
*		- _reg_addr (uint32_t) : target register address (refer to the device's register map)
*		- _msb_start (uint32_t) : position of the MSB of the word to overwrite in the register
*		- _len (uint32_t) : number of bits to write
*		- _bits (uint32_t) : bit values
*/	
void I2C::writeBits(uint8_t _dev_addr, uint8_t _reg_addr, uint8_t _msb_start, uint8_t _len, uint8_t _bits)
{
	uint8_t original_data = readRegValue(_dev_addr, _reg_addr);
	uint8_t mask = ((1 << _len) - 1) << (_msb_start - _len + 1);
  
	_bits <<= (_msb_start - _len + 1);
	_bits &= mask;
	
	original_data &= ~(mask);
	original_data |= _bits;
  
	writeRegValue(_dev_addr, _reg_addr, original_data);
}
