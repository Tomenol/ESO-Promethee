/**	I2C Library :
*		This small library allows one to easily send / receive register 
*		commands / values. 
*		
*	Project : ESO Prométhée
*	Author : MAYNADIE Thomas
*
*	Version : 05-11-2021
**/

#include "I2C.h"

/**
*	Initializes the I2C bus
**/
void I2C::begin()
{
	Wire.begin();
}

/**
*  Retreives and combines values from a given registers (MSB first).
*
* Input values :
*   - _dev_addr (uint8_t) : target device address on the I2C bus
*   - _reg_addr (uint8_t) : address of the first register to read (please refer to the device's register map)
*   - _data (uint8_t*) : raw binary data
*
* Return value : 
*   - Status (uint8_t) : 1 success, 0 error
**/  	
uint8_t I2C::read8(uint8_t _dev_addr, uint8_t _reg_addr, uint8_t *_data)
{
	*_data = 0;

  Wire.beginTransmission(_dev_addr);
  Wire.write(_reg_addr);
  if(Wire.endTransmission() > 0) return 0;

  Wire.beginTransmission(_dev_addr);
  Wire.requestFrom((uint8_t)_dev_addr, (uint8_t) 1);
  
  *_data = Wire.read();

  return 1;
}

/**
*  Retreives and combines values from n consecutive registers (MSB first).
*
* Input values :
*   - _dev_addr (uint8_t) : target device address on the I2C bus
*   - _reg_addr (uint8_t) : address of the first register to read (please refer to the device's register map)
*   - _len (uint8_t) : number of registers to read
*   - _data (uint32_t*) : raw binary data
*
* Return value : 
*   - Status (uint8_t) : 1 success, 0 error
**/  
uint8_t I2C::readBurstRegValue(uint8_t _dev_addr, uint8_t _starting_reg_addr, uint8_t _len, uint8_t* _buffer)
{
  Wire.beginTransmission(_dev_addr);
  Wire.write(_starting_reg_addr);
  if(Wire.endTransmission() > 0) return 0;
  
  Wire.requestFrom((uint8_t)_dev_addr, (uint8_t)_len);
  
  for (uint8_t i = 0; i < _len; i++)
    _buffer[i] = Wire.read();

  if(Wire.endTransmission() > 0) return 0;

  return 1;
}

/**
*  Retreives and combines values from two consecutive registers (MSB first).
*
* Input values :
*   - _dev_addr (uint8_t) : target device address on the I2C bus
*   - _reg_addr (uint8_t) : address of the first register to read (please refer to the device's register map)
*   - _data (uint16_t*) : raw binary data
*
* Return value : 
*   - Status (uint8_t) : 1 success, 0 error
**/  
uint8_t I2C::read16(uint8_t _dev_addr, uint8_t _reg_addr, uint16_t* _data)
{  
  Wire.beginTransmission(_dev_addr);
  Wire.write(_reg_addr);
  if(Wire.endTransmission() > 0) return 0;
  
  Wire.requestFrom((uint8_t)_dev_addr, (uint8_t)2);  
  *_data = (((uint16_t)Wire.read()) << 8) | ((uint16_t)Wire.read());
  
  return 1;
}

/**
*  Retreives and combines values from two consecutive registers (LSB first).
*
* Input values :
*   - _dev_addr (uint8_t) : target device address on the I2C bus
*   - _reg_addr (uint8_t) : address of the first register to read (please refer to the device's register map)
*   - _data (uint16_t*) : raw binary data
*
* Return value : 
*   - Status (uint8_t) : 1 success, 0 error
**/  
uint8_t I2C::read16_LE(uint8_t _dev_addr, uint8_t _reg_addr, uint16_t* _data)
{  
  uint16_t tmp;
  
  if(read16(_dev_addr, _reg_addr, &tmp) == 0) return 0;
  *_data = (tmp >> 8) | (tmp << 8);
  
  return 1;
}

/**
*	Overwrites a register with a given value.
*
*  Input values :
*   - _dev_addr (uint8_t) : target device address on the I2C bus
*   - _reg_addr (uint8_t) : target register address (refer to the device's register map)
*   - _reg_value (uint8_t) : new register value
*   
*  Return value : 
*   - Status (uint8_t) : 1 success, 0 error
**/	
uint8_t I2C::write8(uint8_t _dev_addr, uint8_t _reg_addr, uint8_t _reg_value)
{
	Wire.beginTransmission(_dev_addr);
	Wire.write(_reg_addr);
	Wire.write(_reg_value);
	if(Wire.endTransmission() > 0) return 0;
  
  else return 1;
}

/**
*  Retreives and combines values from n consecutive registers (MSB first).
*
* Input values :
*   - _dev_addr (uint8_t) : target device address on the I2C bus
*   - _reg_addr (uint8_t) : address of the first register to read (please refer to the device's register map)
*   - _len (uint8_t) : number of registers to read
*   - _data (uint32_t*) : raw binary data
*
* Return value : 
*   - Status (uint8_t) : 1 success, 0 error
**/  
uint8_t I2C::read32(uint8_t _dev_addr, uint8_t _starting_reg_addr, uint32_t* _data)
{
  *_data = 0x00;

  Wire.beginTransmission(_dev_addr);
  Wire.write(_starting_reg_addr);
  if(Wire.endTransmission() > 0) return 0;
  
  Wire.requestFrom((uint8_t)_dev_addr, (uint8_t)3);
  for (uint8_t i = 0; i < 3; i++)
  {
    uint8_t _dat = Wire.read();    
    *_data <<= 8;
    *_data |= (uint32_t)_dat;
  }

  return 1;
}
/**
*	Modifies a given number of bits of a register.
*
*	Input values :
*		- _dev_addr (uint32_t) : target device address on the I2C bus
*		- _reg_addr (uint32_t) : target register address (refer to the device's register map)
*		- _msb_start (uint32_t) : position of the MSB of the word to overwrite in the register
*		- _len (uint32_t) : number of bits to write
*		- _bits (uint32_t) : bit values
*   
*  Return value : 
*   - Status (uint8_t) : 1 success, 0 error
**/	
uint8_t I2C::writeBits(uint8_t _dev_addr, uint8_t _reg_addr, uint8_t _msb_start, uint8_t _len, uint8_t _bits)
{
	uint8_t original_data;
  
	if(read8(_dev_addr, _reg_addr, &original_data) == 0) return 0;
	uint8_t mask = ((1 << _len) - 1) << (_msb_start - _len + 1);
  
	_bits <<= (_msb_start - _len + 1);
	_bits &= mask;
	
	original_data &= ~(mask);
	original_data |= _bits;

  if(write8(_dev_addr, _reg_addr, original_data) == 0) return 0;
  return 1;
}
