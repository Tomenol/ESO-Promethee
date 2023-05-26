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

#include "GPS.h"

GNSS_Receiver::GNSS_Receiver(MicroNMEA* _nmea, SoftwareSerial* _gps_serial_bus)
{
  this->nmea = _nmea;
  this->gps_serial_bus = _gps_serial_bus;
}

uint8_t GNSS_Receiver::is_valid()
{
  return (uint8_t)this->nmea->isValid();
}

uint8_t GNSS_Receiver::begin(uint8_t _wait_for_fix)
{

  if(_wait_for_fix == 1)
  {
  	uint8_t iflag = 0;

  	while(iflag == 0)
  	{
  		this->read_gps();
  		
  		if(this->nmea->isValid()) iflag = 1;
  	}
  }

  return 1;
}

uint8_t GNSS_Receiver::read_gps()
{
	char dat;

	if (this->gps_serial_bus->available() == 0) return 0;
	else
	{
		while (this->gps_serial_bus->available())
	   {
	      dat = this->gps_serial_bus->read();
	      this->nmea->process(dat); // process NMEA data 
	   }
	}

	return 1;
}

uint8_t GNSS_Receiver::get_position(double *_longitude, double *_latitude, double *_altitude)
{
	if(!this->nmea->isValid()) return 0;

  long alt;
  this->nmea->getAltitude(alt);

	*_longitude = this->nmea->getLongitude() / 1000000.;
	*_latitude = this->nmea->getLatitude() / 1000000.;
	*_altitude = (double)alt / 1000.;

	return 1;
}

uint8_t GNSS_Receiver::get_time(int *_h, int *_m, int *_s)
{
	if(!this->nmea->isValid()) return 0;

    *_h = (int)this->nmea->getHour();
    *_m = (int)this->nmea->getMinute();
    *_s = (int)this->nmea->getSecond();

	return 1;
}

uint8_t GNSS_Receiver::get_date(int *_y, int *_m, int *_d)
{
	if(!this->nmea->isValid()) return 0;

	*_y = (int)this->nmea->getYear();
	*_m = (int)this->nmea->getMonth();
	*_d = (int)this->nmea->getDay();

	return 1;
}

uint8_t GNSS_Receiver::get_quality(char **_nav_system, char *_n_sat, double *_hdop)
{
	if(!this->nmea->isValid()) return 0;

	*_nav_system = this->nmea->getNavSystem();
	*_n_sat = this->nmea->getNumSatellites();
	*_hdop = this->nmea->getHDOP()/10.;

	return 1;
}
