/*	MPU6050 MEMS IMU Library :
*		This library is an optimized version of the arduino MPU6050 
*		library : https://github.com/ElectronicCats/mpu6050
*			
*	This version allows one to easily configure and retrieve accelerometer
*	and gyroscope measurements.
*
*	Dependencies :
*		- I2C library : used to send / receive data to and from the MPU6050
*		
*	Project : ESO Prométhée
*	Author : MAYNADIE Thomas
*
*	Version : 16-10-2021
*/

#include "Arduino.h"

#include <SoftwareSerial.h>
#include <MicroNMEA.h>

// config values


class GNSS_Receiver
{
  private:
    SoftwareSerial *gps_serial_bus;
    MicroNMEA *nmea;
  
	public:
		GNSS_Receiver(MicroNMEA* _nmea, SoftwareSerial* _gps_serial_bus);

		uint8_t begin(uint8_t _wait_for_fix);

    uint8_t is_valid();
		uint8_t read_gps();

    uint8_t get_position(double *_longitude, double *_lattitude, double *_altitude);
    uint8_t get_time(int *_h, int *_m, int *_s);
		uint8_t get_date(int *_y, int *_d, int *_m);
		uint8_t get_quality(char **_nav_system, char *_n_sat, double *_hdop);
	};
