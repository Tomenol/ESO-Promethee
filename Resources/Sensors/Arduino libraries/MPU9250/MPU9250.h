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

#include "I2C.h"
#include "Arduino.h"

#ifndef MPU_6050_H_
#define MPU_6050_H_

#define LSB_ACCEL_MEASUREMENT 				2048.0
#define LSB_GYRO_MEASUREMENT 				65.5

#define MPU6050_RA_ACCEL_XOUT_H     		0x3B

#define MPU6050_RA_GYRO_XOUT_H      		0x43

#define MPU6050_RA_PWR_MGMT_1       		0x6B
#define MPU6050_RA_GYRO_CONFIG      		0x1B
#define MPU6050_RA_ACCEL_CONFIG     		0x1C

#define MPU6050_CLOCK_PLL_XGYRO         	0x01
#define MPU6050_PWR1_CLKSEL_BIT     		2
#define MPU6050_PWR1_CLKSEL_LENGTH      	3

#define MPU6050_GCONFIG_FS_SEL_BIT      	4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   	2

#define MPU6050_ACONFIG_AFS_SEL_BIT       	4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH    	2

#define MPU6050_PWR1_SLEEP_BIT          	6

class MPU6050
{
	private:
		uint8_t dev_address;
		
		float raw_accel[3];
		float raw_angular_vel[3];
	
	public:
		MPU6050(uint8_t _dev_address);
		
		void initialize();
		
		float* getRawAccelerationVector();
		float* getRawRotationalVelocityVector();
		float* getRawAngularVelocityVector();
};

#endif 
