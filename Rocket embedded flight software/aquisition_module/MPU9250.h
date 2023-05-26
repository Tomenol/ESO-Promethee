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

#ifndef MPU_9250_H_
#define MPU_9250_H_

// config values
#define MPU9250_RA_SMPLRT_DIV      			0x19
#define MPU9250_RV_SMPLRT_DIV      			0x00

#define MPU9250_RA_CONFIG     				  0x1A
#define MPU9250_RV_CONFIG      				  0x00

#define MPU9250_RA_GYRO_CONFIG      		0x1B
#define MPU9250_RV_GYRO_CONFIG       		0x18 // set gyro +/-2000dps

#define MPU9250_RA_ACCEL_CONFIG      		0x1C
#define MPU9250_RV_ACCEL_CONFIG       	0x18// set accel +/- 16g

#define MPU9250_RA_ACCEL_CONFIG_2      	0x1D
#define MPU9250_RV_ACCEL_CONFIG_2       0x00

#define MPU9250_RA_ACCEL_FIFO_EN      	0x23
#define MPU9250_RV_ACCEL_FIFO_EN	      0x00

#define MPU9250_RA_ACCEL_USER_CTRL      0x6A
#define MPU9250_RV_ACCEL_USER_CTRL      0x01

#define MPU9250_RA_ACCEL_PWR_MGMT      	0x6B
#define MPU9250_RV_ACCEL_PWR_MGMT       0x00

#define MPU9250_RA_ACCEL_PWR_MGMT_2     0x6C
#define MPU9250_RV_ACCEL_PWR_MGMT_2     0x00

#define MPU9250_RA_INT_PIN_CFG          0x37
#define MPU9250_RV_INT_PIN_CFG          0x02

// measurements
#define MPU9250_RA_ACCEL_XOUT_H     		0x3B
#define MPU9250_RA_GYRO_XOUT_H      		0x43

#define LSB_ACCEL_MEASUREMENT_2G   			1/16384.0
#define LSB_ACCEL_MEASUREMENT_4G 			  1/8192.0
#define LSB_ACCEL_MEASUREMENT_8G 			  1/4096.0
#define LSB_ACCEL_MEASUREMENT_16G 		  1/2048.0

#define LSB_GYRO_MEASUREMENT_250 			  1/131.0
#define LSB_GYRO_MEASUREMENT_500 			  1/65.5
#define LSB_GYRO_MEASUREMENT_1000 			1/32.8
#define LSB_GYRO_MEASUREMENT_2000 			1/16.4

#define LSB_MAG_MEASUREMENT             4912.0/32760.0

// magnetometer
#define AK8963_SLAVE_ADDRESS 0x0C ///< AK8963 I2C slave address

#define AK8963_WIA 0x00
#define AK8963_INFO 0x01
#define AK8963_ST1 0x02
#define AK8963_HXL 0x03
#define AK8963_HXH 0x04
#define AK8963_HYL 0x05
#define AK8963_HYH 0x06
#define AK8963_HZL 0x07
#define AK8963_HZH 0x08
#define AK8963_ST2 0x09
#define AK8963_CNTL1 0x0A
#define AK8963_CNTL2 0x0B
#define AK8963_ASTC 0x0C
#define AK8963_TS1 0x0D
#define AK8963_TS2 0x0E
#define AK8963_I2CDIS 0x0F
#define AK8963_ASAX 0x10
#define AK8963_ASAY 0x11
#define AK8963_ASAZ 0x12
#define AK8963_RSV 0x13


class MPU9250
{
	private:
		uint8_t dev_address;
	    double magCoeff[3];
	    
	    uint8_t get_magnetometer_coefficients();
    
	public:
		MPU9250(uint8_t _dev_address);
		
		uint8_t initialize();
	    uint8_t reset_device();

	    uint8_t getRawAccelerationVector(double* _accel);
	    uint8_t getRawAngularVelocityVector(double* _angular_vel);
	    uint8_t getRawMagVector(double* _mag);
};

#endif 
