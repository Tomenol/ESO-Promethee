#include <Wire.h>

#include "I2C.h"
#include "MPU6050.h"

#define MPU6050_ADDR_1 0x68
#define MPU6050_ADDR_2 0x69

MPU6050 accelerometer1(MPU6050_ADDR_1);
MPU6050 accelerometer2(MPU6050_ADDR_2);

unsigned long end_time = 0, start_time = 0;

void setup() 
{
  Serial.begin(115200);
  Wire.begin();
  I2C::begin();

  accelerometer1.initialize();
  accelerometer2.initialize();
}

void loop() 
{
    end_time = start_time;
    start_time = micros();

    float *accel1_accel_vec = accelerometer1.getRawAccelerationVector();
    float *accel1_angvel_vec = accelerometer1.getRawAngularVelocityVector();
    float *accel2_accel_vec = accelerometer2.getRawAccelerationVector();
    float *accel2_angvel_vec = accelerometer2.getRawAngularVelocityVector();

    // print data format -> t ax1 ax2 ay1 ay2 az1 az2 wx1 wx2 wy1 wy2 wz1 wz2
    Serial.println(0xFFED, DEC); // start byte
    Serial.println(start_time, DEC);

    for(uint8_t i = 0; i < 3; i++)
    {
        Serial.println(accel1_accel_vec[i], DEC);
        Serial.println(accel2_accel_vec[i], DEC);
    }

    for(uint8_t i = 0; i < 3; i++)
    {
        Serial.println(accel1_angvel_vec[i], DEC);
        Serial.println(accel2_angvel_vec[i], DEC);
    }

    Serial.println(0xFFEC, DEC); // stop byte
}
