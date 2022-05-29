// *******************************
//        Arduino libraries
// *******************************

#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

// *******************************
//        Custom libraries
// *******************************

#include "MPU9250.h"
#include "BMP280.h"

#include "tools.h"

// *******************************
//             Defines
// *******************************

#define PERIODICITY 100 // ms

#define MASTER_PIN_LED_FIX 5
#define MASTER_POWER_INIT 6
#define MASTER_PIN_LED_POWER 7  

#define GPS_TX_PIN 10
#define GPS_RX_PIN 9

// *******************************
//        General variables
// *******************************

uint32_t t_start;

// *******************************
//    Pressure sensor : BMP280
// *******************************

// pressure sensor object
BMP280 bmp_280;

// measurements data
double pressure, temperature;
double altitude_pressure;

// *******************************
//        IMUs : 2x MPU9250
// *******************************

// IMU sensor objects
MPU9250 mpu9250_1(0x68);
MPU9250 mpu9250_2(0x69);

// measurements data
double accel1[3], gyro1[3], mag_meas1[3];
double accel2[3], gyro2[3], mag_meas2[3];

// *******************************
//   GNSS : Adafruit ultimate GPS 
// *******************************

// GPS library 
SoftwareSerial gps_serial_bus(GPS_TX_PIN, GPS_RX_PIN); // rx, tx as seen from the arduino
Adafruit_GPS gnss_receiver(&gps_serial_bus);

uint8_t new_gps_data;


// *******************************
//             SETUP
// *******************************

void setup() 
{
  pinMode(MASTER_PIN_LED_POWER,OUTPUT);
  pinMode(MASTER_PIN_LED_FIX,OUTPUT);
  pinMode(MASTER_POWER_INIT,OUTPUT);
  
  digitalWrite(MASTER_PIN_LED_POWER,1);
  
  I2C::begin();
  Serial.begin(115200); // utilisé seulement pour les tests, il faudra l'enlever

  // init BMP280 chip
  bmp_280.initialize();
  
  // init GPS
  gnss_receiver.begin(9600);
  gnss_receiver.sendCommand("$PMTK251,57600*2C");  //set baud rate to 57600
  gps_serial_bus.end();
  gnss_receiver.begin(57600);

  delay(1000);
  
  gnss_receiver.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  gnss_receiver.sendCommand(PGCMD_ANTENNA);
    
  // init MPU 9250 chips
  mpu9250_1.initialize();
  mpu9250_2.initialize();

  bmp_280.setReferenceAltitude(0);
  digitalWrite(MASTER_POWER_INIT, 1);

  delay(1000);
}

// *******************************
//   MAIN LOOP : data acquisition
// *******************************
void loop() 
{  
  t_start = millis();

  high_rate_measurements(); // gather pressure and IMU measurements

  // gather measurements
  while(millis() - t_start < PERIODICITY)
  {
    gps_measurements(&gnss_receiver, &gps_serial_bus, &new_gps_data); // reads the measurements from the gps
  }

  generate_message(t_start, accel1, accel2, gyro1, gyro2, mag_meas1, mag_meas2, pressure, temperature, altitude_pressure, &gnss_receiver, &gps_serial_bus, &new_gps_data);

  new_gps_data = 0;
}

/*
 * Function high_rate_measurements() :
 *    Gathers the measurements from the IMUs and the pressure sensor (high data rates).
 */
uint8_t high_rate_measurements()
{
  if(bmp_280.readValues(&temperature, &pressure) == 0);
  else altitude_pressure = bmp_280.getAltitude(pressure, temperature);
  
  mpu9250_1.getRawAccelerationVector(accel1);
  mpu9250_1.getRawAngularVelocityVector(gyro1);
  mpu9250_1.getRawMagVector(mag_meas1);
  gps_measurements(&gnss_receiver, &gps_serial_bus, &new_gps_data);

  mpu9250_2.getRawAccelerationVector(accel2);
  mpu9250_2.getRawAngularVelocityVector(gyro2);
  mpu9250_2.getRawMagVector(mag_meas2);
  gps_measurements(&gnss_receiver, &gps_serial_bus, &new_gps_data);

  return 1;
}
