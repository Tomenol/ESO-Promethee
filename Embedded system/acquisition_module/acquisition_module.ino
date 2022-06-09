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
#define PERIODICITY 200 // ms

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
BMP280 bmp_280;

// measurements data
double pressure, temperature;
double altitude_pressure;


// *******************************
//        IMUs : 2x MPU9250
// *******************************
MPU9250 mpu9250_1(0x68);
MPU9250 mpu9250_2(0x69);

// measurements data
double accel1[3], gyro1[3], mag_meas1[3];
double accel2[3], gyro2[3], mag_meas2[3];


// *******************************
//   GNSS : Adafruit ultimate GPS 
// *******************************
SoftwareSerial gps_serial_bus(GPS_TX_PIN, GPS_RX_PIN); // rx, tx as seen from the arduino
Adafruit_GPS gnss_receiver(&gps_serial_bus);
uint8_t new_gps_data;


// *******************************
//             SETUP
// *******************************
void setup() 
{
  // setup LED pins
  pinMode(MASTER_PIN_LED_POWER,OUTPUT); 
  pinMode(MASTER_PIN_LED_FIX,OUTPUT);
  pinMode(MASTER_POWER_INIT,OUTPUT);
  
  // set power LED on as the board is initializing
  digitalWrite(MASTER_PIN_LED_POWER,1);
  
  // Setup communication interfaces
  I2C::begin(); // I2C for sensor communication (BMP/MPU)
  Serial.begin(115200); // Serial for Master/Slave communication

  // init BMP280 chip
  bmp_280.initialize();
  delay(250);
  bmp_280.setReferenceAltitude(0);

  // init MPU 9250 chips
  mpu9250_1.initialize();
  delay(250);
  mpu9250_2.initialize();
  delay(250);
  
  // init GPS
  gnss_receiver.begin(9600);
  gnss_receiver.sendCommand("$PMTK251,57600*2C");  //set baud rate to 57600
  gps_serial_bus.end();
  gnss_receiver.begin(57600);
  delay(1000);
  
  gnss_receiver.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);   // 1 Hz update rate
  gnss_receiver.sendCommand(PGCMD_ANTENNA);

  // Turn on initialization done LED
  digitalWrite(MASTER_POWER_INIT, 1);  
  t_start = millis();
}


// *******************************
//   MAIN LOOP : data acquisition
// *******************************
void loop() 
{  
  // gather GPS measurements
  while(millis() - t_start < PERIODICITY)
  {
    gps_measurements(&gnss_receiver, &gps_serial_bus, &new_gps_data); // reads the measurements from the gps
  }

  t_start = millis();

  // gather pressure and IMU measurements
  high_rate_measurements(); 

  // send data to slave for storage
  generate_message(t_start, accel1, accel2, gyro1, gyro2, mag_meas1, mag_meas2, pressure, temperature, altitude_pressure, &gnss_receiver, &gps_serial_bus, &new_gps_data);

  new_gps_data = 0;
}

uint8_t high_rate_measurements()
/*
 * Function high_rate_measurements() :
 *    Gathers the measurements from the IMUs and the pressure sensor (high data rates).
 */
{
  // read BMP280
  if(bmp_280.readValues(&temperature, &pressure) == 0);
  else altitude_pressure = bmp_280.getAltitude(pressure, temperature);
  
  // read 1st MPU9250
  mpu9250_1.getRawAccelerationVector(accel1);
  mpu9250_1.getRawAngularVelocityVector(gyro1);
  mpu9250_1.getRawMagVector(mag_meas1);
  gps_measurements(&gnss_receiver, &gps_serial_bus, &new_gps_data);

  // read 2nd MPU9250
  mpu9250_2.getRawAccelerationVector(accel2);
  mpu9250_2.getRawAngularVelocityVector(gyro2);
  mpu9250_2.getRawMagVector(mag_meas2);
  gps_measurements(&gnss_receiver, &gps_serial_bus, &new_gps_data);

  return 1;
}
