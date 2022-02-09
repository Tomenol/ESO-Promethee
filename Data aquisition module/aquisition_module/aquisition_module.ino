// *********************************
//        Arduino libraries
// *********************************

#include <SoftwareSerial.h>
#include <MicroNMEA.h>

// *********************************
//          Custom libraries
// *********************************

#include "libraries/MPU9250.h"
#include "libraries/BMP280.h"
#include "libraries/GPS.h"

#define GPS_TX_PIN 8
#define GPS_RX_PIN 7

// *********************************
//    Pressure sensor : BMP280
// *********************************

// pressure sensor object
BMP280 bmp_280;

// measurements data
double pressure, temperature;
double altitude_pressure;

// *********************************
//         IMUs : MPU9250
// *********************************

// IMU sensor objects
MPU9250 mpu9250_1(0x69);
MPU9250 mpu9250_2(0x68);

// measurements data
double accel1[3], gyro1[3], mag_meas1[3];
double accel2[3], gyro2[3], mag_meas2[3];


// *********************************
//   GNSS : Adafruit ultimate GPS 
// *********************************

// gnss navigation data
double gps_longitude, gps_latitude, gps_altitude;
int time_hour, time_minute, time_second;
int time_day, time_month, time_year;

// gnss status / quality data
char *nav_system;
char n_sat;
double hdop;

// NMEA library 
char nmeaBuffer[100];
SoftwareSerial gps_serial_bus(GPS_TX_PIN, GPS_RX_PIN);
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

// GNSS receiver object
GNSS_Receiver gnss_receiver(&nmea, &gps_serial_bus);


// *********************************
//               SETUP
// *********************************

void setup() 
{    
  I2C::begin();
  Serial.begin(115200); // utilisé seulement pour les tests, il faudra l'enlever

  // init BMP280 chip
  bmp_280.initialize();

  // init GPS
  gnss_receiver.begin(0);
  // TODO :
  // attendre fix du GPS pour démarrer
  // set start time to GPS time
  // set start altitude to GPS altitude
  
  // init MPU 9250 chips
  mpu9250_1.initialize();
  mpu9250_2.initialize();

  bmp_280.setReferenceAltitude(0);
  
  // ajouter voyant pour montrer statut initialisation (ex : pas de voyant -> en cours, vert -> initialisation terminée)
}

// *********************************
//   MAIN LOOP : data acquisition
// *********************************

void loop() 
{
  if(gnss_receiver.read_gps())
  {
    if(gnss_receiver.is_valid())
    {
      gnss_receiver.get_position(&gps_longitude, &gps_latitude, &gps_altitude);
      gnss_receiver.get_time(&time_hour, &time_minute, &time_second);
      gnss_receiver.get_date(&time_year, &time_month, &time_day);
      gnss_receiver.get_quality(&nav_system, &n_sat, &hdop);

      Serial.println("GPS DATA : ");
      
      Serial.println(gps_longitude);
      Serial.println(gps_latitude);
      Serial.println(gps_altitude);
    
      Serial.println(time_hour);
      Serial.println(time_minute);
      Serial.println(time_second);
    
      Serial.println(time_year);
      Serial.println(time_month);
      Serial.println(time_day);
    
      Serial.println(nav_system);
      Serial.println(n_sat);
      Serial.println(hdop);
    }
  }
  
  if(bmp_280.readValues(&temperature, &pressure) == 0) Serial.println("Error reading BMP180");
  else altitude_pressure = bmp_280.getAltitude(pressure, temperature);
  
  mpu9250_1.getRawAccelerationVector(accel1);
  mpu9250_1.getRawAngularVelocityVector(gyro1);
  mpu9250_1.getRawMagVector(mag_meas1);

  mpu9250_2.getRawAccelerationVector(accel2);
  mpu9250_2.getRawAngularVelocityVector(gyro2);
  mpu9250_2.getRawMagVector(mag_meas2);
  
  Serial.println("\nOTHER DATA : ");

  Serial.println(accel1[0]);
  Serial.println(accel1[1]);
  Serial.println(accel1[2]);

  Serial.println(accel2[0]);
  Serial.println(accel2[1]);
  Serial.println(accel2[2]);

  Serial.println(gyro1[0]);
  Serial.println(gyro1[1]);
  Serial.println(gyro1[2]);

  Serial.println(gyro2[0]);
  Serial.println(gyro2[1]);
  Serial.println(gyro2[2]);

  Serial.println(mag_meas1[0]);
  Serial.println(mag_meas1[1]);
  Serial.println(mag_meas1[2]);

  Serial.println(mag_meas2[0]);
  Serial.println(mag_meas2[1]);
  Serial.println(mag_meas2[2]);

  Serial.println(pressure);
  Serial.println(temperature);
  Serial.println(altitude_pressure);
}
