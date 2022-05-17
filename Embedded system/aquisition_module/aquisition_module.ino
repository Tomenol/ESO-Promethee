// *******************************
//        Arduino libraries
// *******************************

#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

// *******************************
//          Custom libraries
// *******************************

#include "MPU9250.h"
#include "BMP280.h"
#include "GPS.h"

#define PERIODICITY 100 // ms

#define MASTER_PIN_LED_FIX 5
#define MASTER_POWER_INIT 6
#define MASTER_PIN_LED_POWER 7  

#define GPS_TX_PIN 10
#define GPS_RX_PIN 9

// *******************************
//    Pressure sensor : BMP280
// *******************************

// pressure sensor object
BMP280 bmp_280;

// measurements data
double pressure, temperature;
double altitude_pressure;

// *******************************
//         IMUs : MPU9250
// *******************************

uint32_t t_start;

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
SoftwareSerial gps_serial_bus(GPS_TX_PIN, GPS_RX_PIN);
Adafruit_GPS gnss_receiver(&gps_serial_bus);

uint8_t new_gps_data;


// *******************************
//               SETUP
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

  t_start = 0;
}

// *******************************
//   MAIN LOOP : data acquisition
// *******************************
void loop() 
{  
  t_start = millis();

  // measurements
  while(millis() - t_start < PERIODICITY)
  {
    gps_measurements(); // reads the measurements from the gps
  }
  
  high_rate_measurements(); // gather pressure and IMU measurements

  // send data to slave for storage
  send_data_to_slave(); 
  
  new_gps_data = 0;
}

/*
 * Function gps_measurements() :
 *    Gathers the measurements from the gps (low data rates).
 */
uint8_t gps_measurements()
{
  if(gnss_receiver.available() > 0)
  {
    if (new_gps_data == 0)
    {
      char c = gnss_receiver.read();
      //Serial.print(c);
      
      if(gnss_receiver.newNMEAreceived() && new_gps_data == 0)
      {
        if (gnss_receiver.parse(gnss_receiver.lastNMEA()))
        {
          Serial.println(gnss_receiver.lastNMEA());
          new_gps_data = 1;
        }
      }
    }
    else
    {
      char c = gps_serial_bus.read(); // clearing the buffer without processing the data
      //Serial.print(c);
    }
  }

  return 1;
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

  mpu9250_2.getRawAccelerationVector(accel2);
  mpu9250_2.getRawAngularVelocityVector(gyro2);
  mpu9250_2.getRawMagVector(mag_meas2);

  return 1;
}

/*
 * Function send_data_to_slave() :
 *    Sends the data as dynamic string of characters. Each data point is separed by the separator ';'.
 */
uint8_t send_data_to_slave()
{
  Serial.print("$");
  Serial.print(t_start);
  Serial.print(";");
  Serial.print(accel1[0]);
  Serial.print(";");
  Serial.print(accel1[1]);
  Serial.print(";");
  Serial.print(accel1[2]);
  Serial.print(";");
  Serial.print(accel2[0]);
  Serial.print(";");
  Serial.print(accel2[1]);
  Serial.print(";");
  Serial.print(accel2[2]);
  Serial.print(";");
  Serial.print(gyro1[0]);
  Serial.print(";");
  Serial.print(gyro1[1]);
  Serial.print(";");
  Serial.print(gyro1[2]);
  Serial.print(";");
  Serial.print(gyro2[0]);
  Serial.print(";");
  Serial.print(gyro2[1]);
  Serial.print(";");
  Serial.print(gyro2[2]);
  Serial.print(";");
  Serial.print(mag_meas1[0]);
  Serial.print(";");
  Serial.print(mag_meas1[1]);
  Serial.print(";");
  Serial.print(mag_meas1[2]);
  Serial.print(";");
  Serial.print(mag_meas2[0]);
  Serial.print(";");
  Serial.print(mag_meas2[1]);
  Serial.print(";");
  Serial.print(mag_meas2[2]);
  Serial.print(";");

  Serial.print(pressure);
  Serial.print(";");
  Serial.print(temperature);
  Serial.print(";");
  Serial.print(altitude_pressure);
  Serial.print(";");

  //gps
  if (gnss_receiver.fix)
  {
    Serial.print(gnss_receiver.longitude);
    Serial.print(";");
    Serial.print(gnss_receiver.lon);
    Serial.print(";");
    Serial.print(gnss_receiver.latitude);
    Serial.print(";");
    Serial.print(gnss_receiver.lat);
    Serial.print(";");
    Serial.print(gnss_receiver.altitude);
    Serial.print(";");
    Serial.print(gnss_receiver.speed);
    Serial.print(";");
    Serial.print(gnss_receiver.angle);
    Serial.print(";"); 

    Serial.print((gnss_receiver.hour+2)%24, DEC);
    Serial.print(";");
    Serial.print(gnss_receiver.minute, DEC);
    Serial.print(";");
    Serial.print(gnss_receiver.seconds, DEC);
    Serial.print(";");
  
    Serial.print((int)gnss_receiver.fixquality);
    Serial.print(";");
    Serial.print((int)gnss_receiver.satellites);
    Serial.print(";");
    Serial.print(new_gps_data, DEC);
    Serial.print(";");
    Serial.print((int)gnss_receiver.fix, DEC);
    Serial.println("*");
    
    digitalWrite(MASTER_PIN_LED_FIX,1);
  }
  else
  {
    Serial.println("0.00;E;0.00;N;0.00;0.00;0.00;0.00;0.00;0;0;0;0.00;0;0;0*");
    digitalWrite(MASTER_PIN_LED_FIX, 0);
  }

  return 1;
}
