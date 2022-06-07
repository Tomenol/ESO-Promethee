#ifndef TOOLS_H_
#define TOOLS_H_

#include "Arduino.h"

#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>

// tool sign
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

#define STR_LEN 234

// defines the properties of the variables in the data stream
#define SEPARATOR_LEN 1 // separator = ";"
#define START_LEN 1 // start = "$"
#define END_LEN 1 // end = "*"

// Time in ms
#define MS_TIME_LEN 8
#define MS_TIME_DEC_LEN 7 // Time is positive
#define MS_TIME_SIGN_LEN 0 
#define MS_TIME_PRECISION_LEN 0 

// acceleration
#define ACCEL_LEN 7 // dec + . + ; + ...
#define ACCEL_DEC_LEN 2 // 2 digits before "."
#define ACCEL_SIGN_LEN 1 // acceleration is a signed value
#define ACCEL_PRECISION_LEN 2 // .00

// gyroscopes
#define GYRO_LEN 9 
#define GYRO_DEC_LEN 4 // 4 digits before "."
#define GYRO_SIGN_LEN 1 // angular rate is a signed value
#define GYRO_PRECISION_LEN 2 // .00

// magnetometers
#define MAG_LEN 8 
#define MAG_DEC_LEN 3 // 3 digits before "."
#define MAG_SIGN_LEN 1 // angular rate is a signed value
#define MAG_PRECISION_LEN 2 // .00

// Pressure
#define PRESS_LEN 11
#define PRESS_DEC_LEN 6 // 6 digits before "."
#define PRESS_SIGN_LEN 1 // angular rate is a signed value
#define PRESS_PRECISION_LEN 2 // .00

// Temperature
#define TEMP_LEN 7 
#define TEMP_DEC_LEN 2 // 2 digits before "."
#define TEMP_SIGN_LEN 1 // temperature is a signed value
#define TEMP_PRECISION_LEN 2 // .00

// Angles in degs
#define ANG_LEN 8 
#define ANG_DEC_LEN 3 // 3 digits before "."
#define ANG_SIGN_LEN 1 // an angle is a signed value
#define ANG_PRECISION_LEN 4 // .0000

// Directions in N/S/E/W
#define DIR_LEN 2 
#define DIR_DEC_LEN 1 // directions are letters
#define DIR_SIGN_LEN 0 
#define DIR_PRECISION_LEN 0 

// Altitude
#define ALT_LEN 9 
#define ALT_DEC_LEN 4 
#define ALT_SIGN_LEN 1 // altitudes are real numbers
#define ALT_PRECISION_LEN 2 

// Velocity
#define VEL_LEN 8 
#define VEL_DEC_LEN 3 
#define VEL_SIGN_LEN 1 // velocities are real numbers
#define VEL_PRECISION_LEN 2 

// Satellite count
#define NSAT_LEN 3 
#define NSAT_DEC_LEN 2 
#define NSAT_SIGN_LEN 0 // Satellite count is positive
#define NSAT_PRECISION_LEN 0 

// Time h-m-s
#define TIME_LEN 3 
#define TIME_DEC_LEN 2 
#define TIME_SIGN_LEN 0 // Time is positive
#define TIME_PRECISION_LEN 0 

// Booleans
#define BOOL_LEN 2 
#define BOOL_DEC_LEN 1 
#define BOOL_SIGN_LEN 0 // Time is positive
#define BOOL_PRECISION_LEN 0 

// start positions
#define START_START 	0 
#define START_TIME_MS 	SEPARATOR_LEN 

#define START_AX1 		START_TIME_MS + MS_TIME_LEN
#define START_AY1 		START_AX1 + ACCEL_LEN
#define START_AZ1 		START_AY1 + ACCEL_LEN

#define START_AX2 		START_AZ1 + ACCEL_LEN
#define START_AY2 		START_AX2 + ACCEL_LEN
#define START_AZ2 		START_AY2 + ACCEL_LEN

#define START_WX1 		START_AZ2 + ACCEL_LEN
#define START_WY1 		START_WX1 + GYRO_LEN
#define START_WZ1 		START_WY1 + GYRO_LEN

#define START_WX2 		START_WZ1 + GYRO_LEN
#define START_WY2 		START_WX2 + GYRO_LEN
#define START_WZ2 		START_WY2 + GYRO_LEN

#define START_MX1 		START_WZ2 + GYRO_LEN
#define START_MY1 		START_MX1 + MAG_LEN
#define START_MZ1 		START_MY1 + MAG_LEN
	
#define START_MX2 		START_MZ1 + MAG_LEN
#define START_MY2 		START_MX2 + MAG_LEN
#define START_MZ2 		START_MY2 + MAG_LEN

#define START_PRESS 	START_MZ2 + MAG_LEN
#define START_TEMP 		START_PRESS + PRESS_LEN
#define START_ALTP 		START_TEMP + TEMP_LEN

#define START_LON 		START_ALTP + ALT_LEN
#define START_LOND 		START_LON + ANG_LEN
#define START_LAT 		START_LOND + DIR_LEN
#define START_LATD 		START_LAT + ANG_LEN
#define START_ALT 		START_LATD + DIR_LEN
#define START_VEL 		START_ALT + ALT_LEN
#define START_VELD 		START_VEL + VEL_LEN
#define START_H 		START_VELD + ANG_LEN
#define START_M 		START_H + TIME_LEN
#define START_S 		START_M + TIME_LEN
#define START_NSAT 		START_S + TIME_LEN
#define START_NEW_MEAS 	START_NSAT + NSAT_LEN
#define START_FIX 		START_NEW_MEAS + BOOL_LEN

#define START_END 		START_FIX + BOOL_LEN

uint8_t add_double_variable_to_output_string(double _value, int _sign_len, int _dec_len, int _precision_len);
uint8_t add_char_variable_to_output_string(char _value);

uint8_t generate_message(int _start_time_ms, double *_accel1, double *_accel2, double *_gyro1, double *_gyro2, double *_mag1, double *_mag2, double _pressure, double _temperature, double _alt_pressure, Adafruit_GPS *_gps_instance, SoftwareSerial *_gps_serial_bus, uint8_t *_new_gps_data);
uint8_t gps_measurements(Adafruit_GPS* _gnss_receiver, SoftwareSerial *_gps_serial_bus, uint8_t *_new_gps_data);
#endif TOOLS_H_