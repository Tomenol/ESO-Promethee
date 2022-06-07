#include "tools.h"

uint8_t add_double_variable_to_output_string(
	double _value, 
	int _sign_len, 
	int _dec_len, 
	int _precision_len
	)
{
	int _var_len = _sign_len + _dec_len;

	if (_precision_len > 0)
	{
		_var_len += _precision_len + 1;
	}

	double max_pow_10 =  pow(10.0, _dec_len);
	
	if(_value > max_pow_10)
	{
		int ovf_vals = (int)(_value/max_pow_10);
		_value = _value - (double)ovf_vals*max_pow_10;
	}

	for(int ix = 0; ix < _var_len; ix++)
	{
		if(_sign_len == 1 && ix == 0) // write the sign
		{
			if (sgn(_value) == 1 || sgn(_value) == 0) // check if value is positive or 0
			{
				Serial.print("+");
			}
			else
			{
				Serial.print("-");
			}

			_value = _value/sgn(_value); // make the value positive
		}
		else // write each digit in order
		{
			if(ix - _sign_len != _dec_len)
			{
				double pow_10 = 1.0;
	
				if (ix - _sign_len < _dec_len)
				{
					pow_10 = pow(10.0, (double)(_dec_len - (ix - _sign_len)) - 1);
				}
				else
				{
					pow_10 = pow(10.0, (double)(_dec_len - (ix - _sign_len)));
				}
				
				char dec = (char)(_value/pow_10);

				Serial.print('0' + dec);
				_value = _value - (double)dec * pow_10;
			}
			else
			{
				Serial.print(".");
			}
		}
	}

	// separator
	Serial.print(';');
  
  
	return 1;
}

uint8_t add_char_variable_to_output_string(char _value)
{
	Serial.print(_value);
	Serial.print(';');
  
	return 1;
}

uint8_t generate_message(
	int _start_time_ms,
	double *_accel1,
	double *_accel2,
	double *_gyro1, 
	double *_gyro2, 
	double *_mag1, 
	double *_mag2, 
	double _pressure, 
	double _temperature, 
	double _alt_pressure, 
	Adafruit_GPS *_gps_instance,
	SoftwareSerial *_gps_serial_bus,
	uint8_t *_new_gps_data
	)
{
	// data sentence :                 t_start  ax1    ay1    az1    ax2    ay2    az2    wx1      wy1      wz1      wx2      wy2      wz2      mx1     my1     mz1     mx2     my2     mz2     P          T      zP       lon     d lat     d speed    dir     h  m  s  n  - f
	// char data_sentence[STR_LEN] = "$0000000;+00.00;+00.00;+00.00;+00.00;+00.00;+00.00;+0000.00;+0000.00;+0000.00;+0000.00;+0000.00;+0000.00;+000.00;+000.00;+000.00;+000.00;+000.00;+000.00;+000000.00;+00.00;+0000.00;;*";
	Serial.println("$")
	add_double_variable_to_output_string(_start_time_ms, MS_TIME_SIGN_LEN, MS_TIME_DEC_LEN, MS_TIME_PRECISION_LEN);
	gps_measurements(_gps_instance, _gps_serial_bus, _new_gps_data);

	// acceleration
	add_double_variable_to_output_string(_accel1[0], ACCEL_SIGN_LEN, ACCEL_DEC_LEN, ACCEL_PRECISION_LEN);
	add_double_variable_to_output_string(_accel1[1], ACCEL_SIGN_LEN, ACCEL_DEC_LEN, ACCEL_PRECISION_LEN);
	add_double_variable_to_output_string(_accel1[2], ACCEL_SIGN_LEN, ACCEL_DEC_LEN, ACCEL_PRECISION_LEN);
	gps_measurements(_gps_instance, _gps_serial_bus, _new_gps_data);

	add_double_variable_to_output_string(_accel2[0], ACCEL_SIGN_LEN, ACCEL_DEC_LEN, ACCEL_PRECISION_LEN);
	add_double_variable_to_output_string(_accel2[1], ACCEL_SIGN_LEN, ACCEL_DEC_LEN, ACCEL_PRECISION_LEN);
	add_double_variable_to_output_string(_accel2[2], ACCEL_SIGN_LEN, ACCEL_DEC_LEN, ACCEL_PRECISION_LEN);
	gps_measurements(_gps_instance, _gps_serial_bus, _new_gps_data);

	// gyroscope
	add_double_variable_to_output_string(_gyro1[0], GYRO_SIGN_LEN, GYRO_DEC_LEN, GYRO_PRECISION_LEN);
	add_double_variable_to_output_string(_gyro1[1], GYRO_SIGN_LEN, GYRO_DEC_LEN, GYRO_PRECISION_LEN);
	add_double_variable_to_output_string(_gyro1[2], GYRO_SIGN_LEN, GYRO_DEC_LEN, GYRO_PRECISION_LEN);
	gps_measurements(_gps_instance, _gps_serial_bus, _new_gps_data);	

	add_double_variable_to_output_string(_gyro2[0], GYRO_SIGN_LEN, GYRO_DEC_LEN, GYRO_PRECISION_LEN);
	add_double_variable_to_output_string(_gyro2[1], GYRO_SIGN_LEN, GYRO_DEC_LEN, GYRO_PRECISION_LEN);
	add_double_variable_to_output_string(_gyro2[2], GYRO_SIGN_LEN, GYRO_DEC_LEN, GYRO_PRECISION_LEN);
	gps_measurements(_gps_instance, _gps_serial_bus, _new_gps_data);

	// magnetometer
	add_double_variable_to_output_string(_mag1[0], MAG_SIGN_LEN, MAG_DEC_LEN, MAG_PRECISION_LEN);
	add_double_variable_to_output_string(_mag1[1], MAG_SIGN_LEN, MAG_DEC_LEN, MAG_PRECISION_LEN);
	add_double_variable_to_output_string(_mag1[2], MAG_SIGN_LEN, MAG_DEC_LEN, MAG_PRECISION_LEN);
	gps_measurements(_gps_instance, _gps_serial_bus, _new_gps_data);

	add_double_variable_to_output_string(_mag2[0], MAG_SIGN_LEN, MAG_DEC_LEN, MAG_PRECISION_LEN);
	add_double_variable_to_output_string(_mag2[1], MAG_SIGN_LEN, MAG_DEC_LEN, MAG_PRECISION_LEN);
	add_double_variable_to_output_string(_mag2[2], MAG_SIGN_LEN, MAG_DEC_LEN, MAG_PRECISION_LEN);
	gps_measurements(_gps_instance, _gps_serial_bus, _new_gps_data);

	// Pressure/temp/alt
	add_double_variable_to_output_string(_pressure, PRESS_SIGN_LEN, PRESS_DEC_LEN, PRESS_PRECISION_LEN);
	add_double_variable_to_output_string(_temperature, TEMP_SIGN_LEN, TEMP_DEC_LEN, TEMP_PRECISION_LEN);
	add_double_variable_to_output_string(_alt_pressure, ALT_SIGN_LEN, ALT_DEC_LEN, ALT_PRECISION_LEN);
	gps_measurements(_gps_instance, _gps_serial_bus, _new_gps_data);

	// GPS
	if(_gps_instance->fix == 1)
	{
		add_double_variable_to_output_string(_gps_instance->longitude/100.00, ANG_SIGN_LEN, ANG_DEC_LEN, ANG_PRECISION_LEN);
		add_char_variable_to_output_string(_gps_instance->lon);
		add_double_variable_to_output_string(_gps_instance->latitude/100.0, ANG_SIGN_LEN, ANG_DEC_LEN, ANG_PRECISION_LEN);
		add_char_variable_to_output_string(_gps_instance->lat);
		add_double_variable_to_output_string(_gps_instance->altitude, ALT_SIGN_LEN, ALT_DEC_LEN, ALT_PRECISION_LEN);

		add_double_variable_to_output_string(_gps_instance->speed, VEL_SIGN_LEN, VEL_DEC_LEN, VEL_PRECISION_LEN);
		add_double_variable_to_output_string(_gps_instance->angle, ANG_SIGN_LEN, ANG_DEC_LEN, ANG_PRECISION_LEN);

		add_double_variable_to_output_string((_gps_instance->hour + 2)%24, TIME_SIGN_LEN, TIME_DEC_LEN, TIME_PRECISION_LEN);
		add_double_variable_to_output_string(_gps_instance->minute, TIME_SIGN_LEN, TIME_DEC_LEN, TIME_PRECISION_LEN);
		add_double_variable_to_output_string(_gps_instance->seconds, TIME_SIGN_LEN, TIME_DEC_LEN, TIME_PRECISION_LEN);

		add_double_variable_to_output_string(_gps_instance->satellites, NSAT_SIGN_LEN, NSAT_DEC_LEN, NSAT_PRECISION_LEN);
		add_double_variable_to_output_string(*_new_gps_data, BOOL_SIGN_LEN, BOOL_DEC_LEN, BOOL_PRECISION_LEN);
		add_double_variable_to_output_string(_gps_instance->fix, BOOL_SIGN_LEN, BOOL_DEC_LEN, BOOL_PRECISION_LEN);
	}
	else
	{
		Serial.print("+000.0000;-;+000.0000;-;+0000.00;+000.0000;00;00;00;00;0;0;");
	}

  	Serial.println("*");
	gps_measurements(_gps_instance, _gps_serial_bus, _new_gps_data);

	return 1;
}

/*
 * Function gps_measurements() :
 *    Gathers the measurements from the gps (low data rates).
 */
uint8_t gps_measurements(Adafruit_GPS* _gnss_receiver, SoftwareSerial *_gps_serial_bus, uint8_t *_new_gps_data)
{
  if(_gnss_receiver->available() > 0)
  {
    if ((*_new_gps_data) == 0)
    {
      char c = _gnss_receiver->read();
      //Serial.print(c);
      
      if(_gnss_receiver->newNMEAreceived() && (*_new_gps_data) == 0)
      {
        if (_gnss_receiver->parse(_gnss_receiver->lastNMEA()))
        {
          //Serial.println(_gnss_receiver->lastNMEA());
          (*_new_gps_data) = 1;
        }
      }
    }
    else
    {
      char c = _gps_serial_bus->read(); // clearing the buffer without processing the data
      //Serial.print(c);
    }
  }

  return 1;
}
