#include "SD_helper.h"

uint8_t log_to_SD(char* _path, File* _f, char *_serial_buffer, uint8_t _buffer_length)
{  
  *_f = SD.open(_path, FILE_WRITE);
  
  if (*_f)
  {    
    for(uint8_t i = 0; i < _buffer_length; i++)
    {
      digitalWrite(SLAVE_LED_PIN_RW_SD, 1);
      
      if(log_data(_f, _serial_buffer[i]) == 0)
      {
        _f->close();

        goto fail;
      }
      
      digitalWrite(SLAVE_LED_PIN_RW_SD, 0);
    }

    _f->close();
    return 1;
  }
  else 
  {
    goto fail;
  }
  
  fail:
    digitalWrite(SLAVE_LED_PIN_RW_SD, 0);
    
    while (connect_to_SD(_path, _f, 1) == 0);    
    while(Serial.available() > 0) Serial.read(); // clear serial buffer
  
    return 0;
}

uint8_t log_data(File* _f, char _data)
{  
  if(_data == '$')
  {
    if(_f->print("\n") == 0) return 0;

    if(_f->print(millis()) == 0) return 0;
    if(_f->print("; ") == 0) return 0;
  }
  else
  {
    if(_f->print(_data) == 0) return 0;
  }

  return 1;
}

uint8_t connect_to_SD(char *_path, File* _f, uint8_t _createNew)
{
  int indice = 0;

  digitalWrite(SLAVE_LED_PIN_SD_STATUS, 0);
  
  if (!SD.begin(SD_CS_PIN))
  {    
    return 0;
  }
  else
  {    
    if (_createNew == 1)
    { 
      do 
      {
        if(indice > 9999)
        {
          delay(100000000);
        }
        
        _path[STARTING_INDEX] = (indice / 1000) + '0';
        _path[STARTING_INDEX + 1] = ((indice % 1000) / 100) + '0'; 
        _path[STARTING_INDEX + 2] = ((indice % 1000) % 100 / 10) + '0';
        _path[STARTING_INDEX + 3] = ((indice % 1000) % 100 % 10) + '0';
                
        indice++;
      } while (SD.exists(_path));
      
      *_f = SD.open(_path, O_CREAT | O_WRITE);

      if(!(*_f))
      {
        _f->close();
        return 0;
      }
            
      _f->println("t; ax1; ay1; az1; ax2; ay2; az2; wx1; wy1; wz1; wx2; wy2; wz2; mx1; my1; mz1; mx2; my2; mz2; longitude; latitude; altitude; velocity; heading; vz; T; P; z_pressure;"); // 27 
      _f->close(); // do not close file for higher data rates

      digitalWrite(SLAVE_LED_PIN_SD_STATUS, 1);
      
      while(Serial.available() > 0) Serial.read();
    }
    return 1;
  }
}
