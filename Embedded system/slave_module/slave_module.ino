#include "SD_helper.h"

char file_path[] = "dataxxxx.txt";
File sd_log_file;

char serial_buffer[256];
uint8_t buffer_length;

void setup() 
{
  Serial.begin(115200);
  
  pinMode(SLAVE_LED_PIN_POWER, OUTPUT);
  pinMode(SLAVE_LED_PIN_INIT, OUTPUT);
  pinMode(SLAVE_LED_PIN_RW_SD, OUTPUT);
  pinMode(SLAVE_LED_PIN_SD_STATUS, OUTPUT);

  digitalWrite(SLAVE_LED_PIN_POWER, 1);

  // Start slave initialization
  //while(!connect_to_SD(file_path, &sd_log_file, 1));

  digitalWrite(SLAVE_LED_PIN_INIT, 1);
  digitalWrite(SLAVE_LED_PIN_SD_STATUS, 1);
}

void loop() 
{
  if(read_serial_data() == 1)
  {
    log_to_SD(file_path, &sd_log_file, serial_buffer, buffer_length);
  }
}

uint8_t read_serial_data()
{
  buffer_length = 0;
  
  if(Serial.available() > 0)
  {
    do
    {
      if(Serial.available() > 0)
      {
        serial_buffer[buffer_length] = Serial.read();
        Serial.print(serial_buffer[buffer_length]);
        
        buffer_length++;
      }
    } while(serial_buffer[buffer_length - 1] != '*');
    Serial.println("");
    
    if(serial_buffer[0] != '$') return 0;
    else return 1;
  }

  return 0;
}
