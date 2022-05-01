#include "Arduino.h"

#ifndef __SD_HELPERS_H__
#define __SD_HELPERS_H__

#include <SPI.h>
#include <SD.h>

#define SD_CS_PIN 10
#define STARTING_INDEX 4

#define SLAVE_LED_PIN_POWER 7
#define SLAVE_LED_PIN_INIT 6
#define SLAVE_LED_PIN_RW_SD 5
#define SLAVE_LED_PIN_SD_STATUS 4

uint8_t log_to_SD(char* _path, File* _f, char *_serial_buffer, uint8_t _buffer_length);
uint8_t log_data(File* _f, char _data);
uint8_t connect_to_SD(char* _path, File* _f, uint8_t _createNew);

#endif
