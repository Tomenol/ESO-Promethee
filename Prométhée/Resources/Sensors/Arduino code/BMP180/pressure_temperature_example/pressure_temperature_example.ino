#include <Wire.h>

#include "I2C.h"
#include "BMP180.h"

uint32_t t;

BMP180 bmp180;

void setup() {
  Serial.begin(4800);
  I2C::begin();

  delay(500);
  bmp180.initialize();
  
  t = 0;
}

void loop()
{
  int32_t pressure, temperature;
  bmp180.get_temperature_and_pressure(&temperature, &pressure);
  
  Serial.print("pressure : ");
  Serial.print(pressure, DEC);
  
  Serial.print(", temperature : ");
  Serial.println(temperature, DEC);

  delay(1000);

  t += 1;
}
