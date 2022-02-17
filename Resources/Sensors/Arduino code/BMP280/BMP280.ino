#include "BMP280.h"
#include "I2C.h"
//test
BMP280 bmp_280;

double pressure, temperature;

void setup() {
  Serial.begin(115200);
  I2C::begin();
  
  Serial.println(bmp_280.initialize());

  delay(1000);

  bmp_280.setReferenceAltitude();
}

void loop() 
{
  if(bmp_280.readValues(&temperature, &pressure) == 0) Serial.println("Error reading BMP180");
  else
  {
    Serial.print("\nTemperature & pressure & altitude readings : ");
    Serial.print(temperature, DEC);
    Serial.print(", ");
    Serial.print(pressure, DEC);
    Serial.print(", ");
    Serial.print(bmp_280.getAltitude(pressure, temperature), DEC);
  }
  
  delay(50);
}
