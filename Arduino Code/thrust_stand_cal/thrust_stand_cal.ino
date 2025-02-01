#include "HX711.h"

HX711 thruststand;

uint8_t dataPin  = 2;
uint8_t clockPin = 3;

void setup() 
{
  Serial.begin(115200);
  thruststand.begin(dataPin, clockPin);

  Serial.println("\nEnsure weight is removed from thrust stand to tare load cell then press a key to continue");
  while(!Serial.available());
  while(Serial.available()) Serial.read();

  Serial.print("Raw Value: ");
  Serial.println(thruststand.read());

  thruststand.tare(20);

  Serial.println("Apply mass to thrust stand, enter weight of mass in (whole) grams and press enter");
  uint32_t weight = 0;
  while (Serial.peek() != '\n')
  {
    if (Serial.available())
    {
      char ch = Serial.read();
      if (isdigit(ch))
      {
        weight *= 10;
        weight = weight + (ch - '0');
      }
    }
  }

  Serial.print("Weight [g]: ");
  Serial.println(weight);

  thruststand.calibrate_scale(weight, 1);

  Serial.print("Raw Value: ");
  Serial.println(thruststand.read());

  Serial.print("Scale Value: ");
  Serial.println(thruststand.get_scale());

  //Serial.print("Unit Value: ");
  //Serial.println(thruststand.get_units());

  //Serial.print("Value Value: ");
  //Serial.println(thruststand.get_value());

  //Serial.print("Tare Value: ");
  //Serial.println(thruststand.get_tare());

  //Serial.print("Offset Value: ");
  //Serial.println(thruststand.get_offset());
} 

void loop() 
{
// Nothing in the loop for this, using reset button to reset code
}
