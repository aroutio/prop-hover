#include "HX711.h"
#include <Servo.h>

HX711 thruststand;
Servo ESC;

uint8_t dataPin  = 2;
uint8_t clockPin = 3;

unsigned long endTime = 0;

void throttleLoop(int throttle, int runTime) 
{
  unsigned long startTime = millis();
  endTime = startTime;
  ESC.writeMicroseconds(throttle);
  while((endTime - startTime) <= runTime) 
  {
    Serial.println( (String) "DATA,DATE,TIME," + millis() + "," + thruststand.get_units(3) + "," + throttle );
    endTime = millis();
  }
}
  

void setup() 
{
  Serial.begin(115200);
  thruststand.begin(dataPin, clockPin);

  Serial.println("\nEnsure weight is removed from thrust stand to tare load cell, code will move forward in 10 seconds");
  delay(10000);

  thruststand.tare(20);
  thruststand.set_scale(1051.60);

  Serial.println("Arming Prop");

  ESC1.attach(9, 1000, 2000);
  ESC2.attach(10, 1000, 2000);
  Serial.println("Throttle LOW");
  ESC.writeMicroseconds(1000);

  Serial.println("\nConnect Prop Battery, code will move forward in 10 seconds");
  delay(10000);

  Serial.println("Prop Armed");

  Serial.println("\nData aquistion starts in 10 seconds");
  delay(10000);

  Serial.println("\nThrottle 10%");

  Serial.println("CLEARDATA");
  Serial.println("LABEL,Date,Time,Millis,Thrust Force [g]");
  Serial.println("RESETTIMER");

  ESC.writeMicroseconds(1000);
}

void loop() 
{
  throttleLoop(1000, 5000);
  throttleLoop(1100, 5000);
  throttleLoop(1200, 5000);
  throttleLoop(1300, 5000);
  throttleLoop(1400, 5000);
  throttleLoop(1500, 5000);
  throttleLoop(1600, 5000);
  throttleLoop(1700, 5000);
  throttleLoop(1800, 5000);
  throttleLoop(1900, 5000);
  throttleLoop(2000, 5000);

  throttleLoop(1000, 10000);
  throttleLoop(1100, 5000);
  throttleLoop(1000, 10000);
  throttleLoop(1200, 5000);
  throttleLoop(1000, 10000);
  throttleLoop(1300, 5000);
  throttleLoop(1000, 10000);
  throttleLoop(1400, 5000);
  throttleLoop(1000, 10000);
  throttleLoop(1500, 5000);
  throttleLoop(1000, 10000);
  throttleLoop(1600, 5000);
  throttleLoop(1000, 10000);
  throttleLoop(1700, 5000);
  throttleLoop(1000, 10000);
  throttleLoop(1800, 5000);
  throttleLoop(1000, 10000);
  throttleLoop(1900, 5000);
  throttleLoop(1000, 10000);
  throttleLoop(2000, 5000);
}
