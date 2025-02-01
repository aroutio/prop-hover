#include "HX711.h"
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

#define PIN 40 // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 1 // Popular NeoPixel ring size

HX711 thruststand;
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Servo ESC1;
Servo ESC2;

uint8_t dataPin  = 2;
uint8_t clockPin = 3;

unsigned long endTime = 0;

void setThrottle1(int throttle)
{
  throttle = map(throttle, 0, 100, 1000, 2000);
  ESC1.writeMicroseconds(throttle);
}

void setThrottle2(int throttle)
{
  throttle = map(throttle, 0, 100, 1000, 2000);
  ESC2.writeMicroseconds(throttle);
}

void throttleLoop(int throttle, int runTime) 
{
  unsigned long startTime = millis();
  endTime = startTime;
  setThrottle1(throttle);
  setThrottle2(throttle);
  while((endTime - startTime) <= runTime) 
  {
    Serial.println( (String) "DATA,DATE,TIME," + millis() + "," + thruststand.get_units(3) + "," + throttle );
    endTime = millis();
  }
}

void setup() 
{
  delay(5000);

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.setPixelColor(0, pixels.Color(0, 0, 35));
  pixels.show();   // Send the updated pixel colors to the hardware.

  delay(5000);

  Serial.begin(115200);
  Serial.println("Starting serial comms");
  thruststand.begin(dataPin, clockPin);

  ESC1.attach(9, 1000, 2000);
  ESC2.attach(10, 1000, 2000);
  
  setThrottle1(0); // Set throttle to 0%
  setThrottle2(0); // Set throttle to 0%
  Serial.println("Throttle LOW");

  Serial.println("\nConnect Prop Battery");

  Serial.println("\nEnsure vehicle is preloaded to tare load cell");

  Serial.println("\nCode will move forward in 20 seconds");

  pixels.setPixelColor(0, pixels.Color(0, 35, 0));
  pixels.show(); // Send the updated pixel colors to the hardware.

  delay(20000);

  Serial.println("Prop Armed, and scale calibrated");

  thruststand.tare(20);
  thruststand.set_scale(944.14);

  pixels.setPixelColor(0, pixels.Color(35, 35, 0));
  pixels.show(); // Send the updated pixel colors to the hardware.

  Serial.println("\nProp will spool up in 10 seconds");
  delay(10000);

  pixels.setPixelColor(0, pixels.Color(35, 0, 0));
  pixels.show(); // Send the updated pixel colors to the hardware.

  Serial.println("CLEARDATA");
  Serial.println("LABEL,Date,Time,Millis,Thrust Force [g]");
  Serial.println("RESETTIMER");
}

void loop() 
{
  throttleLoop(0, 5000);
  throttleLoop(10, 5000);
  throttleLoop(20, 5000);
  throttleLoop(30, 5000);
  throttleLoop(40, 5000);
  throttleLoop(50, 5000);
  throttleLoop(60, 5000);
  throttleLoop(70, 5000);
  throttleLoop(80, 5000);
  throttleLoop(90, 5000);
  throttleLoop(100, 5000);

  throttleLoop(95, 5000);
  throttleLoop(100, 5000);
  throttleLoop(90, 5000);
  throttleLoop(100, 5000);
  throttleLoop(85, 5000);
  throttleLoop(100, 5000);
  throttleLoop(80, 5000);
  throttleLoop(100, 5000);
  throttleLoop(75, 5000);
  throttleLoop(100, 5000);

  throttleLoop(0, 10000);
  throttleLoop(100, 5000);
  throttleLoop(0, 10000);
  throttleLoop(75, 5000);
  throttleLoop(0, 100000);
}