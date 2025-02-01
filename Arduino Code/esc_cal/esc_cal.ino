// This code is for two ESCs
#include <Servo.h>

Servo ESC1;
Servo ESC2;

void setup() 
{
  delay(5000);

  Serial.begin(115200);

  Serial.println("\nArduino (trasmitter) is on, press a key to continue");
  while(!Serial.available());
  while(Serial.available()) Serial.read();

  Serial.println("Starting ESC Calibration");

  ESC1.attach(9, 1000, 2000);
  ESC2.attach(10, 1000, 2000);
  Serial.println("Throttle HIGH");
  ESC1.writeMicroseconds(2000);
  ESC2.writeMicroseconds(2000);

  Serial.println("\nConnect Prop Battery then press a key to continue");
  while(!Serial.available());
  while(Serial.available()) Serial.read();
  delay(4000);

  Serial.println("Throttle LOW");
  ESC1.writeMicroseconds(1000);
  ESC2.writeMicroseconds(1000);
  delay(4000);

  Serial.println("ESC Calibrated");
}

void loop() 
{

}
