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

  Serial.println("Starting ESCs Attached");

  ESC1.attach(9, 1000, 2000);
  ESC2.attach(10, 1000, 2000);
  Serial.println("Throttle Low");
  ESC1.writeMicroseconds(1000);
  ESC2.writeMicroseconds(1000);

  Serial.println("\nConnect Prop Battery then press a key to continue. Props will spin in 10 seconds.");
  while(!Serial.available());
  while(Serial.available()) Serial.read();
  delay(10000);

  Serial.println("Throttle 10%");
  ESC1.writeMicroseconds(1100);
  ESC2.writeMicroseconds(1100);
  delay(5000);

  Serial.println("Throttle 0%");
  ESC1.writeMicroseconds(1000);
  ESC2.writeMicroseconds(1000);

  Serial.println("Prop Testing Complete.");
}

void loop() 
{

}
