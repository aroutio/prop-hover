// This code is for two ESCs
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver ESC = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t upperGimbal = 2;
uint8_t lowerGimbal = 3;
uint16_t highThrottle = 2000;
uint16_t lowThrottle = 1000;

void setup() 
{
  delay(5000);

  Serial.begin(115200);

  Serial.println("\nArduino (trasmitter) is on, press a key to continue");
  while(!Serial.available());
  while(Serial.available()) Serial.read();

  ESC.begin();
  ESC.setOscillatorFrequency(26500000);
  ESC.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  Serial.println("Starting ESC Calibration");

  Serial.println("Throttle HIGH");
  ESC.writeMicroseconds(upperGimbal, highThrottle);
  ESC.writeMicroseconds(lowerGimbal, highThrottle);

  Serial.println("\nConnect Prop Battery then press a key to continue");
  while(!Serial.available());
  while(Serial.available()) Serial.read();
  Serial.println("\nKey press detected");
  delay(4000);

  Serial.println("Throttle LOW");
  ESC.writeMicroseconds(upperGimbal, lowThrottle);
  ESC.writeMicroseconds(lowerGimbal, lowThrottle);
  delay(4000);

  Serial.println("ESC Calibrated");
}

void loop() 
{

}
