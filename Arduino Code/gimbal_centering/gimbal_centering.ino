#include <Servo.h>

Servo upperGimbal;
Servo lowerGimbal;

int upperGimbalCenter = 96;
int lowerGimbalCenter = 96;

int gimbalSpeed = 15;

void setup() {

  Serial.begin(115200);

  upperGimbal.attach(10);
  lowerGimbal.attach(9);

  upperGimbal.write(upperGimbalCenter);
  lowerGimbal.write(lowerGimbalCenter);
  delay(15);
}

void loop() {

}
