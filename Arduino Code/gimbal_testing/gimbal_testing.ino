#include <Servo.h>

Servo upperGimbal;
Servo lowerGimbal;

int upperPosition = 0;
int lowerPosition = 0;

int upperGimbalLimit = 20;
int lowerGimbalLimit = 20;

int upperGimbalCenter = 96;
int lowerGimbalCenter = 96;

int gimbalSpeed = 50;

void setup() {

  Serial.begin(115200);

  upperGimbal.attach(10);
  lowerGimbal.attach(9);

  upperGimbal.write(upperGimbalCenter);
  lowerGimbal.write(lowerGimbalCenter);
  delay(15);

  upperGimbal.write(upperGimbalCenter - upperGimbalLimit);
  lowerGimbal.write(lowerGimbalCenter);
  delay(15);
}

void loop() {
  lowerPosition = lowerGimbalCenter;

  for (upperPosition = upperGimbalCenter - upperGimbalLimit; upperPosition <= upperGimbalCenter + upperGimbalLimit; upperPosition += 1) {

    if (upperPosition <= upperGimbalCenter + upperGimbalLimit && upperPosition >= upperGimbalCenter) {
      lowerPosition += 1;
    }

    if (upperPosition >= upperGimbalCenter - upperGimbalLimit && upperPosition <= upperGimbalCenter) {
      lowerPosition -= 1;
    }

    upperGimbal.write(upperPosition);
    lowerGimbal.write(lowerPosition);
    delay(gimbalSpeed);
  }

  for (upperPosition = upperGimbalCenter + upperGimbalLimit; upperPosition >= upperGimbalCenter - upperGimbalLimit; upperPosition -= 1) {

    if (upperPosition >= upperGimbalCenter - upperGimbalLimit && upperPosition <= upperGimbalCenter) {
      lowerPosition -= 1;
    }

    if (upperPosition <= upperGimbalCenter + upperGimbalLimit && upperPosition >= upperGimbalCenter) {
      lowerPosition += 1;
    }

    upperGimbal.write(upperPosition);
    lowerGimbal.write(lowerPosition);
    delay(gimbalSpeed);
  }
}
