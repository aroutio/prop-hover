#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t servoNumber = 1;
uint16_t pulseLength = 317;
uint16_t interval = 5;

void setup() {

  Serial.begin(115200);

  pwm.begin();

  pwm.setOscillatorFrequency(26500000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  Serial.println("\nPress any key to determine gimbal @ 90 deg using writeMicroseconds function.");
  while(!Serial.available());
  while(Serial.available()) Serial.read();

  pwm.setPWM(servoNumber, 0, pulseLength);
  delay(15);
}

void loop() {
  // Calibrates servo moving counter clockwise while looking at servo opposite of white spline gear.
  while(true){
    Serial.println("\nPress any key to increase servo angle by increasing microseconds a set interval.");
    while(!Serial.available());
    while(Serial.available()) Serial.read();
    pulseLength -= interval; // Change + to - to calibrate clockwise while looking at servo opposite of white spline gear.
    Serial.print(pulseLength);
    pwm.setPWM(servoNumber, 0, pulseLength);
  }
}
