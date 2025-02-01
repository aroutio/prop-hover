#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t servoNumber = 0;
uint16_t gimbalUs = 1520;
uint16_t interval = 15;

void setup() {

  Serial.begin(115200);

  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  Serial.println("\nPress any key to determine gimbal @ 90 deg using writeMicroseconds function.");
  while(!Serial.available());
  while(Serial.available()) Serial.read();

  pwm.writeMicroseconds(servoNumber, gimbalUs);
  delay(15);
}

void loop() {
  // Calibrates servo moving counter clockwise while looking at servo opposite of white spline gear.
  while(true){
    Serial.println("\nPress any key to increase servo angle by increasing microseconds a set interval.");
    while(!Serial.available());
    while(Serial.available()) Serial.read();
    gimbalUs += 15; // Change + to - to calibrate clockwise while looking at servo opposite of white spline gear.
    Serial.print(gimbalUs);
    pwm.writeMicroseconds(servoNumber, gimbalUs);
  }
}
