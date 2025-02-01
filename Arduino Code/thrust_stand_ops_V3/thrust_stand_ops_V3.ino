// Make improvements to this code. There is a 5 seconds pause before the props spool up
// and radio comm telemetery does not work so remove the functionality from the code.

#include "HX711.h"
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>     // Arduino standard I2C/Two-Wire Library
#include <RH_RF69.h>
#include <Adafruit_PWMServoDriver.h>

#define PIN 40 // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 1 // Popular NeoPixel ring size

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#define RFM69_CS    4
#define RFM69_INT   3
#define RFM69_RST   2
#define LED        13

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

HX711 thruststand;
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
RH_RF69 rf69(RFM69_CS, RFM69_INT); // Singleton instance of the radio driver

uint8_t dataPin  = 6;
uint8_t clockPin = 7;

unsigned long endTime = 0;

// Defines variable for prop motors
uint8_t upperProp = 2;
uint8_t lowerProp = 3;

uint16_t highThrottle = 1000;
uint16_t lowThrottle = 0;

int radioCharLength = 60;

// Battery Voltage Read
const int analogPin = A1;

void setThrottle1(uint16_t throttle)
{
  throttle = map(throttle, 0, 1000, 1000, 2000);
  pwm.writeMicroseconds(upperProp, throttle);
}

void setThrottle2(int throttle)
{
  throttle = map(throttle, 0, 1000, 1000, 2000);
  pwm.writeMicroseconds(lowerProp, throttle);
}

void sendRadioComms(String data)
{
  // Convert the String to a char array
  char telemetry[data.length() + 1]; // Allocate enough space
  data.toCharArray(telemetry, sizeof(telemetry)); // Convert String to char array

  char radiopacket[radioCharLength]; //Increase this value to prevent cutting off characters when sending data
  strncpy(radiopacket, telemetry, sizeof(radiopacket) - 1); // Ensure safe copy
  radiopacket[sizeof(radiopacket) - 1] = '\0'; // Null-terminate for safetry
  Serial.println(radiopacket);

  // Send a message to the DESTINATION!
  rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
  rf69.waitPacketSent();

  // Now wait for a reply from the server
  Blink(LED, 1, 1); // blink LED 1 times, 1ms between blinks
}

void Blink(byte pin, byte delay_ms, byte loops) 
{
  while (loops--) {
    digitalWrite(pin, HIGH);
    delay(delay_ms);
    digitalWrite(pin, LOW);
    delay(delay_ms);
  }
}

void throttleLoop(int throttle, int runTime) 
{
  unsigned long startTime = millis();
  endTime = startTime;
  setThrottle1(throttle);
  setThrottle2(throttle);
  while((endTime - startTime) <= runTime) 
  {

    int voltageRead = analogRead(analogPin);
    float currentVoltage = (voltageRead * 3.3) / 1023.0;

    // Buid data string to send through radio comms
    String data = String(millis()) + "," + String(thruststand.get_units(2)) + "," + throttle + "," + String(currentVoltage);
    sendRadioComms(data);

    endTime = millis();
  }
}

void setup() 
{  
  digitalWrite(5, HIGH);

  delay(5000);

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.setPixelColor(0, pixels.Color(0, 0, 35));
  pixels.show();   // Send the updated pixel colors to the hardware.

  delay(5000);

  Serial.begin(115200);

  // Everything below this comment is from RadioHead69_AddrDemo_TX_Custom code to setup radio
  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the client
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
  rf69.setEncryptionKey(key);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
  // Radio code stops above this comment

  thruststand.begin(dataPin, clockPin);

  pwm.begin();
  pwm.setOscillatorFrequency(26500000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);

  setThrottle1(lowThrottle); // Set throttle to 0%
  setThrottle2(lowThrottle); // Set throttle to 0%
  Serial.println("\nThrottle LOW");
  sendRadioComms("Throttle LOW");

  digitalWrite(5, LOW);
  
  Serial.println("\nConnect Prop Battery");
  Serial.println("Ensure vehicle is preloaded to tare load cell");
  Serial.println("Code will move forward in 20 seconds");
  sendRadioComms("\nConnect Prop Battery");
  sendRadioComms("Ensure vehicle is preloaded to tare load cell");
  sendRadioComms("Code will move forward in 20 seconds");

  pixels.setPixelColor(0, pixels.Color(0, 35, 0));
  pixels.show(); // Send the updated pixel colors to the hardware.

  delay(20000);

  thruststand.tare(20);
  thruststand.set_scale(359.79);

  Serial.println("\nProp Armed and scale calibrated");
  sendRadioComms("Prop Armed and scale calibrated");

  pixels.setPixelColor(0, pixels.Color(35, 35, 0));
  pixels.show(); // Send the updated pixel colors to the hardware.

  Serial.println("\nProp will spool up in 10 seconds");
  sendRadioComms("Prop will spool up in 10 seconds");

  delay(10000);

  pixels.setPixelColor(0, pixels.Color(35, 0, 0));
  pixels.show(); // Send the updated pixel colors to the hardware.
}

void loop() 
{
  throttleLoop(0, 5000);
  throttleLoop(1000, 5000);

  digitalWrite(5, HIGH);
  throttleLoop(0, 100000);
}