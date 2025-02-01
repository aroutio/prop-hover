#include <Adafruit_NeoPixel.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>     // Arduino standard I2C/Two-Wire Library
#include <SPI.h>
#include <RH_RF69.h>
#include <TFMPI2C.h>  // TFMini-Plus I2C Library v1.7.3
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

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
RH_RF69 rf69(RFM69_CS, RFM69_INT); // Singleton instance of the radio driver
TFMPI2C distanceSensor;
Adafruit_MPU6050 mpu;

float deltaTime = 0.050; // Controls the pace of system control loop
unsigned long previousTime = 0;

int radioCharLength = 60;

// Defines variables for LIDAR sensor
int16_t distance = 0;       // Distance to object in centimeters
int16_t strength = 0;       // Signal strength or quality of return signal
int16_t temperature = 0;       // Internal temperature of Lidar sensor chip
int16_t sensorOutput = 0;

// Defines variable for gimbal Servos
uint8_t upperGimbal = 0;
uint8_t lowerGimbal = 1;

uint16_t upperGimbalCenter = 312;
uint16_t lowerGimbalCenter = 317;

// Defines variable for prop motors
uint8_t upperProp = 2;
uint8_t lowerProp = 3;

uint16_t highThrottle = 1000;
uint16_t lowThrottle = 0;

// Defines variable for complementary filter
int previousHeight = 0;
float currentVelocity = 0;

// Accelerometer variable definitions
float offsetZ = -2.33;
float scaleFactorZ = 0.1;

// Battery Voltage Read
const int analogPin = A1;

void getDistance() {
  distanceSensor.getData(distance, strength, temperature); // Get a frame of data
  if(distanceSensor.status == TFMP_READY)         // If no error...
  {
    sensorOutput = distance;                             // end-of-line.
  }
  else
  {
    if(distanceSensor.status == TFMP_I2CWRITE)  // If I2C error...
    {
      distanceSensor.recoverI2CBus();          // recover hung bus.
    }
  }
}

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

  distanceSensor.recoverI2CBus();

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

  // MPU code below this comment
  // MPU init attempt
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    sendRadioComms("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  sendRadioComms("MPU6050 Found!");

  // MPU settings init
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  delay(100);

  pwm.begin();
  pwm.setOscillatorFrequency(26500000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);

  setThrottle1(lowThrottle); // Set throttle to 0%
  setThrottle2(lowThrottle); // Set throttle to 0%
  Serial.println("Throttle LOW");
  sendRadioComms("Throttle LOW");

  digitalWrite(5, LOW);

  pwm.setPWM(upperGimbal, 0, upperGimbalCenter);
  delay(15);
  pwm.setPWM(lowerGimbal, 0, upperGimbalCenter);
  delay(15);
  Serial.println("Gimbal Centered");
  sendRadioComms("Gimbal Centered");
  
  Serial.println("\nConnect Prop Battery code will move forward in 10 seconds");
  sendRadioComms("Connect Prop Battery code will move forward in 10 seconds");

  pixels.setPixelColor(0, pixels.Color(0, 35, 0));
  pixels.show(); // Send the updated pixel colors to the hardware.

  delay(10000);

  Serial.println("Prop Armed");
  sendRadioComms("Prop Armed");

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
  unsigned long currentTime = millis(); // Get current time

  // Run PID loop at regular intervals
  if (currentTime - previousTime >= deltaTime * 1000) {
    float elapsedTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float currentAcceleration = ((a.acceleration.z - offsetZ) * scaleFactorZ) - 1;
    
    getDistance();
    int currentHeight = sensorOutput;

    currentVelocity = (currentHeight - previousHeight) / deltaTime;

    int voltageRead = analogRead(analogPin);

    float currentVoltage = (voltageRead * 3.3) / 1023.0;

    int currentThrottle = 500;

    setThrottle1(currentThrottle);
    setThrottle2(currentThrottle);

    previousHeight = currentHeight;

    // Buid data string to send through radio comms
    String data = String(currentTime) + "," + String(currentHeight) + "," + String(currentVelocity) + "," + String(currentAcceleration) + "," + currentThrottle + "," + String(currentVoltage);
    sendRadioComms(data);
  }
}
