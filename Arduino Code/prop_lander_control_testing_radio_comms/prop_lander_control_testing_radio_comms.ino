#include <Servo.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <RH_RF69.h>

#define DEV_I2C Wire
#define PIN 40 // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 1 // Popular NeoPixel ring size

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#define RFM69_CS    4
#define RFM69_INT   3
#define RFM69_RST   2
#define LED        13

VL53L4CX sensor(&DEV_I2C, A1);  // Create sensor instance on I2C bus, with A1 as an interrupt pin (optional)
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Servo ESC1;
Servo ESC2;
RH_RF69 rf69(RFM69_CS, RFM69_INT); // Singleton instance of the radio driver

int distance = -1;  // Variable to store the measured distance in mm

// PID gain definitions
float Kp = 1; // Proportional gain
float Ki = 0; // Integral gain
float Kd = 0.6; // Derivative gain

int desiredHeight = 400; // Desired height (in mm)

// Control variable definitions
float currentHeight = 0;
float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;
float output = 0;

// Add a maximum and minimum limit for the integral term
float integralMax = 100; // Set these limits based on this system
float integralMin = -100;

float alpha = 0.6; // Smoothing factor (0 < smoothingFactor < 1)
float filteredOutput = 0; // Initial filtered Output

float throttleOutput = 0;      // Current throttle output
float throttleTarget = 0;      // Target throttle from the PID
float maxSlewRate = 0.5;       // Maximum change per control loop (e.g., 0.5% per iteration)

float deltaTime = 0.050; // Controls the pace of system control loop

unsigned long previousTime = 0;


int getDistance()
{
  VL53L4CX_MultiRangingData_t measurementData;  // Structure to hold sensor data
  uint8_t newDataReady = 0;
  int status;

  // Check if new measurement data is ready
  status = sensor.VL53L4CX_GetMeasurementDataReady(&newDataReady);
  if (status == 0 && newDataReady) { // If data is ready and no errors, get measurement data
   
    sensor.VL53L4CX_GetMultiRangingData(&measurementData);

    // Update distance variable with the first object's distance (if an object is detected)
    if (measurementData.NumberOfObjectsFound == 0) {
      distance = -1;
    }

    if (measurementData.NumberOfObjectsFound == 1) {
      distance = measurementData.RangeData[0].RangeMilliMeter;
    }

    if (measurementData.NumberOfObjectsFound == 2) {
      distance = measurementData.RangeData[1].RangeMilliMeter;
    } 

    // Clear the interrupt and start a new measurement
    sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
  }

  return distance;
}

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

void sendRadioComms(String data)
{
  // Convert the String to a char array
  char telemetry[data.length() + 1]; // Allocate enough space
  data.toCharArray(telemetry, sizeof(telemetry)); // Convert String to char array

  char radiopacket[70];
  strncpy(radiopacket, telemetry, sizeof(radiopacket) - 1); // Ensure safe copy
  radiopacket[sizeof(radiopacket) - 1] = '\0'; // Null-terminate for safetry
  Serial.println(radiopacket);

  // Send a message to the DESTINATION!
  rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
  rf69.waitPacketSent();

  // Now wait for a reply from the server
  Blink(LED, 1, 1); // blink LED 1 times, 40ms between blinks
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

float lowPassFilter(float output) {
  // Apply low-pass filter formula
  filteredOutput = alpha * output + (1 - alpha) * filteredOutput;
  return filteredOutput;
}

// Not being used yet
void SlewRateLimitedThrottle(int throttleTarget) {
  float maxChange = maxSlewRate * deltaTime; // Max allowable change in this time step
  
  // Limit the rate of change
  if (throttleTarget > throttleOutput) {
    throttleOutput = min(throttleOutput + maxChange, throttleTarget);
  } else {
    throttleOutput = max(throttleOutput - maxChange, throttleTarget);
  }
  
  // Send the limited throttle to the motor
  setThrottle1(throttleOutput);
  setThrottle2(throttleOutput);
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

  // Initialize I2C communication
  DEV_I2C.begin();

  // Initialize the VL53L4CX sensor
  sensor.begin();
  sensor.VL53L4CX_Off();           // Ensure the sensor is off first
  sensor.InitSensor(0x12);          // Initialize sensor with default settings
  sensor.VL53L4CX_StartMeasurement(); // Start continuous measurement

  ESC1.attach(9, 1000, 2000);
  ESC2.attach(10, 1000, 2000);
  
  setThrottle1(0); // Set throttle to 0%
  setThrottle2(0); // Set throttle to 0%
  Serial.println("Throttle LOW");
  sendRadioComms("Throttle LOW");

  Serial.println("\nConnect Prop Battery, code will move forward in 10 seconds");
  sendRadioComms("Connect Prop Battery, code will move forward in 10 seconds");

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

  Serial.println("CLEARDATA");
  sendRadioComms("CLEARDATA");

  Serial.println("LABEL,Time [s],Height [mm],Throttle [%],Error,Integral,Derivative");
  sendRadioComms("LABEL,Time [s],Height [mm],Throttle [%],Error,Integral,Derivative");

  Serial.println("RESETTIMER");
  sendRadioComms("RESETTIMER");
}

void loop() 
{
  unsigned long currentTime = millis(); // Get current time

  // Run PID loop at regular intervals
  if (currentTime - previousTime >= deltaTime * 1000) {
    float elapsedTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    int currentHeight = lowPassFilter(getDistance()); // Read height sensor

    error = desiredHeight - currentHeight; // Calculate error

    integral += error * elapsedTime;
    
    // Clamp the integral term
    integral = constrain(integral, integralMin, integralMax);

    derivative = (error - previousError) / elapsedTime; // Calculate derivative

    output = Kp * error + Ki * integral + Kd * derivative; // Compute the PID output
  
    output = constrain(output, 70, 100); // Ensure output is within prop control range in terms of % throttle

    setThrottle1(output); // Apply output to prop
    setThrottle2(output); // Apply output to prop

    previousError = error;

    String data = "DATA," + String(currentTime) + "," + String(currentHeight) + "," + String(output) + "," + String(error) + "," + String(integral) + "," + String(derivative);

    // Call the radioComms function
    sendRadioComms(data);
  }
}
