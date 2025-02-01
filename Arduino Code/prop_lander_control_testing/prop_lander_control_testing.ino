#include <Servo.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <Adafruit_NeoPixel.h>

#define DEV_I2C Wire
#define PIN 40 // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 1 // Popular NeoPixel ring size

VL53L4CX sensor(&DEV_I2C, A1);  // Create sensor instance on I2C bus, with A1 as an interrupt pin (optional)
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Servo ESC1;
Servo ESC2;

int distance = -1;  // Variable to store the measured distance in mm

// PID gain definitions
float Kp = 1; // Proportional gain
float Ki = 0.1; // Integral gain
float Kd = 0.8; // Derivative gain

int desiredHeight = 300; // Desired height (in mm)

// Control variable definitions
float currentHeight = 0;
float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;
float output = 0;

float deltaTime = 0.01; // Controls the pace of system control loop
float daqDeltaTime = 0.05;

unsigned long previousTime = 0;
unsigned long previousDAQTime = 0;

int getDistance()
{
  VL53L4CX_MultiRangingData_t measurementData;  // Structure to hold sensor data
  uint8_t newDataReady = 0;
  int status;

  // Check if new measurement data is ready
  status = sensor.VL53L4CX_GetMeasurementDataReady(&newDataReady);
  if (status == 0 && newDataReady) // If data is ready and no errors, get measurement data
  { 
    sensor.VL53L4CX_GetMultiRangingData(&measurementData);

    // Update distance variable with the first object's distance (if an object is detected)
    if (measurementData.NumberOfObjectsFound = 1) 
    {
      distance = measurementData.RangeData[0].RangeMilliMeter;
    } else 
    {
      //Serial.println("No object detected");
      distance = -1;  // Set distance to -1 if no object is detected
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

void setup() 
{
  delay(5000);

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.setPixelColor(0, pixels.Color(0, 0, 35));
  pixels.show();   // Send the updated pixel colors to the hardware.

  delay(5000);

  Serial.begin(115200);
  Serial.println("Starting serial comms");

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

  Serial.println("\nConnect Prop Battery, code will move forward in 10 seconds");

  pixels.setPixelColor(0, pixels.Color(0, 35, 0));
  pixels.show(); // Send the updated pixel colors to the hardware.

  delay(10000);

  Serial.println("Prop Armed");

  pixels.setPixelColor(0, pixels.Color(35, 35, 0));
  pixels.show(); // Send the updated pixel colors to the hardware.

  Serial.println("\nProp will spool up in 10 seconds");
  delay(10000);

  pixels.setPixelColor(0, pixels.Color(35, 0, 0));
  pixels.show(); // Send the updated pixel colors to the hardware.

  Serial.println("CLEARDATA");
  Serial.println("LABEL,Date,Time,Millis,Thrust Force [g]");
  Serial.println("RESETTIMER");
}

void loop() 
{
  unsigned long currentTime = millis(); // Get current time

  // Run PID loop at regular intervals
  if (currentTime - previousTime >= deltaTime * 1000) {
    previousTime = currentTime;

    int currentHeight = getDistance(); // Read height sensor

    error = desiredHeight - currentHeight; // Calculate error

    integral += error * deltaTime; // Calculate integral

    derivative = (error - previousError) / deltaTime; // Calculate derivative

    output = Kp * error + Ki * integral + Kd * derivative; // Compute the PID output
  
    output = constrain(output, 75, 100); // Ensure output is within prop control range in terms of % throttle

    setThrottle1(output); // Apply output to prop
    setThrottle2(output); // Apply output to prop

    previousError = error;
  }

  if (currentTime - previousDAQTime >= daqDeltaTime * 1000) {
    previousDAQTime = currentTime;

    Serial.println( (String) "DATA,DATE,TIME," + millis() + "," + getDistance() + "," + output );
  }
}
