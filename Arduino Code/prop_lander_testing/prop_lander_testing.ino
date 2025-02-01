#include <Servo.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <Adafruit_NeoPixel.h>

#define DEV_I2C Wire
#define PIN 40 // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 1 // Popular NeoPixel ring size

VL53L4CX sensor(&DEV_I2C, A1);  // Create sensor instance on I2C bus, with A1 as an interrupt pin (optional)
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Servo ESC;

int distance = -1;  // Variable to store the measured distance in mm

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

  ESC.attach(9, 1000, 2000);
  
  Serial.println("Throttle LOW");
  ESC.writeMicroseconds(1000);

  Serial.println("\nConnect Prop Battery, code will move forward in 10 seconds");

  pixels.setPixelColor(0, pixels.Color(0, 35, 0));
  pixels.show();   // Send the updated pixel colors to the hardware.

  delay(10000);

  Serial.println("Prop Armed");

  pixels.setPixelColor(0, pixels.Color(35, 35, 0));
  pixels.show();   // Send the updated pixel colors to the hardware.

  Serial.println("\nProp will spool up in 10 seconds");
  delay(10000);

  pixels.setPixelColor(0, pixels.Color(35, 0, 0));
  pixels.show();   // Send the updated pixel colors to the hardware.

  ESC.writeMicroseconds(2000); // this line says set throttle at 100%, not supposed to be here
}

void loop() 
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
      Serial.print("Distance: ");
      Serial.print(distance); // Print distance in mm
      Serial.println(" mm");
    } else 
    {
      //Serial.println("No object detected");
      distance = -1;  // Set distance to -1 if no object is detected
    }

    // Clear the interrupt and start a new measurement
    sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
  }

  delay(10);  // Small delay between readings
}
