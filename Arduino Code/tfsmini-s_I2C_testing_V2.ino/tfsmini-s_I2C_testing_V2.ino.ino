#include <Wire.h>     // Arduino standard I2C/Two-Wire Library
#include <TFMPI2C.h>  // TFMini-Plus I2C Library v1.7.3

TFMPI2C tfmP;         // Create a TFMini-Plus I2C object

// Initialize data variables
int16_t distance = 0;       // Distance to object in centimeters
int16_t strength = 0;       // Signal strength or quality of return signal
int16_t temperature = 0;       // Internal temperature of Lidar sensor chip

void setup() {

  Serial.begin(115200);
  // put your setup code here, to run once:
  tfmP.recoverI2CBus();
  delay(500);
}

void loop() {
  tfmP.getData(distance, strength, temperature); // Get a frame of data
  if( tfmP.status == TFMP_READY)         // If no error...
  {
    Serial.println(String(distance));                     // end-of-line.
  }
  else
  {
    tfmP.printFrame();                 // Display error and data frame
    if( tfmP.status == TFMP_I2CWRITE)  // If I2C error...
    {
      tfmP.recoverI2CBus();          // recover hung bus.
    }
  }
  delay(50);    //  Run loop at approximately 20Hz.

}
