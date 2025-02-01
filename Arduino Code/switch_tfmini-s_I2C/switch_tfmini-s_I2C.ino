#include <SoftwareSerial.h>

SoftwareSerial tfminiSerial(2, 3); // RX, TX (adjust pins as needed)

void setup() {
  Serial.begin(115200);   // Initialize serial monitor for debugging
  tfminiSerial.begin(115200); // Initialize TFmini-S UART communication

  // Wait for initialization
  delay(1000);

  // Send command to switch to I2C mode
  uint8_t switchToI2C[] = {0x5A, 0x05, 0x0A, 0x01, 0x6A}; // Command to switch to I2C mode
  sendCommand(switchToI2C, sizeof(switchToI2C));

  // Wait for a moment after switching modes
  delay(1000);

  // Send command to save settings (including mode change)
  uint8_t saveSettings[] = {0x5A, 0x04, 0x11, 0x6F}; // Command to save settings
  sendCommand(saveSettings, sizeof(saveSettings));

  // Wait a moment for the settings to be saved
  delay(1000);

  Serial.println("Mode switched to I2C and settings saved.");
}

void sendCommand(uint8_t *command, size_t length) {
  tfminiSerial.write(command, length);  // Send command to TFmini-S over UART
  Serial.println("Command sent.");
}

void loop() {
  // Nothing to do here
}

