#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float offsetZ = 0;
float scaleFactorZ = 0;

const float gravity = 9.81;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("Calibrating accelerometer for Z-axis...");
  calibrateZAxis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply offset and scale factor for Z-axis
  float az = (a.acceleration.z - offsetZ) * scaleFactorZ * gravity;

  // Print calibrated Z-axis value
  Serial.print("Accel Z (m/s^2): ");
  Serial.println(az);

  delay(100);
}

void calibrateZAxis() {
  float z_up = 0, z_down = 0;

  // Collect readings for Z-axis UP
  z_up = collectZAxisAverage("Align Z-axis UP (sensor flat on its back) and press ENTER.");

  // Collect readings for Z-axis DOWN
  z_down = collectZAxisAverage("Align Z-axis DOWN (sensor flat on its front) and press ENTER.");

  // Calculate offset
  offsetZ = (z_up + z_down) / 2.0;

  // Calculate scale factor
  scaleFactorZ = 2.0 / abs(z_up - z_down);

  Serial.println("Z-axis Calibration Complete!");
  Serial.print("Offset Z: "); Serial.println(offsetZ);
  Serial.print("Scale Factor Z: "); Serial.println(scaleFactorZ);
}

float collectZAxisAverage(const char* prompt) {
  Serial.println(prompt);
  while (!Serial.available());
  Serial.read(); // Clear serial buffer

  float sum = 0;
  const int numReadings = 100;

  for (int i = 0; i < numReadings; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sum += a.acceleration.z;
    delay(10);
  }

  return sum / numReadings;
}