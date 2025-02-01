#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float offsetX = 0;
float scaleFactorX = 0;

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

  Serial.println("Calibrating accelerometer for X-axis...");
  calibrateXAxis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply offset and scale factor for Z-axis
  float ax = (a.acceleration.x - offsetX) * scaleFactorX * gravity;

  // Print calibrated Z-axis value
  Serial.print("Accel X (m/s^2): ");
  Serial.println(ax);

  delay(100);
}

void calibrateXAxis() {
  float x_up = 0, x_down = 0;

  // Collect readings for Z-axis UP
  x_up = collectXAxisAverage("Align X-axis UP (based on axis label) and press ENTER.");

  // Collect readings for Z-axis DOWN
  x_down = collectXAxisAverage("Align X-axis (based on axis label) DOWN and press ENTER.");

  // Calculate offset
  offsetX = (x_up + x_down) / 2.0;

  // Calculate scale factor
  scaleFactorX = 2.0 / abs(x_up - x_down);

  Serial.println("X-axis Calibration Complete!");
  Serial.print("Offset X: "); Serial.println(offsetX);
  Serial.print("Scale Factor X: "); Serial.println(scaleFactorX);
}

float collectXAxisAverage(const char* prompt) {
  Serial.println(prompt);
  while (!Serial.available());
  Serial.read(); // Clear serial buffer

  float sum = 0;
  const int numReadings = 100;

  for (int i = 0; i < numReadings; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sum += a.acceleration.x;
    delay(10);
  }

  return sum / numReadings;
}