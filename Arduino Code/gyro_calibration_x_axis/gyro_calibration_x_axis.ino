#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float gyroOffsetX = 0;

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

  Serial.println("Calibrating gyroscope (X-axis only)...");
  calibrateGyroscopeX();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply offset to gyroscope X-axis reading
  float gx_cal = g.gyro.x - gyroOffsetX;

  // Print calibrated gyroscope X-axis value
  Serial.print("Gyro X (rad/s): "); Serial.println(gx_cal);

  delay(100);
}

void calibrateGyroscopeX() {
  Serial.println("Keep the sensor stationary during calibration (X-axis only), press ENTER.");
  while (!Serial.available());
  Serial.read(); // Clear serial buffer

  float sumX = 0;
  const int numReadings = 500;

  for (int i = 0; i < numReadings; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sumX += g.gyro.x;

    delay(5); // Small delay between readings
  }

  // Calculate offset for X-axis
  gyroOffsetX = sumX / numReadings;

  Serial.println("Gyroscope X-axis Calibration Complete!");
  Serial.print("Offset X: "); Serial.println(gyroOffsetX);
}
