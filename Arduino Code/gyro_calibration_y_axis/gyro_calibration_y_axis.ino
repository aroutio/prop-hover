#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float gyroOffsetY = 0;

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

  Serial.println("Calibrating gyroscope (Y-axis only)...");
  calibrateGyroscopeY();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply offset to gyroscope Y-axis reading
  float gy_cal = g.gyro.y - gyroOffsetY;

  // Print calibrated gyroscope Y-axis value
  Serial.print("Gyro Y (rad/s): "); Serial.println(gy_cal);

  delay(100);
}

void calibrateGyroscopeY() {
  Serial.println("Keep the sensor stationary during calibration (Y-axis only), press ENTER.");
  while (!Serial.available());
  Serial.read(); // Clear serial buffer

  float sumY = 0;
  const int numReadings = 500;

  for (int i = 0; i < numReadings; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sumY += g.gyro.y;

    delay(5); // Small delay between readings
  }

  // Calculate offset for Y-axis
  gyroOffsetY = sumY / numReadings;

  Serial.println("Gyroscope Y-axis Calibration Complete!");
  Serial.print("Offset Y: "); Serial.println(gyroOffsetY);
}
