#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float gyroOffsetZ = 0;

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

  Serial.println("Calibrating gyroscope (Z-axis only)...");
  calibrateGyroscopeZ();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply offset to gyroscope Z-axis reading
  float gz_cal = g.gyro.z - gyroOffsetZ;

  // Print calibrated gyroscope Z-axis value
  Serial.print("Gyro Z (rad/s): "); Serial.println(gz_cal);

  delay(100);
}

void calibrateGyroscopeZ() {
  Serial.println("Keep the sensor stationary during calibration (Z-axis only), press ENTER.");
  while (!Serial.available());
  Serial.read(); // Clear serial buffer

  float sumZ = 0;
  const int numReadings = 500;

  for (int i = 0; i < numReadings; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sumZ += g.gyro.z;

    delay(5); // Small delay between readings
  }

  // Calculate offset for Z-axis
  gyroOffsetZ = sumZ / numReadings;

  Serial.println("Gyroscope Z-axis Calibration Complete!");
  Serial.print("Offset Z: "); Serial.println(gyroOffsetZ);
}
