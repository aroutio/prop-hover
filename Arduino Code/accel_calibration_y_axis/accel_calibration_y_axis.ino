#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float offsetY = 0;
float scaleFactorY = 0;

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

  Serial.println("Calibrating accelerometer for Y-axis...");
  calibrateYAxis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply offset and scale factor for Z-axis
  float ay = (a.acceleration.y - offsetY) * scaleFactorY * gravity;

  // Print calibrated Z-axis value
  Serial.print("Accel Y (m/s^2): ");
  Serial.println(ay);

  delay(100);
}

void calibrateYAxis() {
  float y_up = 0, y_down = 0;

  // Collect readings for Z-axis UP
  y_up = collectYAxisAverage("Align Y-axis UP (based on axis label) and press ENTER.");

  // Collect readings for Z-axis DOWN
  y_down = collectYAxisAverage("Align Y-axis (based on axis label) DOWN and press ENTER.");

  // Calculate offset
  offsetY = (y_up + y_down) / 2.0;

  // Calculate scale factor
  scaleFactorY = 2.0 / abs(y_up - y_down);

  Serial.println("Y-axis Calibration Complete!");
  Serial.print("Offset Y: "); Serial.println(offsetY);
  Serial.print("Scale Factor Y: "); Serial.println(scaleFactorY);
}

float collectYAxisAverage(const char* prompt) {
  Serial.println(prompt);
  while (!Serial.available());
  Serial.read(); // Clear serial buffer

  float sum = 0;
  const int numReadings = 100;

  for (int i = 0; i < numReadings; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sum += a.acceleration.y;
    delay(10);
  }

  return sum / numReadings;
}