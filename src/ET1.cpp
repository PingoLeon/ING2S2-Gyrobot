#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int numSamples = 200;

float mapToAngle(float zValue) {
  zValue = constrain(zValue, -9.8, 9.8);
  
  float mappedAngle = asin(zValue / 9.8) * 180.0 / PI;
  
  float scaledAngle = mappedAngle * (90.0 / 54.72);
  
  float finalAngle = map(scaledAngle, -148, 111, -90, 90);
  
  return finalAngle;
}





void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10); // Wait for serial connection

  Serial.println("Collecting accelerometer data...");
  delay(100);


  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  printAccelerometerData(numSamples);
}

void loop() {
}

void printAccelerometerData(int numSamples) {
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    float angle = mapToAngle(a.acceleration.z);

    Serial.print("Acceleration (m/s^2): ");
    Serial.print("Z: "); Serial.print(a.acceleration.z);
    Serial.print(", Angle: "); Serial.println(angle);

    delay(100);
  }
}