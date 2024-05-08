#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 is ready!");
  delay(1000);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Serial.print("\n\n\n\n\n\n\n\n\n\n\n\n\n\nAcceleration - X=");
  Serial.print(a.acceleration.x);
  Serial.print(" / Y=");
  Serial.print(a.acceleration.y);
  Serial.print(" / Z=");
  Serial.print(a.acceleration.z);
  //Serial.print("\nGyro - X=");
  //Serial.print(g.gyro.x);
  //Serial.print(", Y=");
  //Serial.print(g.gyro.y);
  //Serial.print(", Z=");
  //Serial.println(g.gyro.z);
}