#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

Adafruit_MPU6050 mpu;
Servo servoRight, servoLeft;

//Capter l'angle de tangeage à partir de l'accéléromètre en Y et Z
float AccelerometerData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float angle = (atan2(a.acceleration.y, a.acceleration.z) * 180 / PI) - 90; // l'angle varie de -90 à 90 degrés, 0 étant la position stable
  Serial.print("Angle: "); Serial.println(angle);
  return angle;
}

// Fonction setup
void setup() {
  Serial.begin(9600);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  
  servoRight.attach(10);
  servoLeft.attach(11);
}

// Fonction loop
void loop() {
  AccelerometerData();
}