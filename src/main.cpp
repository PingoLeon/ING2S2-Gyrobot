#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

Adafruit_MPU6050 mpu;
Servo servoRight, servoLeft;
//45 15 20 
// 15 35 25
// 100 30 10
float Kp = 15, Ki = 35, Kd = 25;
float integral = 0, previousError = 0;
unsigned long previousTime = 0;

double minCorrection = -90;
double maxCorrection = 90;

//Capter l'angle de tangeage à partir de l'accéléromètre en Y et Z
float AccelerometerData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float angle = (atan2(a.acceleration.y, a.acceleration.z) * 180 / PI) - 90; // l'angle varie de -90 à 90 degrés, 0 étant la position stable
  return angle;
}

float update(float setpoint, float actualValue) {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds

    float error = setpoint - actualValue;
    integral += error * deltaTime;
    float derivative = (error - previousError) / deltaTime;

    previousError = error;
    previousTime = currentTime;

    return Kp * error + Ki * integral + Kd * derivative;
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
  
  servoRight.attach(10); // Connect the rikght servo to pin 9
  servoLeft.attach(11); // Connect the left servo to pin 10
}

void loop() {
  float angle = AccelerometerData();
  if (angle < -90 || angle > 90) {
    Serial.println("Angle out of range");
    servoRight.write(90);
    servoLeft.write(90);
    while (angle <  -90 || angle > 90) {
      angle = AccelerometerData();
    }
  }
  float correction = update(0, angle); // Assume we want to maintain 0 degree
  
  if (correction > maxCorrection) {
    maxCorrection = correction;
  } else if (correction < minCorrection) {
    minCorrection = correction;
  }
  
  correction = map(correction, minCorrection, maxCorrection, -90, 90); // Map the correction to the servo range (-90 to 90
  
  
  Serial.println("\nAngle: " + String(angle) + " Correction: " + String(correction));
  servoRight.write(90 + correction);
  servoLeft.write(90 - correction);
  delay(100);
}