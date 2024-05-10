#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

Adafruit_MPU6050 mpu;

const int numSamples = 200; // Number of accelerometer readings to print
const int servoPinLeft = 10; // Servo pin for left wheel
const int servoPinRight = 11; // Servo pin for right wheel

Servo servoLeft;
Servo servoRight;

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,1,1,1, DIRECT);

// Function to map Z-axis values to angles
float mapToAngle(float zValue) {
  // Assuming the sensor is perfectly vertical when zValue is 9.81 (gravity)
  // The angle is then the arcsin of the zValue divided by gravity
  float angle = asin(zValue / 9.81) * (180.0 / PI); // Convert to degrees
  return angle;
}

// Function to measure position Y and return deltaY
float measurePositionY() {
  // Assuming the sensor is stationary at the start
  // The change in Y position (deltaY) is then the integral of the Y acceleration over time
  // For simplicity, we can just return the Y acceleration as a proxy for deltaY
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return a.acceleration.y;
}

// Function to control the servos based on stabilization algorithm
void controlServos(float angle) {
  // Assuming the servos are centered at 90 degrees
  // If the angle is positive, the robot is tilting backwards
  // If the angle is negative, the robot is tilting forwards
  // We can then adjust the servos to counteract the tilt
  if (angle > 0) {
    servoLeft.write(90 + angle);
    servoRight.write(90 - angle);
  } else {
    servoLeft.write(90 - angle);
    servoRight.write(90 + angle);
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10); // Wait for serial connection

  Serial.println("Collecting accelerometer data...");
  delay(100);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Attach servos
  servoLeft.attach(servoPinLeft);
  servoRight.attach(servoPinRight);

  // Initialize PID
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Use complementary filter to get angle
  float accelAngle = mapToAngle(a.acceleration.z);
  float gyroRate = g.gyro.z;
  float dt = 0.01; // Time elapsed since last measurement, in seconds
  float angle = 0.98 * (angle + gyroRate * dt) + 0.02 * accelAngle;

  // Use PID controller to get output
  Input = angle;
  myPID.Compute();

  // Control servos based on PID output
  controlServos(Output);

  // Print accelerometer data with mapped angles
  Serial.print("Acceleration (m/s^2): ");
  Serial.print("Z: "); Serial.print(a.acceleration.z);
  Serial.print(", Angle: "); Serial.println(angle);

  // Wait between each measurement
  delay(100);
}