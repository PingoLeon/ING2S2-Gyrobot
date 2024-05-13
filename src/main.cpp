#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

Adafruit_MPU6050 mpu;

//Utiliser le PID de Z lorsque l'on voudra avancer
// Position stable : X = ? Y = 10 Z = 0
// Inclinaison en avant : X = ? Y = 0 Z = -10
// Inclinaison en arrière : X = ? Y = 0 Z = 10
//Position renversée : X = ? Y = -10 Z = 0

const int numSamples = 200; // Number of accelerometer readings to print
const int servoPinLeft = 10; // Servo pin for left wheel
const int servoPinRight = 11; // Servo pin for right wheel

Servo servoLeft;
Servo servoRight;

double Y_Setpoint = 10, Z_Setpoint = 0.0; // Valeurs cibles
double Y_Input, Z_Input; // Valeurs actuelles
double Y_Output, Z_Output; // Commandes de sortie

//pmax = 20
//p = 12
//imax = 7


double Y_Kp = 20, Y_Ki = 12, Y_Kd = 0; // Coefficients PID
double Z_Kp = 20, Z_Ki = 12, Z_Kd = 0; // Coefficients PID


PID Y_PID(&Y_Input, &Y_Output, &Y_Setpoint, Y_Kp, Y_Ki, Y_Kd, DIRECT);
PID Z_PID(&Z_Input, &Z_Output, &Z_Setpoint, Z_Kp, Z_Ki, Z_Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10); // Wait for serial connection

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Attach servos
  servoLeft.attach(servoPinLeft);
  servoRight.attach(servoPinRight);

  // Initialize PID
  Y_PID.SetMode(AUTOMATIC);
  Z_PID.SetMode(AUTOMATIC);
}

void loop() {
  // Read accelerometer data
  sensors_event_t accel, gyro, temp;
  
  mpu.getEvent(&accel, &gyro, &temp);
  Y_Input = accel.acceleration.y;
  Z_Input = accel.acceleration.z;
  
  if (Y_Input < 0){
    //La position d'équilibre n'est plus du tout maintenue
    //On arrête le robot
    servoLeft.write(90);
    servoRight.write(90);
    delay(10);
    Serial.println("Le robot est renversé, arrêt du programme");
    while(Y_Input < 0){
      mpu.getEvent(&accel, &gyro, &temp);
      Y_Input = accel.acceleration.y;
      Z_Input = accel.acceleration.z;
    }
  }

  // Compute PID output
  Y_PID.Compute();
  Z_PID.Compute();

  // Set servo positions based on PID output
  int correction = Y_Output - Z_Output;
  int servoLeftPos = 90 + correction;
  int servoRightPos = 90 - correction;
  servoLeft.write(servoLeftPos);
  servoRight.write(servoRightPos);

  // Print PID output for debugging
  Serial.print("Y_Output: ");
  Serial.print(Y_Output);
  Serial.print(", Z_Output: ");
  Serial.println(Z_Output);
  
}