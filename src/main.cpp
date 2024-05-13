#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

Adafruit_MPU6050 mpu;
Servo servoRight, servoLeft;


double Setpoint = 0, Input, Output;
double  Kp=45, 
        Ki=14, 
        Kd=20;
        
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


//Capter l'angle de tangeage à partir de l'accéléromètre en Y et Z
float AccelerometerData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float angle = (atan2(a.acceleration.y, a.acceleration.z) * 180 / PI) - 90; // l'angle varie de -90 à 90 degrés, 0 étant la position stable
  return angle;
}

// Fonction setup
void setup() {
  Serial.begin(9600); //Serial 
  if (!mpu.begin()) { //MPU6050 Initialization
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  
  servoRight.attach(10);  //servoRight Initialization (Droite)
  servoLeft.attach(11); //servoLeft Initialization (Gauche)
  
  myPID.SetMode(AUTOMATIC); //PID Initialization
}

// Fonction loop
void loop() {
  Input = AccelerometerData();
  while (Input <  -90 || Input > 90) {
    servoRight.write(90);
    servoLeft.write(90);
    Serial.println("Angle out of range");
    
    Input = AccelerometerData();
  }
  myPID.Compute();
  
  Serial.println("Angle: " + String(Input) + " / PID Output: " + String(Output));
  
  double correction = Output;
  int servoRightPos = 90 + correction;
  int servoLeftPos = 90 - correction;
  servoRight.write(servoRightPos);
  servoLeft.write(servoLeftPos);
}