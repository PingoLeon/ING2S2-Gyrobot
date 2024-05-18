#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

Servo servoDroit;
Servo servoGauche;

// Paramètres PID
float Kp = 3;  // Coefficient proportionnel
float Ki = 0;  // Coefficient intégral
float Kd = 0.12;  // Coefficient dérivatif

// Variables pour le PID
float previous_error = 0;
float integral = 0;
unsigned long lastTime;

// Variables pour le filtre de Kalman
float Q_angle = 0.01; // Process noise variance for the accelerometer
float Q_gyro = 0.01; // Process noise variance for the gyroscope
float R_angle = 0.3; // Measurement noise variance
float angle = 0; // The angle calculated by the Kalman filter
float bias = 0; // The gyro bias calculated by the Kalman filter
float P[2][2] = {{0, 0}, {0, 0}}; // Error covariance matrix

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    if (!mpu.begin()) {
        Serial.println("Erreur du capteur MPU6050!");
        while (1) yield();
    }

    servoDroit.attach(10); // Connecter le premier servo à la broche D10
    servoGauche.attach(11); // Connecter le second servo à la broche D11

    // Configuration de MPU6050
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    lastTime = millis();
}

void loop() {
    
    mpu.getEvent(&a, &g, &temp);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // Lecture des données du capteur
    float accel_angle = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
    float gyro_rate = g.gyro.x * 180 / PI;

    // Application du filtre de Kalman
    float rate = gyro_rate  - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_gyro * dt;

    float S = P[0][0] + R_angle;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = accel_angle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    float desiredAngle = 94.7; // Angle vertical, 95 stabilité sur notre robot
    float error = desiredAngle - angle;

    // Calcul du PID
    integral += error;
    float derivative = (error - previous_error);
    float output = Kp * error + Ki * integral + Kd * derivative;

	//88.5
    servoDroit.write(89 + output);
    servoGauche.write(90 - output);
    
    Serial.print(">Angle de Tangeage:");
    Serial.println(angle);

    previous_error = error;
}