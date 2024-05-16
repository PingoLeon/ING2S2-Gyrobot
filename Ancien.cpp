#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

//? Objet MPU6050
Adafruit_MPU6050 mpu; 
sensors_event_t a, g, temp;

//* Objet servo
Servo servoRight;  
Servo servoLeft;  // Objet servo

//!Coefficients du PID
// 3 0.1 0.01
//3 0 0.1
float kp = 3;
float ki = 0;
float kd = 0.1; 
float value = 0;
float erreur = 0; 
float erreur_prec = 0; // Pour stocker l'erreur précédente
float somme_erreur = 0; // Pour stocker la somme cumulée des erreurs
unsigned long previousTime = 0; // Temps précédent

//! Variables pour le filtre de Kalman
float bias = 0;
float Q_angle = 0.1;
float Q_bias = 0.01;
float R_measure = 0.2;
float angle = 0;
float rate = 0;
float P[2][2] = {{0, 0}, {0, 0}};
float Q[2][2] = {
	{0.001, 0},
	{0, 0.003}
};

void setup() {
	Serial.begin(115200);
	while (!Serial) delay(10);

	if (!mpu.begin()) {
        Serial.println("Erreur du capteur MPU6050!");
        while (1) yield();
    }
	
	servoRight.attach(10);
	servoLeft.attach(11);
	
		
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);
	mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

	
}
void loop() {
	//*  Calcul du temps écoulé pour le PID (en secondes)
	unsigned long currentTime = millis();
	float dt = (currentTime - previousTime) / 1000.0; 
	previousTime = currentTime;
	
	//* Capter l'angle de roulis à partir de l'accéléromètre en Y et Z
	mpu.getEvent(&a, &g, &temp);
	float roll = (atan2(a.acceleration.y, a.acceleration.z) * 180 / PI); // l'angle varie de -90 à 90 degrés, 0 étant la position stable
	float gyro_rate = g.gyro.x * 180 / PI; // Taux de roulis en degrés par seconde
	float rate = gyro_rate - bias;
	bias += rate * dt;
	
	angle += dt * rate;
	
	//! Kalman filter
	// Prédictions
	angle += dt * rate;
	P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q[0][0]);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += Q[1][1] * dt;

	// Mise à jour de Kalman
	float S = P[0][0] + R_measure; // Estimation de l'erreur
	float K[2]; // Gain de Kalman
	K[0] = P[0][0] / S;
	K[1] = P[1][0] / S;
	float y = roll - angle; // Erreur de mesure
	angle += K[0] * y;
	bias += K[1] * y;
	float P00_temp = P[0][0];
	float P01_temp = P[0][1];

	// Mise à jour de l'estimation de l'erreur
	P[0][0] -= K[0] * P00_temp;
	P[0][1] -= K[0] * P01_temp;
	P[1][0] -= K[1] * P00_temp;
	P[1][1] -= K[1] * P01_temp;
	
	
	//! PID
	float desired_angle = 90; // Angle désiré
	erreur=desired_angle-angle; // Calcul de l'erreur
	
	somme_erreur += erreur * dt; // Ajoute l'erreur actuelle à la somme cumulée
	value = kp * erreur + ki * somme_erreur + kd * (erreur - erreur_prec) / dt;
	erreur_prec = erreur;
	
	//! Commande des servos
	servoRight.write(90+value);
	servoLeft.write(90-value);
}


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;


//Capter l'angle de tangeage à partir de l'accéléromètre en Y et Z
float AccelerometerData() {
	sensors_event_t a, g, temp;
	mpu.getEvent(&a, &g, &temp);
	float angle = (atan2(a.acceleration.y, a.acceleration.z) * 180 / PI) - 90; // l'angle varie de -90 à 90 degrés, 0 étant la position stable
	Serial.print("Angle: "); Serial.println(angle);
	return angle;
}// Fonction setup
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
}



// Fonction loop
void loop() {
	AccelerometerData();
}