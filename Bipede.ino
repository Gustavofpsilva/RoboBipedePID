#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;

Servo servoHipRight;
Servo servoHipLeft;
Servo servoKneeRight;
Servo servoKneeLeft;

const int hipRightPin = 2;
const int hipLeftPin = 3;
const int kneeRightPin = 4;
const int kneeLeftPin = 5;

// Parâmetros PID
double setpoint = 0; // Ângulo desejado
double input, output;
double Kp = 10; // Ganho proporcional
double Ki = 0.1; // Ganho integral
double Kd = 1; // Ganho derivativo

double elapsedTime, previousTime;
double error, lastError;
double cumError, rateError;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  servoHipRight.attach(hipRightPin);
  servoHipLeft.attach(hipLeftPin);
  servoKneeRight.attach(kneeRightPin);
  servoKneeLeft.attach(kneeLeftPin);

  servoHipRight.write(90);
  servoHipLeft.write(90);
  servoKneeRight.write(90);
  servoKneeLeft.write(90);

  previousTime = millis();
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calcula o ângulo de inclinação em relação ao eixo X (pitch)
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * (180.0 / M_PI);

  // Define o ângulo desejado
  double setpoint = 0;

  // Calcula o erro
  double error = setpoint - pitch;

  // Calcula o tempo decorrido
  double currentTime = millis();
  double elapsedTime = (currentTime - previousTime) / 1000.0;

  // Calcula as componentes PID
  double P = Kp * error;
  cumError += error * elapsedTime;
  double I = Ki * cumError;
  rateError = (error - lastError) / elapsedTime;
  double D = Kd * rateError;

  // Calcula a saída PID
  double output = P + I + D;

  // Mapeia a saída PID para a faixa de movimento dos servos
  int servoAngle = map(output, -90, 90, 45, 135);

  // Limita a faixa de movimento dos servos
  servoAngle = constrain(servoAngle, 45, 135);

  // Atualiza a posição dos servos
  servoHipRight.write(servoAngle);
  servoHipLeft.write(servoAngle);
  servoKneeRight.write(servoAngle);
  servoKneeLeft.write(servoAngle);

  // Atualiza variáveis para a próxima iteração
  lastError = error;
  previousTime = currentTime;

  delay(20); // Pequeno atraso para estabilidade
}
