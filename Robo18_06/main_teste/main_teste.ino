#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

// Definir os pinos dos motores
#define ENA 9
#define ENB 8
#define IN1 51
#define IN2 49
#define IN3 47
#define IN4 45

#define DIREITA 1
#define ESQUERDA -1

// Definir os pinos dos sensores de linha
int lineFollowSensor0 = 13;
int lineFollowSensor1 = 39;
int lineFollowSensor2 = 12;
int lineFollowSensor3 = 37;
int lineFollowSensor4 = 11;

// Sensores de linha auxiliares
int CURVA_ESQUERDA = 14;
int CURVA_DIREITA = 10;

// Array para armazenar os valores dos sensores
int LFSensor[5] = { 0, 0, 0, 0, 0 };

// Definições dos pinos dos sensores ultrassônicos
const int TRIGGER_PIN = A0;
const int ECHO_PIN = A1;
int distancia_maxima = 10;
int distancia;

// Vars para PID
int Kp = 50;
int Ki = 0;
int Kd = 0;

float error = 0, P = 0, I = 0, D = 0, PIDvalue = 0;
float previousError = 0, previousI = 0;

// Velocidades
int iniMotorPower = 160;
int leftMotorSpeed;
int rightMotorSpeed;
float adj = 1;

// Velocidades mínimas e máximas dos motores
int minMotorSpeed = 100;
int maxMotorSpeed = 160;
bool inclina = false;

// Função para ler os valores dos sensores de linha
String lerSensores() {
  String sensorValues = "";

  // Lê os valores dos sensores diretamente
  bool valorSensor0 = digitalRead(lineFollowSensor0);
  bool valorSensor1 = !digitalRead(lineFollowSensor1);  // Inverte o valor lido
  bool valorSensor2 = digitalRead(lineFollowSensor2);
  bool valorSensor3 = !digitalRead(lineFollowSensor3);  // Inverte o valor lido
  bool valorSensor4 = digitalRead(lineFollowSensor4);

  // Constrói a string de valores
  sensorValues += valorSensor0 ? "0" : "1";
  sensorValues += valorSensor1 ? "0" : "1";  // Valor já invertido
  sensorValues += valorSensor2 ? "0" : "1";
  sensorValues += valorSensor3 ? "0" : "1";  // Valor já invertido
  sensorValues += valorSensor4 ? "0" : "1";

  return sensorValues;
}

// Função para ler valores de distancia
int ultrassonicRead() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;  // Converter a duração em centímetros
  return distance;
}

void verificarError(String sensor) {
  if (sensor == "00001") {
    error = 4;
  } else if (sensor == "00011") {
    error = 3;
  } else if (sensor == "00010") {
    error = 2;
  } else if (sensor == "00110") {
    error = 1;
  } else if (sensor == "00100") {
    error = 0;
  } else if (sensor == "01100") {
    error = -1;
  } else if (sensor == "01000") {
    error = -2;
  } else if (sensor == "11000") {
    error = -3;
  } else if (sensor == "10000") {
    error = -4;
  } else if (sensor == "11111" || sensor == "00000") {
    error = 0;
  } else {
    error = 0;
  }  // Valor padrão para situações não cobertas

  return;
}

// Movimentos
void moverFrente() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, +leftMotorSpeed);
  analogWrite(ENB, +rightMotorSpeed);
}

void moverFrenteSemPID() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, iniMotorPower);
  analogWrite(ENB, iniMotorPower);
}

void moverBackSemPID() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, iniMotorPower);
  analogWrite(ENB, iniMotorPower);
}

void roboParar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void moverTras() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, leftMotorSpeed);
  analogWrite(ENB, rightMotorSpeed);
}

void motorTurn90(int direction) {
  if (direction == ESQUERDA) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 160);
    analogWrite(ENB, 160);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 160);
    analogWrite(ENB, 160);
  }
}

void motorTurn(int direction) {
  if (direction == ESQUERDA) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    int adjustedLeftSpeed = leftMotorSpeed - 30;
    if (adjustedLeftSpeed < minMotorSpeed) {
      adjustedLeftSpeed = minMotorSpeed;
    }

    analogWrite(ENA, adjustedLeftSpeed);
    analogWrite(ENB, rightMotorSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    int adjustedRightSpeed = rightMotorSpeed - 30;
    if (adjustedRightSpeed < minMotorSpeed) {
      adjustedRightSpeed = minMotorSpeed;
    }

    analogWrite(ENA, leftMotorSpeed);
    analogWrite(ENB, adjustedRightSpeed);
  }
}

void calculatePID() {
  P = error;
  I = I + error;
  D = error - previousError;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;
}

void checkPIDvalues() {
  Serial.print("PID: ");
  Serial.print(Kp);
  Serial.print(" - ");
  Serial.print(Ki);
  Serial.print(" - ");
  Serial.println(Kd);
}

void motorPIDcontrol() {
  if (inclina != true) {
    leftMotorSpeed = iniMotorPower - PIDvalue;
    rightMotorSpeed = iniMotorPower + PIDvalue;

    // Limitar a velocidade mínima e máxima
    if (leftMotorSpeed < minMotorSpeed) {
      leftMotorSpeed = minMotorSpeed;
    } else if (leftMotorSpeed > maxMotorSpeed) {
      leftMotorSpeed = maxMotorSpeed;
    }

    if (rightMotorSpeed < minMotorSpeed) {
      rightMotorSpeed = minMotorSpeed;
    } else if (rightMotorSpeed > maxMotorSpeed) {
      rightMotorSpeed = maxMotorSpeed;
    }
  } else {
    leftMotorSpeed = 255;
    rightMotorSpeed = 255;
  }
}

void logicFollowLINE() {
  if (error == 0) {
    moverFrente();
  } else if (error < 0) {
    motorTurn(DIREITA);
  } else if (error > 0) {
    motorTurn(ESQUERDA);
  } else {
    moverFrente();
  }
}

void desvio(int direction) {

  if (direction == ESQUERDA) {
    roboParar();
    delay(200);
    moverBackSemPID();
    delay(300);
    motorTurn(DIREITA);  // ESQUERDA
    delay(1300);
    moverFrenteSemPID();
    delay(800);
    motorTurn(ESQUERDA);  // DIREITA
    delay(1500);
    moverFrenteSemPID();
    delay(2300);
    motorTurn(ESQUERDA);  // DIREITA
    delay(1500);
    moverFrenteSemPID();
    delay(800);
    motorTurn(DIREITA);  // ESQUERDA
    delay(1500);
    moverBackSemPID();
    delay(300);
  } else {
    roboParar();
    delay(200);
    moverBackSemPID();
    delay(300);
    motorTurn(ESQUERDA);  // DIREITA
    delay(1300);
    moverFrenteSemPID();
    delay(800);
    motorTurn(DIREITA);  // ESQUERDA
    delay(1500);
    moverFrenteSemPID();
    delay(2300);
    motorTurn(DIREITA);  // ESQUERDA
    delay(1500);
    moverFrenteSemPID();
    delay(800);
    motorTurn(ESQUERDA);  // DIREITA
    delay(1500);
    moverBackSemPID();
    delay(300);
  }
}

void rotinaCor() {
}

void rotinaCorDuplo() {
}

bool rotinaMpu() {
  mpu6050.update();
  Serial.print("angleX : ");
  Serial.print(mpu6050.getAngleX());
  Serial.print("\tangleY : ");
  Serial.print(mpu6050.getAngleY());
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());

  if (anguloX >= 25) {
    inclina = true;
  } else {
    inclina = false;
  }
}

String pegar_valor_sensores(int s1, int s2, int s3, int s4, int s5) {
  return String(!digitalRead(s1)) + String(!digitalRead(s2)) + String(!digitalRead(s3)) + String(!digitalRead(s4)) + String(!digitalRead(s5));
}

void centralizarRobo() {
  roboParar();
  delay(300);

  String valueSensors = pegar_valor_sensores(CURVA_ESQUERDA, lineFollowSensor0, lineFollowSensor2, lineFollowSensor4, CURVA_DIREITA);

  if (valueSensors == "10000" or valueSensors == "11000" or valueSensors == "11100") {
    while (digitalRead(lineFollowSensor2) == 0) {
      motorTurn90(ESQUERDA);
    }
  } else if (valueSensors == "00001" or valueSensors == "00011" or valueSensors == "00111") {
    while (digitalRead(lineFollowSensor2) == 0) {
      motorTurn90(DIREITA);
    }

  } else {
    return;
  }
}

void ajusteAhead() {
  roboParar();
  delay(100);
  // 01000
  String valueSensorsFrente = pegar_valor_sensores(CURVA_ESQUERDA, lineFollowSensor0, lineFollowSensor2, lineFollowSensor4, CURVA_DIREITA);
  String valueSensorsFrenteVerso = pegar_valor_sensores(lineFollowSensor0, lineFollowSensor1, lineFollowSensor2, lineFollowSensor3, lineFollowSensor4);

  if (valueSensorsFrente != "00000") {
    return;
  } else {
    if (valueSensorsFrenteVerso == "01000") {
      while (digitalRead(lineFollowSensor2) == 0) {
        motorTurn90(DIREITA);
      }

    } else if (valueSensorsFrenteVerso == "00010") {
      while (digitalRead(lineFollowSensor2) == 0) {
        motorTurn90(ESQUERDA);
      }
    }
  }
}

void logicMoreLines() {
  String valueSensorsNew;
  String valueSensors = pegar_valor_sensores(CURVA_ESQUERDA, lineFollowSensor0, lineFollowSensor2, lineFollowSensor4, CURVA_DIREITA);


  if (valueSensors == "11111" or valueSensors == "10001") {
    moverFrenteSemPID();
    delay(500);
    ajusteAhead();

    valueSensorsNew = pegar_valor_sensores(CURVA_ESQUERDA, lineFollowSensor0, lineFollowSensor2, lineFollowSensor4, CURVA_DIREITA);

    if (valueSensorsNew != "00000") {
      return;
    } else {
      moverBackSemPID();
      delay(500);
    }
  }

  if (valueSensors == "00001" or valueSensors == "00011" or valueSensors == "00111") {
    moverFrenteSemPID();
    delay(350);
    ajusteAhead();

    valueSensorsNew = pegar_valor_sensores(CURVA_ESQUERDA, lineFollowSensor0, lineFollowSensor2, lineFollowSensor4, CURVA_DIREITA);
    //   Serial.println(valueSensorsNew);

    if (valueSensorsNew != "00000") {
      return;
    } else {
      moverFrenteSemPID();
      delay(300);
      motorTurn90(DIREITA);
      delay(1000);

      while (digitalRead(lineFollowSensor2) != 0) {
        motorTurn90(DIREITA);
      }
    }
  }

  if (valueSensors == "10000" or valueSensors == "11000" or valueSensors == "11100") {
    moverFrenteSemPID();
    delay(350);
    ajusteAhead();

    valueSensorsNew = pegar_valor_sensores(CURVA_ESQUERDA, lineFollowSensor0, lineFollowSensor2, lineFollowSensor4, CURVA_DIREITA);

    if (valueSensorsNew != "00000") {
      return;
    } else {
      moverFrenteSemPID();
      delay(300);
      motorTurn90(ESQUERDA);
      delay(1000);

      while (digitalRead(lineFollowSensor2) != 0) {
        motorTurn90(ESQUERDA);
      }
    }
  }
}

void logicDistance() {
  if (distancia <= distancia_maxima && distancia > 2) {
    centralizarRobo();
    desvio_quadratico(ESQUERDA);
  }
  return;
}

void setup() {
  // Inicializar os pinos dos motores como saída
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Inicializar os pinos dos sensores como entrada
  pinMode(lineFollowSensor0, INPUT);
  pinMode(lineFollowSensor1, INPUT);
  pinMode(lineFollowSensor2, INPUT);
  pinMode(lineFollowSensor3, INPUT);
  pinMode(lineFollowSensor4, INPUT);

  pinMode(CURVA_ESQUERDA, INPUT);
  pinMode(CURVA_DIREITA, INPUT);


  // Inicializar os pinos dos  sensores de distancia de obstaculo
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);


  // Iniciar a comunicação serial para depuração
  Serial.begin(9600);

  // Mpu
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  mpu6050.update();
  distancia = ultrassonicRead();
  delay(5000);
  while (true) {
    distancia = ultrassonicRead();
    // Ler os valores dos sensores
    String sensorValues = lerSensores();
    verificarError(sensorValues);
    // Serial.println(sensorValues);
    calculatePID();
    motorPIDcontrol();
    logicMoreLines();
    logicFollowLINE();
    logicDistance();
    rotinaMpu();
  }
}
