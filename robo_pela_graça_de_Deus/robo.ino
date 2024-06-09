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
int lineFollowSensor0 = 11;
int lineFollowSensor1 = 39;
int lineFollowSensor2 = 12;
int lineFollowSensor3 = 37;
int lineFollowSensor4 = 13;

// Sensores de linha auxiliares
int CURVA_ESQUERDA = 10;
int CURVA_DIREITA = 14;

// Array para armazenar os valores dos sensores
int LFSensor[5] = {0, 0, 0, 0, 0};

// Definições dos pinos dos sensores ultrassônicos
const int TRIGGER_PIN = A0;
const int ECHO_PIN = A1;
int distancia_maxima = 10;
int distancia;

// Vars para PID
int Kp = 50;
int Ki = 0;
int Kd = 0;

float error=0, P=0, I=0, D=0, PIDvalue=0;
float previousError=0, previousI=0;

// Velocidades
int iniMotorPower = 160;
int leftMotorSpeed;
int rightMotorSpeed;
int adj = 1;

// Velocidades mínimas e máximas dos motores
int minMotorSpeed = 100;
int maxMotorSpeed = 180;

// Função para ler os valores dos sensores de linha
String lerSensores() {
    String sensorValues = "";
    LFSensor[0] = digitalRead(lineFollowSensor0);
    LFSensor[1] = digitalRead(lineFollowSensor1);
    LFSensor[2] = digitalRead(lineFollowSensor2);
    LFSensor[3] = digitalRead(lineFollowSensor3);
    LFSensor[4] = digitalRead(lineFollowSensor4);
    
    // Inverter os valores dos sensores 1 e 3
    if (LFSensor[1] == 1) {LFSensor[1] = 0;} else {LFSensor[1] = 1;}
    if (LFSensor[3] == 1) {LFSensor[3] = 0;} else {LFSensor[3] = 1;}

    for (int i = 0; i < 5; i++) {
      if (LFSensor[i] == 1){
        sensorValues += String(0);
      } else {
        sensorValues += String(1);
      }
    }

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
  int distance = duration * 0.034 / 2; // Converter a duração em centímetros
  return distance;
}

void verificarError(String sensor) {
    if (sensor == "00001") { error = 4; }
    else if (sensor == "00011") { error = 3; }
    else if (sensor == "00010") { error = 2; }
    else if (sensor == "00110") { error = 1; }
    else if (sensor == "00100") { error = 0; }
    else if (sensor == "01100") { error = -1; }
    else if (sensor == "01000") { error = -2; }
    else if (sensor == "11000") { error = -3; }
    else if (sensor == "10000") { error = -4; }
    else if (sensor == "11111" || sensor == "00000") { error = 0;}
    else { error = 0; } // Valor padrão para situações não cobertas

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

void motorTurn90(int direction){
    if (direction == ESQUERDA){
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENA, 140);
        analogWrite(ENB, 140);
    } else {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENA, 140);
        analogWrite(ENB, 140);
    }
}

void motorTurn(int direction){
    if (direction == ESQUERDA){
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

void calculatePID () {
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

void motorPIDcontrol () {
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
}

void logicFollowLINE() {
    // if (error == 4){ motorTurn(ESQUERDA); } 
    // else if (error == 3){ motorTurn(ESQUERDA); }
    // else if (error == 2){ motorTurn(ESQUERDA); }
    // else if (error == 1){ motorTurn(ESQUERDA); }
    // else if (error == 0){ moverFrente(); }
    // else if (error == -1){ motorTurn(DIREITA); }
    // else if (error == -2){ motorTurn(DIREITA); }
    // else if (error == -3){ motorTurn(DIREITA); }
    // else if (error == -4){ motorTurn(DIREITA); }

    if (error == 0) {
      moverFrente();
    } else if (error < 0){
      motorTurn(DIREITA);
    } else if (error > 0){
      motorTurn(ESQUERDA);
    } else {
      moverFrente();
    }
}

void logicDistance() {
  if(distancia <= distancia_maxima && distancia > 2){
    roboParar();
    delay(200);
      motorTurn(DIREITA);
      delay(1000);
      moverFrenteSemPID();
      delay(1000);
      motorTurn(ESQUERDA);
      delay(1000);
      moverFrenteSemPID();
      delay(1000);
      motorTurn(ESQUERDA);
      delay(1000);
  }
}

// Função principal que implementa a lógica dos sensores e movimentos
void logicMoreLines(String sensorValores) {
    int sens[2];
    sens[0] = digitalRead(CURVA_ESQUERDA);
    sens[1] = digitalRead(CURVA_DIREITA);
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
}

void loop() {
  distancia = ultrassonicRead();
  delay(100);
  while(true){
    distancia = ultrassonicRead();
    // Ler os valores dos sensores
    String sensorValues = lerSensores();
    verificarError(sensorValues);
    calculatePID();
    motorPIDcontrol();
    logicFollowLINE();
    logicDistance();
    logicMoreLines(sensorValues);
  }
}
