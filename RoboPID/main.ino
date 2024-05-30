int mode = 0;

// Definir os pinos dos motores
#define ENA 9
#define ENB 8
#define IN1 51
#define IN2 49
#define IN3 47
#define IN4 45

#define DIREITA 1
#define ESQUERDA -1

# define STOPPED 0
# define FOLLOWING_LINE 1
# define NO_LINE 2

// Definir os pinos dos sensores de linha
int lineFollowSensor0 = 11;
int lineFollowSensor1 = 39;
int lineFollowSensor2 = 12;
int lineFollowSensor3 = 37;
int lineFollowSensor4 = 13;

// Array para armazenar os valores dos sensores
int LFSensor[5] = {0, 0, 0, 0, 0};

// Vars para PID
int Kp = 50;
int Ki = 0;
int Kd = 0;

float error=0, P=0, I=0, D=0, PIDvalue=0;
float previousError=0, previousI=0;

// Velocidades
int iniMotorPower = 180;
int leftMotorSpeed;
int rightMotorSpeed;
int adj = 1;

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

void verificarError(String sensor) {
    if (sensor == "00001") { erro = 4; }
    else if (sensor == "00011") { mode = FOLLOWING_LINE; error = 3; }
    else if (sensor == "00010") { mode = FOLLOWING_LINE; error = 2; }
    else if (sensor == "00110") { mode = FOLLOWING_LINE; error = 1; }
    else if (sensor == "00100") { mode = FOLLOWING_LINE; error = 0; }
    else if (sensor == "01100") { mode = FOLLOWING_LINE; error = -1; }
    else if (sensor == "01000") { mode = FOLLOWING_LINE; error = -2; }
    else if (sensor == "11000") { mode = FOLLOWING_LINE; error = -3; }
    else if (sensor == "10000") { mode = FOLLOWING_LINE; error = -4; }
    else if (sensor == "11111" || sensor == "00000") { mode = NO_LINE; error = 0;}
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

void moverTras() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, -leftMotorSpeed);
    analogWrite(ENB, -ightMotorSpeed);
}

void motorTurn(int direction){
    if (direction == ESQUERDA){
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);

        analogWrite(ENA, leftMotorSpeed - 30);
        analogWrite(ENB, rightMotorSpeed);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        analogWrite(ENA, leftMotorSpeed);
        analogWrite(ENB, rightMotorSpeed - 30);
    }
}

void calculatePID ()
{
   P = error;
   I = I + error;
   D = error - previousError;
   PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
   previousError = error;
}

void checkPIDvalues()
{
  Serial.print("PID: ");
  Serial.print(Kp);
  Serial.print(" - ");
  Serial.print(Ki);
  Serial.print(" - ");
  Serial.println(Kd);  
  
}

void motorPIDcontrol ()
{
    leftMotorSpeed = iniMotorPower - PIDvalue;
    rightMotorSpeed = iniMotorPower - PIDvalue;
}

void logicFollowLINE() {
    if (error == 4){ motorTurn(ESQUERDA); } 
    else if (error == 3){ motorTurn(ESQUERDA); }
    else if (error == 2){ motorTurn(ESQUERDA); }
    else if (error == 1){ motorTurn(ESQUERDA); }
    else if (error == 0){ moverFrente(); }
    else if (error == -1){ motorTurn(DIREITA); }
    else if (error == -2){ motorTurn(DIREITA); }
    else if (error == -3){ motorTurn(DIREITA); }
    else if (error == -4){ motorTurn(DIREITA); }
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

    // Iniciar a comunicação serial para depuração
    Serial.begin(9600);
}


void loop() {
    switch (mode){
    case FOLLOWING_LINE:
    // Ler os valores dos sensores
        String sensorValues = lerSensores();
        verificarErro(sensorValues);
        calculatePID();
        motorPIDcontrol();
        logicFollowLINE();
        break;
    
    default:
        break;
    }
}