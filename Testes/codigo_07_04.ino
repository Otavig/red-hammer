#include <TCS230.h>

// Motores
#define motorENA 18
#define motorENB 16
#define in1 19
#define in2 17
#define in3 15
#define in4 14

// Defina os pinos de entrada e saída para o sensor TCS230 Esquerda
#define S0E A6
#define S1E A5
#define S2E A14
#define S3E A15
#define OUTE A13
// FAZER O MESMO PARA DIREITA E ESQUERDA MODIFICAR COM CALMA

// Defina os pinos de entrada e saída para o sensor TCS230 Direita
#define S0D A9
#define S1D A8
#define S2D A10
#define S3D A12
#define OUTD A11

// Sensores de refletância
#define s5_ 53
#define s4_ 52
#define s3_ 51
#define s2_ 50
#define s1_ 49

// Defina os limiares para a detecção da cor verde
#define GREEN_THRESHOLD_LOW 50
#define GREEN_THRESHOLD_HIGH 200
#define RED_THRESHOLD 100
#define BLUE_THRESHOLD 100

// Variáveis para armazenar os valores RGB lidos pelo sensor
int redValue, greenValue, blueValue;

// Função para ler o valor de cor específico do sensor
int readColor(int color) {
  digitalWrite(S2, color & 1); // Configura os bits S2 e S3 para selecionar o canal de cor
  digitalWrite(S3, (color >> 1) & 1);
  delay(10); // Aguarda um curto período para estabilizar a leitura
  return pulseIn(OUT, LOW); // Retorna o tempo de pulso (proporcional à intensidade da cor)
}

// Função para verificar se a cor detectada pelo sensor é verde
bool isGreenDetected() {
  // Lê os valores de cor do sensor
  redValue = readColor(RED);
  greenValue = readColor(GREEN);
  blueValue = readColor(BLUE);
  
  // Verifica se a cor detectada é verde
  if (greenValue > GREEN_THRESHOLD_LOW && greenValue < GREEN_THRESHOLD_HIGH && redValue < RED_THRESHOLD && blueValue < BLUE_THRESHOLD) {
    return true;
  } else {
    return false;
  }
}

// Função para verificar se a cor detectada pelo sensor é verde à direita
bool isRightSensorGreenDetected() {
  // Configura o sensor para ler a cor verde
  digitalWrite(S2D, LOW);
  digitalWrite(S3D, LOW);
  
  delay(10); // Aguarda um curto período para estabilizar a leitura
  
  // Retorna verdadeiro se a cor detectada for verde
  if (digitalRead(OUTD) == LOW) {
    return true;
  } else {
    return false;
  }
}

// Função para verificar se a cor detectada pelo sensor é verde à esquerda
bool isLeftSensorGreenDetected() {
  // Configura o sensor para ler a cor verde
  digitalWrite(S2E, HIGH);
  digitalWrite(S3E, HIGH);
  
  delay(10); // Aguarda um curto período para estabilizar a leitura
  
  // Retorna verdadeiro se a cor detectada for verde
  if (digitalRead(OUTE) == LOW) {
    return true;
  } else {
    return false;
  }
}

// Definições do controlador PID
#define KP 1.0  // Ganho proporcional
#define KI 0.0  // Ganho integral
#define KD 0.0  // Ganho derivativo

// Variáveis globais para o controlador PID
float lastError = 0;
float integral = 0;

// Função para realizar leituras dos sensores de refletância
void readSensors(int &s1_, int &s2_, int &s3_, int &s4_, int &s5_) {
  s1_ = digitalRead(s1);
  s2_ = digitalRead(s2);
  s3_ = digitalRead(s3);
  s4_ = digitalRead(s4);
  s5_ = digitalRead(s5);
}

// Funções para motores 

void frente(){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    analogWrite(motorENA, veloc);
    analogWrite(motorENB, veloc);
}

void direita(){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    analogWrite(motorENA, 200);
    analogWrite(motorENB, (veloc-80));
}

void esquerda(){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    analogWrite(motorENA, (veloc-80));
    analogWrite(motorENB, 200);
}

void eCurva(){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    analogWrite(motorENA, (veloc+75));
    analogWrite(motorENB, (veloc+75));
}

void dCurva(){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    analogWrite(motorENA, (veloc+75));
    analogWrite(motorENB, (veloc+75));
}

void parar(){
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

void tras() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    analogWrite(motorENA, 180);
    analogWrite(motorENB, 180);
}


// Funções de desvio de obstaculos
// void obstaculoFrontal(){

// }

// Função para controlar o movimento do robô usando PID
void autoTrack() {
  int s1_, s2_, s3_, s4_, s5_;
  readSensors(s1_, s2_, s3_, s4_, s5_);

  // Calcular o erro
  float error = s3_ - 1; // Se o sensor do meio detectar branco, o erro será negativo
  
  // Atualizar a última leitura de erro
  lastError = error;

  // Calcular o termo proporcional
  float proportional = KP * error;

  // Calcular o termo integral
  integral += error;
  float integralTerm = KI * integral;

  // Calcular o termo derivativo
  float derivative = error - lastError;
  float derivativeTerm = KD * derivative;

  // Calcular a velocidade dos motores
  float leftSpeed = 100 + proportional + integralTerm + derivativeTerm;
  float rightSpeed = 100 - proportional - integralTerm - derivativeTerm;

  // Movimentar o robô
  if (s3_ == BRANCO) {
    // Aplicar PID para manter o robô alinhado com a linha
    analogWrite(motorENA, leftSpeed);
    analogWrite(motorENB, rightSpeed);
  } else if (s2_ != BRANCO){
    esquerda();
  } else if(s4_ != BRANCO){
    direita();
  }
  else if (s1_ == BRANCO && s5_ != BRANCO) {
    // Giro de 90 graus à direita
    dCurva();
  } else if (s1_ != BRANCO && s5_ == BRANCO) {
    // Giro de 90 graus à esquerda
    eCurva();
  } else {
    // Se não for nenhum dos casos acima, seguir em frente
    frente();
  }
}

void setup() {
  // Inicialize os pinos do sensor Esquerdo
  pinMode(S0E, OUTPUT);
  pinMode(S1E, OUTPUT);
  pinMode(S2E, OUTPUT);
  pinMode(S3E, OUTPUT);
  pinMode(OUTE, INPUT);

  // Inicialize os pinos do sensor Direito
  pinMode(S0D, OUTPUT);
  pinMode(S1D, OUTPUT);
  pinMode(S2D, OUTPUT);
  pinMode(S3D, OUTPUT);
  pinMode(OUTD, INPUT);  
  
  // Configurando motores
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Configure o sensor para a taxa de frequência mais alta para melhor precisão
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);

  // Configurar pinos dos sensores de refletância
  pinMode(s1_, INPUT);
  pinMode(s2_, INPUT);
  pinMode(s3_, INPUT);
  pinMode(s4_, INPUT);
  pinMode(s5_, INPUT);
  
  Serial.begin(9600); // Inicie a comunicação serial para depuração
}

void loop() {
  // Chamando as funções para verificar a cor detectada pelos sensores
  bool rightGreen = isRightSensorGreenDetected();
  bool leftGreen = isLeftSensorGreenDetected();

  // Imprimindo o resultado na porta serial
  Serial.print("Sensor direito: ");
  Serial.println(rightGreen ? "Verde" : "Não verde");

  Serial.print("Sensor esquerdo: ");
  Serial.println(leftGreen ? "Verde" : "Não verde");

  // Aguarde um curto período antes da próxima leitura
  delay(500); 
  
  autoTrack(); // iniciar segui-linha com PID
}