#include "Ultrasonic.h" //INCLUSÃO DA BIBLIOTECA NECESSÁRIA PARA FUNCIONAMENTO DO CÓDIGO

// // echos
// const int eFrente = 7; // Recebe 

// // triggers
// const int tFrente = 6; // Envia

// Ultrasonic ultrasonic(tFrente,eFrente); //INICIALIZANDO OS PINOS DO ARDUINO

// Sensores de cores
int verde = 0;
int verde2 = 0;

unsigned long ultimaDetecaoVerde = 0;
unsigned long intervaloDetecao = 2000; 

// Motores
#define motorENA 18
#define motorENB 16
#define in1 19
#define in2 17
#define in3 15
#define in4 14

// Sensores de reflatancia
#define mED A12 // meio extrema direita
#define mEE A15 // meio extrema esquerda
#define mE A14 // meio esquerda
#define mD  A11 // meio direita
#define meio A13 // meio

#define BRANCO 1

// Especificações 
#define velocidade 70

#define velocVirar 120

void setup(){
    Serial.begin(9600);
    
    // Configurando motores
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    // // Iniciar motores como 0
    // digitalWrite(in1, LOW);
    // digitalWrite(in2, LOW);
    // digitalWrite(in3, LOW);
    // digitalWrite(in4, LOW);

    // configurarPinosSensor(PINO_SENSOR_S0, PINO_SENSOR_S1, PINO_SENSOR_S2, PINO_SENSOR_S3, PINO_SENSOR_OUT);
    // configurarPinosSensor(PINO_SENSOR_S0_2, PINO_SENSOR_S1_2, PINO_SENSOR_S2_2, PINO_SENSOR_S3_2, PINO_SENSOR_OUT_2);
}   

// void hcsr04(int trigPin){
//   digitalWrite(trigPin, LOW);
//   delayMicroseconds(2);
//   digitalWrite(trigPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPin, LOW);
//   distancia = (ultrasonic.Ranging(CM)); //VARIÁVEL GLOBAL RECEBE O VALOR DA DISTÂNCIA MEDIDA
//   result = String(distancia);
//   delay(500);
// }

// void desvioFrontal(){
//   String resultado = hcsr04(tFrente);
//   if(resultado == 8){
//     tras();
//     delay(300);
//     curva90Direita();
//     delay(690);
//   }
//   return
// }

void frente(){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    analogWrite(motorENA, velocidade);
    analogWrite(motorENB, velocidade);
}

void direita(){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    analogWrite(motorENA, 0);
    analogWrite(motorENB, velocVirar);
}

void esquerda(){
     digitalWrite(in1, HIGH);
     digitalWrite(in2, LOW);
     digitalWrite(in3, LOW);
     digitalWrite(in4, HIGH);

     analogWrite(motorENA, velocVirar);
     analogWrite(motorENB, 0);
}

void curva90Direita(){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    analogWrite(motorENA, velocVirar);
    analogWrite(motorENB, velocVirar);
}

void curva90Esquerda(){
     digitalWrite(in1, HIGH);
     digitalWrite(in2, LOW);
     digitalWrite(in3, LOW);
     digitalWrite(in4, HIGH);

     analogWrite(motorENA, velocVirar);
     analogWrite(motorENB, velocVirar);
}

void tras(){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    analogWrite(motorENA, velocidade);
    analogWrite(motorENB, velocidade);
}

void pararRobo(){
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);

    analogWrite(motorENA, 0);
    analogWrite(motorENB, 0);
}

void autoTrack(){
  // Sensores
  int valorMeio = analogRead(meio);
  int valorMeioE = analogRead(mE);
  int valorMeioD = analogRead(mD);

  int valorExtremaE = analogRead(mEE); 
  int valorExtremaD = analogRead(mED);


  while(valorMeio != BRANCO){
    frente();
    break;
  }

  while(valorMeioE != BRANCO){
    esquerda();
    break;
  }

  while(valorMeioD != BRANCO){
    direita();
    break;
  }

  while (valorMeio == BRANCO && valorMeioE == BRANCO && valorMeioD == BRANCO){
    frente();
    break;
  }

  while (valorMeio != BRANCO && valorMeioE != BRANCO && valorMeioD != BRANCO){
    frente();
    break;
  }

  // || verde2 > 0 && tempoAtual - ultimaDetecaoVerde >= intervaloDetecao
  if (valorExtremaE != BRANCO){ 
    pararRobo();
    delay(300);
    frente();
    delay(100);
    curva90Esquerda();
    delay(690);
    frente();
    delay(100);
    pararRobo();
  }

  // || verde > 0 && tempoAtual - ultimaDetecaoVerde >= intervaloDetecao
  if (valorExtremaD != BRANCO){
    pararRobo();
    delay(300);
    frente();
    delay(100);
    curva90Direita();
    delay(690);
    frente();
    delay(100);
    pararRobo();
  }
}

void configurarPinosSensor(int pinS0, int pinS1, int pinS2, int pinS3, int pinOUT) {
  pinMode(pinS0, OUTPUT);
  pinMode(pinS1, OUTPUT);
  pinMode(pinS2, OUTPUT);
  pinMode(pinS3, OUTPUT);
  pinMode(pinOUT, INPUT);
  digitalWrite(pinS0, HIGH);
  digitalWrite(pinS1, LOW);
}

void leituraCores(int &v, int pinS0, int pinS1, int pinS2, int pinS3, int pinOUT) {
  digitalWrite(pinS2, LOW);
  digitalWrite(pinS3, HIGH); // Define para ler a cor verde

  v = pulseIn(pinOUT, digitalRead(pinOUT) == HIGH ? LOW : HIGH);
}

void loop(){
    // leituraCores(verde, PINO_SENSOR_S0, PINO_SENSOR_S1, PINO_SENSOR_S2, PINO_SENSOR_S3, PINO_SENSOR_OUT);
    // leituraCores(verde2, PINO_SENSOR_S0_2, PINO_SENSOR_S1_2, PINO_SENSOR_S2_2, PINO_SENSOR_S3_2, PINO_SENSOR_OUT_2);

    // unsigned long tempoAtual = millis();

    // autoTrack();
    frente();

    delay(500);
}