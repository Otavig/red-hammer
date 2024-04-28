// motores
#define motorENA 18
#define motorENB 16
#define in1 19
#define in2 17
#define in3 15
#define in4 14

// Sensores de refletância
#define s5 53
#define s4 52
#define s3 51
#define s2 50
#define s1 49

//Sensor de COR esquerdo
#define pinS0_e A6
#define pinS1_e A5
#define pinS2_e A14
#define pinS3_e A15
#define pinOUT_e A13

//Sensor de COR direito
#define pinS0_d A9
#define pinS1_d A8
#define pinS2_d A10
#define pinS3_d A12
#define pinOUT_d A11

#define BRANCO 1

// Sensor de cor Armazenamento
int vermelho = 0;
int verde = 0;
int azul = 0;

// Verdes 
int verde_esquerdo = 0;
int verde_direito = 0;

int veloc = 200;

void configurarPinosSensor(int pinS0, int pinS1, int pinS2, int pinS3, int pinOUT) {
  pinMode(pinS0, OUTPUT);
  pinMode(pinS1, OUTPUT);
  pinMode(pinS2, OUTPUT);
  pinMode(pinS3, OUTPUT);
  pinMode(pinOUT, INPUT);

  digitalWrite(pinS0, HIGH);
  digitalWrite(pinS1, LOW);
}

void setup(){
  Serial.begin(9600);

  // Configurar pinos dos sensores de refletância
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);

  // Configurando motores
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Sensor Cor
  // configurarPinosSensor(pinS0_e, pinS1_e, pinS2_e, pinS3_e, pinOUT_e);
  // configurarPinosSensor(pinS0_d, pinS1_d, pinS2_d, pinS3_d, pinOUT_d);
}


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


void leiturasCores(int &v, int pinS0, int pinS1, int pinS2, int pinS3, int pinOUT){
}

void autoTrack(){
    // Linha
      int s5_ = digitalRead(s5);
      int s4_ = digitalRead(s4);
      int s3_ = digitalRead(s3);
      int s2_ = digitalRead(s2);
      int s1_ = digitalRead(s1);

      while(s3_ != BRANCO){
        frente();
        break;
      }
      while(s2_ != BRANCO){
        direita();
        delay(5);
        break;
      }
      while(s4_ != BRANCO){
        esquerda();
        delay(5);
        break;
      }

      while(s2_ == BRANCO && s3_ == BRANCO && s4_ == BRANCO){
        frente();
        break;
      }

      //Encruzilhada
      while(s1_ != BRANCO && s5_ != BRANCO){
        frente();
        break;
      }

    // if(s1_ != BRANCO){

    // }

    // if(s5_ != BRANCO){

    // }

    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);

    // delay(1000);
}

void loop(){
    // Iniciar motores como parados
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);

    // Ler as cores
    // leiturasCores(verde_esquerdo, pinS0_e, pinS1_e, pinS2_e, pinS3_e, pinOUT_e);
    // leiturasCores(verde_direito, pinS0_d, pinS1_d, pinS2_d, pinS3_d, pinOUT_d);

    // // Imprimir os valores para debug
    // Serial.println("---------------");
    // Serial.println(verde_esquerdo);
    // Serial.println(verde_direito);
  
    // Linha
    autoTrack();

    // // Se ambos os sensores detectaram verde
    // if (verde_esquerdo == 1 && verde_direito == 1) {
    //     frente(); // Avançar
    //     delay(200);
    //     parar();
    //     delay(1000);
    // }
    // // Se apenas o sensor esquerdo detectou verde
    // if (verde_esquerdo == 1) {
    //     frente(); // Avançar
    //     delay(200);
    //     parar();
    //     delay(1000);
    //     eCurva(); // Fazer uma curva para a esquerda
    //     delay(600);
    // }
    // // Se apenas o sensor direito detectou verde
    // if (verde_direito == 1) {
    //     frente(); // Avançar
    //     delay(200);
    //     parar();
    //     delay(1000);
    //     dCurva(); // Fazer uma curva para a direita
    //     delay(600);
    // }
}
