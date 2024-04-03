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

#define BRANCO 1

int veloc = 180;
int verde = 0;

unsigned long ultimaDetecaoVerde = 0;
unsigned long intervaloDetec = 2000;

void configurarPinosSensor(int pinS0, int pinS1, int pinS2, int pinS3, int pinOUT) {
  pinMode(pinS0_e, OUTPUT);
  pinMode(pinS1_e, OUTPUT);
  pinMode(pinS2_e, OUTPUT);
  pinMode(pinS3_e, OUTPUT);
  pinMode(pinOUT_e, INPUT);
  digitalWrite(pinS0_e, HIGH);
  digitalWrite(pinS1_e, LOW);
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
  configurarPinosSensor(pinS0_e, pinS1_e, pinS2_e, pinS3_e, pinOUT_e);
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
    analogWrite(motorENB, (veloc-100));
}

void esquerda(){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    analogWrite(motorENA, (veloc-100));
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
  digitalWrite(pinS2, LOW);
  digitalWrite(pinS3, HIGH);

  v = pulseIn(pinOUT, digitalRead(pinOUT) == HIGH ? LOW : HIGH);
}

void autoTrack(){
  int s5_ = digitalRead(s5);
  int s4_ = digitalRead(s4);
  int s3_ = digitalRead(s3);
  int s2_ = digitalRead(s2);
  int s1_ = digitalRead(s1);

    // Exibir os valores dos sensores no console
    // Serial.print("SensorAi| ");
    // Serial.println(s2);
    // Serial.print("SensorAiai| ");
    // Serial.println(s3);
    // Serial.print("SensorAiaiai| ");
    // Serial.println(s4);
    // Serial.print("Valor do sensor de extrema esquerda: ");
    // Serial.println(valorExtremaE);
    // Serial.print("Valor do sensor de extrema direita: ");
    // Serial.println(valorExtremaD);
    // Serial.println(); // Adiciona uma linha em branco para facilitar a leitura

  while(s3_ != BRANCO){
    frente();
    break;
  }
  while(s2_ != BRANCO){
    direita();
    break;
  }
  while(s4_ != BRANCO){
    esquerda();
    break;
  }

  while(s2_ == BRANCO && s3_ == BRANCO && s4_ == BRANCO){
    frente();
    break;
  }

  if(s1_ != BRANCO){
    frente();
    delay(200);
    parar();
    delay(2000);
    parar();
    delay(1000);
    dCurva();
    delay(600);
  }

  if(s5_ != BRANCO){
    frente();
    delay(200);
    parar();
    delay(2000);
    parar();
    delay(1000);
    eCurva();
    delay(600);

  }

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

    // Cor
    leiturasCores(verde, pinS0_e, pinS1_e, pinS2_e, pinS3_e, pinOUT_e);
    unsigned long tempoAtual = millis();
    
    if (verde > 0 && tempoAtual - ultimaDetecaoVerde >= intervaloDetec) {
      frente();
      delay(200);
      parar();
      delay(2000);
      parar();
      delay(1000);
      dCurva();
      delay(600);
      ultimaDetecaoVerde = tempoAtual;
    } else if (verde == 0) {
      Serial.println("Verde não encontrado");
    }

    // Linha
    autoTrack();
}