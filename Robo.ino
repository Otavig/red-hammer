// Motores
#define in1 19
#define in2 17
#define in3 15
#define in4 14
#define motorENA 18
#define motorENB 16

// Sensores de refletância
#define s5 53
#define s4 52
#define s3 51
#define s2 50
#define s1 49

#define BRANCO 1

// Especificações 
#define velocidade 255

#define velocVirar 120

// Especificações 
#define velocidade 80
#define velocVirar 140

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

    // Iniciar motores como parados
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

void frente() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    analogWrite(motorENA, 180);
    analogWrite(motorENB, 180);
}

void direita() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    analogWrite(motorENA, 255);
    analogWrite(motorENB, 80);
}

void direita90(){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    analogWrite(motorENA, 180);
    analogWrite(motorENB, 200);
}

void esquerda() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    analogWrite(motorENA, 180);
    analogWrite(motorENB, 80);
}

void esquerda90() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    analogWrite(motorENA, 255); // Motor A vai para trás
    analogWrite(motorENB, 255);
}


void parar(){
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

void autoTrack(){
  int s5_ = digitalRead(s5);
  int s4_ = digitalRead(s4);
  int s3_ = digitalRead(s3);
  int s2_ = digitalRead(s2);
  int s1_ = digitalRead(s1);

    // Exibir os valores dos sensores no console
    // Serial.print("SensorAi| ");
    // Serial.println(s2_);
    // Serial.print("SensorAiai| ");
    // Serial.println(s3_);
    // Serial.print("SensorAiaiai| ");
    // Serial.println(s4_);
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
    direita90();
    delay(600);
  }

  if(s5_ != BRANCO){
    frente();
    delay(200);
    parar();
    delay(2000);
    parar();
    delay(1000);
    esquerda90();
    delay(600);

  }

    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);

  // delay(1000);
}

void loop(){
    // // Leitura dos sensores de refletância
    // int valorMeio = digitalRead(meio);
    // int valorMeioE = digitalRead(mE);
    // int valorMeioD = digitalRead(mD);
    // int valorExtremaE = digitalRead(mEE);
    // int valorExtremaD = digitalRead(mED);

    // // Exibir os valores dos sensores no console
    // Serial.print("Valor do sensor de meio: ");
    // Serial.println(valorMeio);
    // Serial.print("Valor do sensor de meio esquerda: ");
    // Serial.println(valorMeioE);
    // Serial.print("Valor do sensor de meio direita: ");
    // Serial.println(valorMeioD);
    // Serial.print("Valor do sensor de extrema esquerda: ");
    // Serial.println(valorExtremaE);
    // Serial.print("Valor do sensor de extrema direita: ");
    // Serial.println(valorExtremaD);
    // Serial.println(); // Adiciona uma linha em branco para facilitar a leitura

    // // Você pode adicionar mais lógica conforme necessário

    // frente();
    // delay(1000);
    // tras();
    // delay(1000);

    autoTrack();

}
