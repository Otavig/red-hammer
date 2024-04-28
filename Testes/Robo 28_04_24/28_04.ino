#define ENA 18
#define IN1 19
#define IN2 17
#define IN3 15
#define IN4 14
#define ENB 16
 
#define r1 2
#define r2 3
#define r3 4
#define r4 5
#define r5 6

// Definição dos pinos de refletância
int refletanciaPinos[5] = {r1, r2, r3, r4, r5};

// Array de velocidades permitidas
int velocidadesFrente[] = {130, 140, 160, 180, 200};
int velocidadesLado[] = {130, 140, 160, 180, 200};

// Pesos para as velocidades permitidas
float pesosFrente[] = {0.1, 0.2, 0.4, 0.6, 0.8};
float pesosLado[] = {0.1, 0.2, 0.4, 0.6, 0.8};

int a, b, c, d, e;
int setpoint = 3; // Sensor do meio: 1 branco, 0 preto

// Variáveis do PID
double kp = 1.0;
double ki = 0.1;
double kd = 0.05;
double lastError = 0;
double integral = 0;

void setup(){
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
 
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Inicialização dos pinos de refletância
  for(int i = 0; i < 5; i++) {
    pinMode(refletanciaPinos[i], INPUT);
  }
 
  Serial.begin(9600);
}
 
void mfrente(int vel){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
 
  analogWrite(ENA, vel);
  analogWrite(ENB, vel);
}
 
void mdireita(int vinput){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
 
  analogWrite(ENA, vinput);
  analogWrite(ENB, 120);
}
 
void mesquerda(int vinput){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
 
  analogWrite(ENA, 120);
  analogWrite(ENB, vinput);
}
 
void tras() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
 
    analogWrite(ENA, 180);
    analogWrite(ENB, 180);
}

void mparar(){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
 
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}
 
// Função para calcular a média ponderada das velocidades permitidas com base nos pesos
int calcularVelocidade(int *velocidades, float *pesos, int tamanho) {
  float somaPesos = 0;
  int velocidadeFinal = 0;

  for(int i = 0; i < tamanho; i++) {
    somaPesos += pesos[i];
  }

  for(int i = 0; i < tamanho; i++) {
    velocidadeFinal += velocidades[i] * (pesos[i] / somaPesos);
  }

  return velocidadeFinal;
}

// Função para controlar o movimento do robô com base nos valores dos sensores de refletância
void track(){
  // Leitura dos sensores de refletância
  a = digitalRead(r1);
  b = digitalRead(r2);
  c = digitalRead(r3);
  d = digitalRead(r4);
  e = digitalRead(r5);

  // Calculo do erro
  int error = setpoint - c;

  // Ação proporcional
  double pTerm = kp * error;

  // Ação integral
  integral += error;
  double iTerm = ki * integral;

  // Ação derivativa
  double dTerm = kd * (error - lastError);
  lastError = error;

  // Calculo da saída do PID
  int velPID = pTerm + iTerm + dTerm;

  // Verificação com base nos valores dos sensores de refletância
  if (c == LOW) {
    int vel = calcularVelocidade(velocidadesFrente, pesosFrente, sizeof(velocidadesFrente) / sizeof(velocidadesFrente[0]));
    mfrente(vel);
  } 
  else if (b == LOW) {
    mesquerda(velPID);
  } 
  else if (d == LOW) {
    mdireita(velPID);
  } 
  else {
    mparar();
  }

  // Saída para depuração
  Serial.print("C: "); Serial.print(c);
  Serial.print("B: "); Serial.print(b);
  Serial.print("D: "); Serial.print(d);
  Serial.println();
}
 
void loop(){
  mparar();
  track();
}
