// Definição dos pinos para os sensores de cor (esquerda e direita)
const int PINO_SENSOR_S0E = 6;
const int PINO_SENSOR_S1E = 7;
const int PINO_SENSOR_S2E = 17;
const int PINO_SENSOR_S3E = 16;
const int PINO_SENSOR_OUTE = 41;

const int PINO_SENSOR_S0D = 2;
const int PINO_SENSOR_S1D = 3;
const int PINO_SENSOR_S2D = 4;
const int PINO_SENSOR_S3D = 5;
const int PINO_SENSOR_OUTD = 15;

// Pinos da ponte H
const int ENA = 9;
const int IN1 = 51;
const int IN2 = 49;
const int IN3 = 47;
const int IN4 = 45;
const int ENB = 8;

// Velocidade dos motores (valores entre 0 e 255)
int velocidade = 80;

// Variáveis que armazenam o valor das cores
int vermelhoE = 0;
int verdeE = 0;
int azulE = 0;
int vermelhoD = 0;
int verdeD = 0;
int azulD = 0;

// Endereço I2C do MPU6050
const int MPU_ADDR = 0x68;

// Variáveis para armazenar os dados do sensor MPU6050
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Pinos do sensor ultrassônico
const int PINO_SENSOR_ECHO = A1;
const int PINO_SENSOR_TRIGGER = A0;

// Constantes auxiliares para controlar os motores
const int DISTANCIA_SEGURA = 25; // [cm]
const int PAUSA = 100; // [ms]

// Funções auxiliares
int ler_distancia(void);
void mover_frente(void);
void mover_tras(void);
void parar(void);
void virar_direita_90(void);
void virar_esquerda_90(void);
void desviar_obstaculo(void);
void check_inclinacao(void);
void leitura_coresE(void);
void leitura_coresD(void);
void ajustarVelocidadeMotores(int velocidade);

void setup() {
  Serial.begin(9600);

  // Configuração dos pinos da ponte H como saída
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Inicializa os motores com velocidade máxima
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // Configuração dos pinos dos sensores de cor
  pinMode(PINO_SENSOR_S0E, OUTPUT);
  pinMode(PINO_SENSOR_S1E, OUTPUT);
  pinMode(PINO_SENSOR_S2E, OUTPUT);
  pinMode(PINO_SENSOR_S3E, OUTPUT);
  pinMode(PINO_SENSOR_OUTE, INPUT);
  pinMode(PINO_SENSOR_S0D, OUTPUT);
  pinMode(PINO_SENSOR_S1D, OUTPUT);
  pinMode(PINO_SENSOR_S2D, OUTPUT);
  pinMode(PINO_SENSOR_S3D, OUTPUT);
  pinMode(PINO_SENSOR_OUTD, INPUT);

  // Configuração dos pinos do sensor ultrassônico
  pinMode(PINO_SENSOR_ECHO, INPUT);
  pinMode(PINO_SENSOR_TRIGGER, OUTPUT);
  digitalWrite(PINO_SENSOR_TRIGGER, LOW);

  // Inicialização do MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  digitalWrite(PINO_SENSOR_S0E, LOW);
  digitalWrite(PINO_SENSOR_S1E, HIGH);
  digitalWrite(PINO_SENSOR_S0D, LOW);
  digitalWrite(PINO_SENSOR_S1D, HIGH);
}

void loop() {
  leitura_coresE();
  leitura_coresD();
  ajustarVelocidadeMotores(velocidade);

  if (vermelhoE < azulE && vermelhoE < verdeE) {
    Serial.println("Vermelho");
  }
  
  if (azulE < vermelhoE && azulE < verdeE) {
    Serial.println("Azul");
  }
  
  if (vermelhoE >= 270 && vermelhoE <= 760 &&
      verdeE >= 270 && verdeE <= 760 &&
      azulE >= 270 && azulE <= 760 &&
      (vermelhoE + verdeE + azulE) / 3 > 300) {
    Serial.println("Verde E detectado");
    digitalWrite(ENA, LOW);
    digitalWrite(ENB, LOW);
    delay(5000);
  }

  if (vermelhoD >= 270 && vermelhoD <= 760 &&
      verdeD >= 270 && verdeD <= 760 &&
      azulD >= 270 && azulD <= 760 &&
      (vermelhoD + verdeD + azulD) / 3 > 300) {
    Serial.println("Verde D detectado");
    digitalWrite(ENA, LOW);
    digitalWrite(ENB, LOW);
    delay(5000);
  }

  check_distancia();
  check_inclinacao();

  delay(100);
}

void leitura_coresE() {
  digitalWrite(PINO_SENSOR_S2E, LOW);
  digitalWrite(PINO_SENSOR_S3E, LOW);
  vermelhoE = pulseIn(PINO_SENSOR_OUTE, digitalRead(PINO_SENSOR_OUTE) == HIGH ? LOW : HIGH);
  digitalWrite(PINO_SENSOR_S3E, HIGH);
  azulE = pulseIn(PINO_SENSOR_OUTE, digitalRead(PINO_SENSOR_OUTE) == HIGH ? LOW : HIGH);
  digitalWrite(PINO_SENSOR_S2E, HIGH);
  verdeE = pulseIn(PINO_SENSOR_OUTE, digitalRead(PINO_SENSOR_OUTE) == HIGH ? LOW : HIGH);
}

void leitura_coresD() {
  digitalWrite(PINO_SENSOR_S2D, LOW);
  digitalWrite(PINO_SENSOR_S3D, LOW);
  vermelhoD = pulseIn(PINO_SENSOR_OUTD, digitalRead(PINO_SENSOR_OUTD) == HIGH ? LOW : HIGH);
  digitalWrite(PINO_SENSOR_S3D, HIGH);
  azulD = pulseIn(PINO_SENSOR_OUTD, digitalRead(PINO_SENSOR_OUTD) == HIGH ? LOW : HIGH);
  digitalWrite(PINO_SENSOR_S2D, HIGH);
  verdeD = pulseIn(PINO_SENSOR_OUTD, digitalRead(PINO_SENSOR_OUTD) == HIGH ? LOW : HIGH);
}

void ajustarVelocidadeMotores(int velocidade) {
  analogWrite(ENA, velocidade);
  analogWrite(ENB, velocidade);
}

int ler_distancia() {
  digitalWrite(PINO_SENSOR_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(PINO_SENSOR_TRIGGER, LOW);
  return pulseIn(PINO_SENSOR_ECHO, HIGH) / 58;
}

void mover_frente() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 140);
  analogWrite(ENB, 140);
}

void mover_tras() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 140);
  analogWrite(ENB, 140);
}

void parar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void virar_direita_90() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 140);
  analogWrite(ENB, 140);
  delay(1700);
  parar();
}

void virar_esquerda_90() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 140);
  analogWrite(ENB, 140);
  delay(1700);
  parar();
}

void desviar_obstaculo() {
  parar();
  delay(500);
  mover_tras();
  delay(1000);
  parar();
  virar_direita_90();
  mover_frente();
  delay(2000);
  parar();
  virar_esquerda_90();
  mover_frente();
  delay(2000);
  parar();
  virar_esquerda_90();
  mover_frente();
  delay(2000);
  parar();
  virar_direita_90();
  mover_frente();
  delay(1000);
  parar();
}

void check_distancia() {
  int distancia = ler_distancia();
  if (distancia < DISTANCIA_SEGURA) {
    desviar_obstaculo();
  }
  delay(PAUSA);
}

void check_inclinacao() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();

  float inclinacaoX = atan2(ay, az + abs(ax)) * 180 / PI;
  float inclinacaoY = atan2(ax, az + abs(ay)) * 180 / PI;

  if (abs(inclinacaoX) > 15 || abs(inclinacaoY) > 15) {
    parar();
  }
}
