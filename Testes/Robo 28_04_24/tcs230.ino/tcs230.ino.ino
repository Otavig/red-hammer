#define S0_L 2 // Pinos de controle do TCS230 esquerdo
#define S1_L 3
#define S2_L 4
#define S3_L 5
#define OUT_L 6 // Pino de saída do TCS230 esquerdo

#define S0_R 7 // Pinos de controle do TCS230 direito
#define S1_R 8
#define S2_R 9
#define S3_R 10
#define OUT_R 11 // Pino de saída do TCS230 direito

#define limiarVerde 500 // Limiar para detecção de verde (ajuste conforme necessário)

void setup() {
  // Configuração dos pinos do TCS230 esquerdo
  pinMode(S0_L, OUTPUT);
  pinMode(S1_L, OUTPUT);
  pinMode(S2_L, OUTPUT);
  pinMode(S3_L, OUTPUT);
  pinMode(OUT_L, INPUT);

  // Configuração dos pinos do TCS230 direito
  pinMode(S0_R, OUTPUT);
  pinMode(S1_R, OUTPUT);
  pinMode(S2_R, OUTPUT);
  pinMode(S3_R, OUTPUT);
  pinMode(OUT_R, INPUT);
  
  // Configuração inicial dos TCS230 (exemplo)
  digitalWrite(S0_L, HIGH);
  digitalWrite(S1_L, LOW);
  digitalWrite(S0_R, HIGH);
  digitalWrite(S1_R, LOW);
  
  Serial.begin(9600); // Inicializa a comunicação serial
}

void loop() {
  // Realizar a leitura dos sensores TCS230
  int corVerdeEsq = lerCorTCS230(S2_L, S3_L, OUT_L); // Ler cor verde da esquerda
  int corVerdeDir = lerCorTCS230(S2_R, S3_R, OUT_R); // Ler cor verde da direita

  // Calcular a média ponderada das leituras dos dois sensores
  // Ponderação pode ser ajustada conforme necessário
  float mediaCorVerde = (corVerdeEsq * 0.5) + (corVerdeDir * 0.5);

  // Verificar se a cor lida é verde com base no limiar
  if (mediaCorVerde > limiarVerde) {
    // A cor é considerada verde
    Serial.println("Cor Verde Detectada!");
  } else {
    // A cor não é verde
    Serial.println("Outra Cor Detectada!");
  }

  // Aguardar um curto período de tempo antes da próxima leitura
  delay(100);
}

// Função para ler a cor verde de um sensor TCS230 específico
int lerCorTCS230(int pinS2, int pinS3, int pinOut) {
  // Configurar os pinos S2 e S3 para o sensor TCS230 específico
  digitalWrite(pinS2, LOW);
  digitalWrite(pinS3, HIGH);

  // Realizar a leitura do sensor TCS230
  int cor = pulseIn(pinOut, digitalRead(pinOut) == HIGH ? LOW : HIGH); // Leitura do pulso de saída

  return cor;
}
