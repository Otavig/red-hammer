// Definições de pinos para o sensor de cor direito (Sensor 1)
#define S0_1 4
#define S1_1 5
#define S2_1 3
#define S3_1 2
#define sensorOut_1 6

// Definições de pinos para o sensor de cor esquerdo (Sensor 2)
#define S0_2 9
#define S1_2 10
#define S2_2 11
#define S3_2 12
#define sensorOut_2 13

void setup() {
  // Inicialização dos pinos do sensor 1
  pinMode(S0_1, OUTPUT);
  pinMode(S1_1, OUTPUT);
  pinMode(S2_1, OUTPUT);
  pinMode(S3_1, OUTPUT);
  pinMode(sensorOut_1, INPUT);

  // Inicialização dos pinos do sensor 2
  pinMode(S0_2, OUTPUT);
  pinMode(S1_2, OUTPUT);
  pinMode(S2_2, OUTPUT);
  pinMode(S3_2, OUTPUT);
  pinMode(sensorOut_2, INPUT);

  // Configuração do sensor 1: Frequência de escala de 2%
  digitalWrite(S0_1, LOW);
  digitalWrite(S1_1, HIGH);

  // Configuração do sensor 2: Frequência de escala de 2%
  digitalWrite(S0_2, LOW);
  digitalWrite(S1_2, HIGH);

  // Inicialização da comunicação serial
  Serial.begin(9600);
}

char lerCor(int sensorOut, int S2, int S3) {
  int redColor = 0;
  int greenColor = 0;
  int blueColor = 0;
  char color;

  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redColor = pulseIn(sensorOut, LOW);
  redColor = map(redColor, 70, 120, 255, 0);

  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenColor = pulseIn(sensorOut, LOW);
  greenColor = map(greenColor, 100, 199, 255, 0);

  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueColor = pulseIn(sensorOut, LOW);
  blueColor = map(blueColor, 38, 84, 255, 0);

  if (redColor > greenColor && redColor > blueColor && (redColor < -4200 && redColor > -4900)) {
    color = 'V'; // Vermelho
  } else if (greenColor > redColor && greenColor > blueColor && (greenColor < -2100 && greenColor > -2300)) {
    color = 'G'; // Verde
  } else if (blueColor > redColor && blueColor > greenColor && (blueColor < -3100 && blueColor > -3300)) {
    color = 'A'; // Azul
  } else {
    color = 'I'; // Indefinido
  }

  return color;
}

void loop() {
  // Chamada da função para obter a cor do sensor direito
  char corDireita = lerCor(sensorOut_1, S2_1, S3_1);
  // Chamada da função para obter a cor do sensor esquerdo
  char corEsquerda = lerCor(sensorOut_2, S2_2, S3_2);

  // Verifica se alguma das cores é verde
  if (corDireita == 'G' || corEsquerda == 'G') {
    // Se sim, imprime "Verde detectado" e retorna 'G'
    Serial.println("Verde detectado!");
    Serial.print("Valor do sensor direito: ");
    Serial.println(corDireita);
    Serial.print("Valor do sensor esquerdo: ");
    Serial.println(corEsquerda);
    delay(1000);
    return;
  }

  delay(1000); // Pausa para legibilidade na serial
}

