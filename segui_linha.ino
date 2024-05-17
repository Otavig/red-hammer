// Define os pinos dos motores
#define ENA 9
#define ENB 8
#define IN1 51
#define IN2 49
#define IN3 47
#define IN4 45
// Define os pinos dos sensores de linha
#define SENSOR_ESQUERDA_BAIXO 39
#define SENSOR_ESQUERDA 11
#define SENSOR_MEIO 12
#define SENSOR_DIREITA 13
#define SENSOR_DIREITA_BAIXO 37

// Sensores de linha auxiliares
#define CURVA_ESQUERDA 10
#define CURVA_DIREITA 14

const int VELOCIDADE_BAIXA = 80;
const int VELOCIDADE_ALTA = 100;
const int VELOCIDADE_VIRAR = 140;

// Função para mover para frente
void moverFrente(int velocidade) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, velocidade);
  analogWrite(ENB, velocidade);
}
// Função para mover para trás
void moverTras(int velocidade) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, velocidade);
  analogWrite(ENB, velocidade);
}
// Função para virar à esquerda
void virarEsquerda(int velocidade) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, velocidade - 80);
  analogWrite(ENB, velocidade);
}
// Função para virar à direita
void virarDireita(int velocidade) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, velocidade);
  analogWrite(ENB, velocidade - 80);
}

void virarDireita90(int velocidade){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, VELOCIDADE_VIRAR);
  analogWrite(ENB, VELOCIDADE_VIRAR);
}
void virarEsquerda90(int velocidade){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, VELOCIDADE_VIRAR);
  analogWrite(ENB, VELOCIDADE_VIRAR);
}

// Função para parar
void pararMovimento() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void grau_curva(int L, int R){
  if (L == 0 && R == 0){
    moverFrente(VELOCIDADE_BAIXA);  
  } else if (L == 0 && R == 1){
    moverFrente(VELOCIDADE_ALTA);
    delay(700);
    virarDireita90(VELOCIDADE_ALTA);
    delay(1000);
    int estado = 1;
    int S_C;
    while (estado == 1){
      S_C = digitalRead(SENSOR_MEIO);
      if (S_C == 0){
        estado = 0;
        break;
      } else{
      virarDireita90(VELOCIDADE_ALTA);
      }
    }
    return;
  } else if (L == 1 && R == 0){
    moverFrente(VELOCIDADE_ALTA);
    delay(500);
    virarEsquerda90(VELOCIDADE_ALTA);
    delay(1000);
    int estado = 1;
    int S_C;
    while (estado == 1){
      S_C = digitalRead(SENSOR_MEIO);
      if (S_C == 0){
        estado = 0;
        break;
      } else{
      virarEsquerda90(VELOCIDADE_ALTA);
      }
    }
    return;
  } else {
    return;
  }

  return;
}

void setup() {
  Serial.begin(9600);

  // Configura os pinos dos motores como saída
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  // Configura os pinos dos sensores de linha como entrada
  pinMode(SENSOR_ESQUERDA, INPUT);
  pinMode(SENSOR_MEIO, INPUT);
  pinMode(SENSOR_DIREITA, INPUT);
  pinMode(SENSOR_DIREITA_BAIXO, INPUT);
  pinMode(SENSOR_ESQUERDA_BAIXO, INPUT);

  // Sensores auxiliares para curva do robô
  pinMode(CURVA_ESQUERDA, INPUT);
  pinMode(CURVA_DIREITA, INPUT);
}

void rastrear(){
  // Sensores principais
  int S_ESQUERDA = digitalRead(SENSOR_ESQUERDA);
  int S_MEIO = digitalRead(SENSOR_MEIO);
  int S_DIREITA = digitalRead(SENSOR_DIREITA);

  // Sensores baixos
  int S_ESQUERDA_BAIXO = digitalRead(SENSOR_ESQUERDA_BAIXO);
  int S_DIREITA_BAIXO = digitalRead(SENSOR_DIREITA_BAIXO);
  Serial.println(S_ESQUERDA_BAIXO);
  Serial.println(S_DIREITA_BAIXO);

  // Sensores auxiliares
  int S_CURVA_ESQUERDA = digitalRead(CURVA_ESQUERDA);
  int S_CURVA_DIREITA = digitalRead(CURVA_DIREITA);

  // Verificação de curva primária com refletância
  grau_curva(S_CURVA_ESQUERDA, S_CURVA_DIREITA);
  // Básico
  if (S_ESQUERDA_BAIXO == 1){
      virarDireita(VELOCIDADE_VIRAR);
  }else if(S_DIREITA_BAIXO == 1){
      virarEsquerda(VELOCIDADE_VIRAR);
  }
  else{
    if (S_ESQUERDA == 1 && S_MEIO == 0 && S_DIREITA == 1){
        moverFrente(VELOCIDADE_BAIXA);
    } else if (S_ESQUERDA == 0 && S_MEIO == 1 && S_DIREITA == 1){
        virarDireita(VELOCIDADE_VIRAR);
        delay(50);
    } else if (S_ESQUERDA == 1 && S_MEIO == 1 && S_DIREITA == 0){
        virarEsquerda(VELOCIDADE_VIRAR);
        delay(50);
      }else {
          moverFrente(VELOCIDADE_VIRAR);
      
      }
    }
  }

void loop() {
  rastrear();
}
