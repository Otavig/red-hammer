// Definir os pinos dos motores
#define ENA 9
#define ENB 8
#define IN1 51
#define IN2 49
#define IN3 47
#define IN4 45

// Definir os pinos dos sensores de linha
int lineFollowSensor0 = 11;
int lineFollowSensor1 = 39;
int lineFollowSensor2 = 12;
int lineFollowSensor3 = 37;
int lineFollowSensor4 = 13;

// Array para armazenar os valores dos sensores
int LFSensor[5] = {0, 0, 0, 0, 0};

int erro;
int kp = 50;

// Função para ler os valores dos sensores de linha
String lerSensores() {
    String sensorValues = "";
    LFSensor[0] = digitalRead(lineFollowSensor0);
    LFSensor[1] = digitalRead(lineFollowSensor1);
    LFSensor[2] = digitalRead(lineFollowSensor2);
    LFSensor[3] = digitalRead(lineFollowSensor3);
    LFSensor[4] = digitalRead(lineFollowSensor4);
    
    // Inverter os valores dos sensores 1 e 3
    if (LFSensor[1] == 1) {LFSensor[1] = 0;} else {LFSensor[1] = 1;}
    if (LFSensor[3] == 1) {LFSensor[3] = 0;} else {LFSensor[3] = 1;}

    for (int i = 0; i < 5; i++) {
      if (LFSensor[i] == 1){
        sensorValues += String(0);
      } else {
        sensorValues += String(1);
      }
    }

    return sensorValues;
}

void verificarErro(String sensor) {
    if (sensor == "00001") { erro = 4; }
    else if (sensor == "00010") { erro = 2; }
    else if (sensor == "00011") { erro = 3; }
    else if (sensor == "00100") { erro = 0; }
    else if (sensor == "00110") { erro = 1; }
    else if (sensor == "01000") { erro = -2; }
    else if (sensor == "01100") { erro = -1; }
    else if (sensor == "10000") { erro = -4; }
    else if (sensor == "11000") { erro = -3; }
    else { erro = 0; } // Valor padrão para situações não cobertas

    return;
}

// Funções para controle dos motores
void moverFrente(int velocidade) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, velocidade);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, velocidade);
}

void moverTras(int velocidade) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, velocidade);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, velocidade);
}

void moverEsquerda(int velocidade) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, velocidade);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, velocidade);
}

void moverDireita(int velocidade) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, velocidade);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, velocidade);
}

void pararMotores() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
}

void setup() {
    // Inicializar os pinos dos motores como saída
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Inicializar os pinos dos sensores como entrada
    pinMode(lineFollowSensor0, INPUT);
    pinMode(lineFollowSensor1, INPUT);
    pinMode(lineFollowSensor2, INPUT);
    pinMode(lineFollowSensor3, INPUT);
    pinMode(lineFollowSensor4, INPUT);

    // Iniciar a comunicação serial para depuração
    Serial.begin(9600);
}

void loop() {
    // Ler os valores dos sensores
    String sensorValues = lerSensores();
    verificarErro(sensorValues);

    // Exibir os valores dos sensores em formato binário para depuração
    Serial.print("Sensor Values: ");
    Serial.println(sensorValues);
    Serial.print("Erro: ");
    Serial.println(erro);

    // Controlar os motores com base no valor do erro
    int velocidadeBase = 200;
    int velocidadeCorrecao = kp * erro;

    if (erro == 0) {
        moverFrente(velocidadeBase);
    } else if (erro > 0) {
        moverDireita(velocidadeBase + velocidadeCorrecao);
    } else {
        moverEsquerda(velocidadeBase - velocidadeCorrecao);
    }

    // Adicionar um pequeno atraso para evitar sobrecarga na comunicação serial
    delay(500);
}
