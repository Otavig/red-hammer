#include <Arduino.h>

// motores
#define motorENA 18
#define motorENB 16
#define in1 19
#define in2 17
#define in3 15
#define in4 14

// Sensor de COR esquerdo (TCS230)
#define SensorEsq_S0 A6
#define SensorEsq_S1 A5
#define SensorEsq_S2 A14
#define SensorEsq_S3 A15
#define SensorEsq_OUT A13

// Sensor de COR direito (TCS230)
#define SensorDir_S0 A9
#define SensorDir_S1 A8
#define SensorDir_S2 A10
#define SensorDir_S3 A12
#define SensorDir_OUT A11

// Sensores de refletância
#define s5 53
#define s4 52
#define s3 51
#define s2 50
#define s1 49

// Velocidade do rôbo
int speed = 200;

// Variaveis que armazenam o valor das cores
int verdeEsquerdo = 0;
int verdeDireito = 0;

#define white 1

void motor_frente(){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    analogWrite(motorENA, speed);
    analogWrite(motorENB, speed);
}

void motor_direito(){
    digitalWrite(in1, speed);
    digitalWrite(in2, speed);
    digitalWrite(in3, speed);
    digitalWrite(in4, LOW);

    analogWrite(motorENA, 200);
    analogWrite(motorENB, (speed-80));
}

void motor_esquerdo(){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    analogWrite(motorENA, (speed-80));
    analogWrite(motorENB, 200);
}

void motor_eCurva(){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    analogWrite(motorENA, (speed+75));
    analogWrite(motorENB, (speed+75));
}

void motor_dCurva(){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    analogWrite(motorENA, (speed+75));
    analogWrite(motorENB, (speed+75));
}

void motor_parar(){
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

void motor_tras() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    analogWrite(motorENA, 180);
    analogWrite(motorENB, 180);
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

    // Configurando Sensores de COR
    // Sensor de COR esquerdo (TCS230)
    pinMode(SensorEsq_S0, OUTPUT);
    pinMode(SensorEsq_S1, OUTPUT);
    pinMode(SensorEsq_S2, OUTPUT);
    pinMode(SensorEsq_S3, OUTPUT);
    pinMode(SensorEsq_OUT, INPUT);

    // Sensor de COR direito (TCS230)
    pinMode(SensorDir_S0, OUTPUT);
    pinMode(SensorDir_S1, OUTPUT);
    pinMode(SensorDir_S2, OUTPUT);
    pinMode(SensorDir_S3, OUTPUT);
    pinMode(SensorDir_OUT, INPUT);
 }




void leitura_cores(){
    // Esquerdo 
    digitalWrite(SensorEsq_S0, LOW);
    digitalWrite(SensorEsq_S3, LOW);
    
    // Direito 
    digitalWrite(SensorDir_S0, LOW);
    digitalWrite(SensorDir_S3, LOW);

    if(digitalRead(SensorEsq_OUT) == HIGH){
        verdeEsquerdo = pulseIn(SensorEsq_OUT, LOW);
    } else {
        verdeEsquerdo = pulseIn(SensorEsq_OUT, HIGH);
    }

    if(digitalRead(SensorDir_OUT) == HIGH){
        verdeDireito = pulseIn(SensorDir_OUT, LOW);
    } else {
        verdeDireito = pulseIn(SensorDir_OUT, HIGH);
    }
}

int leitura_sensor() {
    // Ler os valores dos sensores de refletância
    int s1_valor = digitalRead(s1);
    int s2_valor = digitalRead(s2);
    int s3_valor = digitalRead(s3);
    int s4_valor = digitalRead(s4);
    int s5_valor = digitalRead(s5);

    // Criar um número inteiro que represente a configuração dos sensores
    int configuracao = (s1_valor << 4) | (s2_valor << 3) | (s3_valor << 2) | (s4_valor << 1) | s5_valor;

    // Retornar a configuração
    return configuracao;
    Serial.println(configuracao);
}

// // Definindo o tipo de função para os manipuladores de motor
// typedef void (*MotorHandler)();

// // Array de configurações dos sensores
// int sensorConfigurations[] = {11011, 10011, 11001, 10111};

// // Array de funções correspondentes
// MotorHandler motorHandlers[] = {PID, motor_esquerdo, motor_direito, motor_esquerdo};

// void seguir_linha() {
//     int configuracao = leitura_sensor();

//     // Procurar a função de manipulação de motor correspondente à configuração atual
//     for (int i = 0; i < sizeof(sensorConfigurations) / sizeof(sensorConfigurations[0]); i++) {
//         if (sensorConfigurations[i] == configuracao) {
//             // Chamar a função de manipulação de motor encontrada
//             motorHandlers[i]();
//             return; // Encontrou a configuração, então retorna
//         }
//     }

//     // Implementar lógica para casos não especificados
// }

// void seguir_linha() {
//     // Chamar a função PID para calcular a saída do PID
//     PID();

//     // Usar a saída do PID para controlar os motores
//     if (saida_PID > 0) {
//         motor_direito();
//     } else if (saida_PID < 0) {
//         motor_esquerdo();
//     } else {
//         // Se a saída do PID for zero, você pode querer parar os motores ou manter a velocidade atual
//         // Implemente a lógica aqui conforme necessário
//     }
// }

void seguir_linha() {
    int configuracao = leitura_sensor();

    // Chamar a função de manipulação de motor correspondente diretamente
    if (configuracao == 11011 || configuracao == 11111 || configuracao == 00000 || configuracao == 00100) {
        motor_frente();
    }
    
    if (configuracao == 11001 || configuracao == 11101){
        motor_direito();
    }

    if (configuracao == 10011 || configuracao == 10111) {
        motor_esquerdo();
    }
}

void loop(){
    // Iniciar os sensores de COR
    digitalWrite(SensorDir_S0,HIGH); // Inicia o codigo com o pino S0 em nivel alto
    digitalWrite(SensorDir_S1,LOW); // Inicia o codigo com o pino S1 em nivel baixo

    digitalWrite(SensorEsq_S0,HIGH); // Inicia o codigo com o pino S0 em nivel alto
    digitalWrite(SensorEsq_S1,LOW); // Inicia o codigo com o pino S1 em nivel baixo

    // Iniciar motores como parados
    motor_parar();
    delay(100);

    // Seguir linha
    seguir_linha();
}
