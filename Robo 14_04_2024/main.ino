#include <Arduino.h>
#include "defines.h"
#include "motors.h"
#include "sensor_cor.h"
#include "reflatancia.h"

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
    pinMode(SensorDir_OUT, INPUT)
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