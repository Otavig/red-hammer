// Bibliotecas
#include <Arduino.h>
#include <Wire.h>
#include "Pins.h"

#include "Arenas.h"


// Definições
// Pins Motores DC 6V-3V - Ponte H
#define PinIN1 2
#define PinIN2 4
#define PinIN3 6
#define PinIN4 7

#define PinENA 3
#define PinENB 5


// Pins Reflatancia
#define sensorExtremaEsquerda A2 // Extremidade Esquerda
#define sensorEsquerdo A3 // Esquerda
#define sensorDireito A4 // Direita
#define sensorExtremaDireita A5 // Extremidade Direita

#define SensorEsquerdoVirar A1
#define SensorDireitoVirar A6

// Instruções aqui
void loop() {
  seguirLinha();
}