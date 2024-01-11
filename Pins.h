void setup() {
  Serial.begin(9600);

  // Instalação dos pinos motores
  pinMode(PinIN1, OUTPUT);
  pinMode(PinIN2, OUTPUT);
  pinMode(PinIN3, OUTPUT);
  pinMode(PinIN4, OUTPUT);

  pinMode(PinENA, OUTPUT);
  pinMode(PinENB, OUTPUT);
}