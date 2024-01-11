const int velocidade = 200;
const int velocidadeLeve = velocidade * 0.8;

void andarFrente(int velocidade){
    // Move para a frente
    digitalWrite(PinIN1, HIGH);
    digitalWrite(PinIN2, LOW);
    digitalWrite(PinIN3, HIGH);
    digitalWrite(PinIN4, LOW);

    analogWrite(PinENA, velocidade);
    analogWrite(PinENB, velocidade);
}

void andarDireita(int velocidade){
    // Direciona os motores para Direita 
    digitalWrite(PinIN1, HIGH);
    digitalWrite(PinIN2, LOW);
    digitalWrite(PinIN3, LOW);
    digitalWrite(PinIN4, HIGH);

    analogWrite(PinENA, velocidade);
    analogWrite(PinENB, 0);
}

void andarEsquerda(int velocidade){
    // Direciona os motores para Esquerda
    digitalWrite(PinIN1, LOW);
    digitalWrite(PinIN2, HIGH);
    digitalWrite(PinIN3, HIGH);
    digitalWrite(PinIN4, LOW);

    analogWrite(PinENA, 0);
    analogWrite(PinENB, velocidade);
}

void andarTras(int velocidade){
    // Move para trás
    digitalWrite(PinIN1, LOW);
    digitalWrite(PinIN2, HIGH);
    digitalWrite(PinIN3, LOW);
    digitalWrite(PinIN4, HIGH);

    analogWrite(PinENA, velocidade);
    analogWrite(PinENB, velocidade);
}

void pararMotores(){
    // Move para trás
    digitalWrite(PinIN1, LOW);
    digitalWrite(PinIN2, LOW);
    digitalWrite(PinIN3, LOW);
    digitalWrite(PinIN4, LOW);

    analogWrite(PinENA, 0);
    analogWrite(PinENB, 0);
    
}
