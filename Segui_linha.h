// vars
int valorSensorExtremaEsquerda = analogRead(sensorExtremaEsquerda);
int valorSensorEsquerdo = analogRead(sensorEsquerda);
int valorSensorDireito = analogRead(sensorDireita);
int valorSensorExtremaDireita = analogRead(sensorExtremaDireita);

int valorSensorEsquerdoVirar = analogRead(SensorEsquerdoVirar);
int valorSensorDireitoVirar = analogRead(SensorDireitoVirar);

// Valor de identificação da cor preta
const int linhaPreta = 900;

void serialReflatancia() {
  Serial.println("------------------");

  Serial.print("Sensor 0:");
  Serial.println(valorSensorEsquerdoVirar);

  Serial.print("Sensor 1:");
  Serial.println(valorSensorExtremaEsquerda);

  Serial.print("Sensor 2:");
  Serial.println(valorSensorEsquerdo);

  Serial.print("Sensor 3:");
  Serial.println(valorSensorDireito);

  Serial.print("Sensor 4:");
  Serial.println(valorSensorExtremaDireita);

  Serial.print("Sensor 5:");
  Serial.println(valorSensorDireitoVirar);
}

void linhaCurva() {
  if (valorSensorEsquerdoVirar < linhaPreta && valorSensorDireitoVirar > linhaPreta) {
    if (valorSensorEsquerdo > linhaPreta && valorSensorDireito > linhaPreta) {
      pararMotores();
      if (valorSensorExtremaEsquerda < linhaPreta && valorSensorExtremaDireita < linhaPreta) {
        return;
      } else {
        andarEsquerda();
      }
    } else if (valorSensorEsquerdo > linhaPreta && valorSensorDireito < linhaPreta) {
      andarDireita(velocidade);
    } else if (valorSensorEsquerdo < linhaPreta && valorSensorDireito > linhaPreta) {
      andarEsquerda(velocidade);
    } else {
      return;
    }
  } else if (valorSensorEsquerdoVirar > linhaPreta && valorSensorDireitoVirar < linhaPreta) {
    if (valorSensorEsquerdo > linhaPreta && valorSensorDireito > linhaPreta) {
      pararMotores();
      if (valorSensorExtremaEsquerda < linhaPreta && valorSensorExtremaDireita < linhaPreta) {
        return;
      } else {
        andarDireita();
      }
    } else if (valorSensorEsquerdo > linhaPreta && valorSensorDireito < linhaPreta) {
      andarDireita(velocidade);
    } else if (valorSensorEsquerdo < linhaPreta && valorSensorDireito > linhaPreta) {
      andarEsquerda(velocidade);
    } else {
      return;
    }
  }
}

void linhaEncruzilhada() {
  if (valorSensorEsquerdoVirar > linhaPreta && valorSensorDireitoVirar > linhaPreta) {
    if (valorSensorEsquerdo > linhaPreta && valorSensorDireito > linhaPreta) {
      andarFrente(velocidade);
    } else if (valorSensorEsquerdo > linhaPreta && valorSensorDireito < linhaPreta) {
      andarDireita(velocidade);
    } else if (valorSensorEsquerdo < linhaPreta && valorSensorDireito > linhaPreta) {
      andarEsquerda(velocidade);
    } else {
      return;
    }
  }
}

void seguirLinha() {
  if (valorSensorExtremaEsquerda < linhaPreta && valorSensorEsquerda < linhaPreta && valorSensorDireita < linhaPreta && valorSensorExtremaDireita < linhaPreta) {
    andarFrente(velocidade);
  } else if (valorSensorEsquerdo > linhaPreta && valorSensorDireito > linhaPreta) {
    linhaCurva();
    linhaEncruzilhada();
    andarFrente(velocidade);
  } else if (valorSensorEsquerdo < linhaPreta && valorSensorDireito > linhaPreta) {
    linhaCurva();
    linhaEncruzilhada();
    andarEsquerda(velocidadeLeve);
  } else if (valorSensorEsquerdo > linhaPreta && valorSensorDireito < linhaPreta) {
    linhaCurva();
    linhaEncruzilhada();
    andarDireita(velocidadeLeve);
  } else if (valorSensorExtremaEsquerda > linhaPreta && valorSensorEsquerda > linhaPreta && valorSensorDireita < linhaPreta && valorSensorExtremaDireita < linhaPreta) {
    linhaCurva();
    linhaEncruzilhada();
    andarDireita(velocidadeLeve);
  } else if (valorSensorExtremaEsquerda < linhaPreta && valorSensorEsquerda < linhaPreta && valorSensorDireita > linhaPreta && valorSensorExtremaDireita > linhaPreta) {
    linhaCurva();
    linhaEncruzilhada();
    andarEsquerda(velocidadeLeve);
  } else {
    linhaCurva();
    linhaEncruzilhada();
    andarFrente(velocidade);
  }
}
