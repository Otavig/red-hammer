// Variaveis que armazenam o valor das cores
int verdeEsquerdo = 0;
int verdeDireito = 0;

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