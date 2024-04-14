// Velocidade do rôbo
int speed = 200;

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