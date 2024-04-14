#define white 1

// Definindo as constantes do PID
const float Kp = 2.0;
const float Ki = 0.5;
const float Kd = 1.0;

// Variáveis para armazenar os erros anteriores
float error_anterior = 0;
float integral = 0;
float derivativo = 0;

// Variável para armazenar a saída do PID
float saida_PID = 0;

// Variável para o valor desejado, que será usado para calcular o erro no PID
float valor_desejado = 0; // Defina o valor desejado aqui

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
}

// void PID() {
//     // Calcular o erro atual
//     float erro_atual = leitura_sensor() - valor_desejado; // Agora a função leitura_sensor() está sendo usada corretamente

//     // Calcular a integral
//     integral += erro_atual;

//     // Calcular o derivativo
//     derivativo = erro_atual - error_anterior;

//     // Calcular a saída do PID
//     saida_PID = Kp * erro_atual + Ki * integral + Kd * derivativo;

//     // Atualizar o erro anterior
//     error_anterior = erro_atual;

//     // Ajustar a velocidade dos motores com base na saída do PID
//     if (saida_PID > 0) {
//         motor_direito();
//     } else {
//         motor_esquerdo();
//     }
// }

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