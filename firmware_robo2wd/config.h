#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// 1. MAPEAMENTO DE PINOS (GPIOs)
// ==========================================
// ESC - Controle dos Motores DC (Sinais PWM)
#define PIN_ESC_CH1        18   // Motor 1 (Esquerdo)
#define PIN_ESC_CH2        19   // Motor 2 (Direito)

// Encoder 1 (Motor Esquerdo) - Leitura em Quadratura 4x
#define PIN_ENC1_A         23   // Sinal A
#define PIN_ENC1_B         22   // Sinal B

// Encoder 2 (Motor Direito) - Leitura em Quadratura 4x
#define PIN_ENC2_A         33   // Sinal A
#define PIN_ENC2_B         32   // Sinal B

// ==========================================
// 2. PARÂMETROS MECÂNICOS E CINEMÁTICOS (CALIBRADOS)
// ==========================================
// Teste experimental: 1 rotação manual de 360° = ~2180 a 2190 pulsos (quadratura 4x)
// Relação real do motor: 12 PPR * 45 (redução 1:45) * 4 (quadratura 4x) = 2160 pulsos
#define PPR_MOTOR          12.0f
#define REDUCAO_MOTOR      45.0f                // Calibrado empiricamente (~2180 pulsos / 48)
#define QUADRATURA_MULT    4.0f                 // Modo Quadratura 4x
#define PPR_EFETIVO        2160.0f              // 2180 pulsos por volta completa da roda

#define DIAMETRO_RODA_MM   70.0f                // Diâmetro da roda em mm (7 cm)
#define CIRCUNF_RODA_MM    (PI * DIAMETRO_RODA_MM) // ~219.911 mm por volta da roda
#define MM_POR_PULSO       (CIRCUNF_RODA_MM / PPR_EFETIVO) // ~0.10087 mm por pulso
#define DIST_RODAS_MM      120.0f               // Distância entre eixos das rodas em mm (12 cm)

// ==========================================
// 3. PARÂMETROS DO ESC (Sinal Servo / PWM)
// ==========================================
#define ESC_PWM_FREQ       50       // Frequência do PWM em Hz (período de 20ms)
#define ESC_PWM_RES_BITS   16       // Resolução do canal LEDC em bits (0 a 65535)

// Limites em microssegundos (us)
#define ESC_NEUTRO_US      1500     // Ponto neutro (motor parado)
#define ESC_ARM_TEMPO_MS   3000     // Tempo segurando neutro no boot para armar o ESC

// Limites seguros operacionais
#define ESC_MIN_US         1150     // Potência máxima permitida em ré
#define ESC_MAX_US         1850     // Potência máxima permitida para frente

// ==========================================
// 4. PARÂMETROS DO CONTROLADOR PID
// ==========================================
// Ganhos PID para velocidade (RPM)
#define PID_KP_M1          2.2f
#define PID_KI_M1          1.0f
#define PID_KD_M1          0.04f

#define PID_KP_M2          2.2f
#define PID_KI_M2          1.0f
#define PID_KD_M2          0.04f

#define CONTROL_LOOP_MS    20       // Período de amostragem do loop de controle (20ms = 50Hz)

// ==========================================
// 5. PARÂMETROS DA TRAJETÓRIA DE TESTE (QUADRADO)
// ==========================================
#define QUADRADO_LADOS     4
#define LADO_QUADRADO_MM   500.0f   // Comprimento de cada lado em mm (50 cm)
#define ANGULO_CURVA_GRAUS 90.0f    // Ângulo de cada curva em graus

#define VELOCIDADE_CRUZEIRO_RPM 35.0f // Velocidade alvo para avanço em linha reta (~40% max)
#define VELOCIDADE_GIRO_RPM     20.0f // Velocidade alvo para curvas em pivô central
#define PAUSA_ENTRE_MOVIMENTOS_MS 800 // Pausa de estabilização entre movimentos

// ==========================================
// 6. REGRAS DE SEGURANÇA E TELEMETRIA
// ==========================================
#define SERIAL_BAUD_RATE   115200
#define DESBAL_MAX_PORCENT 35.0f    // Diferença máxima tolerada entre motores em reta
#define DESBAL_TIMEOUT_MS  600      // Tempo máximo de desbalanceamento antes de parar

#endif // CONFIG_H
