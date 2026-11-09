#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// 1. MAPEAMENTO DE PINOS (GPIOs)
// ==========================================
// ESC - Controle dos Motores DC via MCPWM de Hardware
// Mapeamento cruzado corrigido conforme comportamento do ESC/motores:
#define PIN_ESC_CH1        19   // Conectado ao ESC Canal 1 (Motor 1 - Esquerdo)
#define PIN_ESC_CH2        18   // Conectado ao ESC Canal 2 (Motor 2 - Direito)

// Encoder 1 (Motor Esquerdo) - Leitura por Hardware PCNT
#define PIN_ENC1_A         23   // Canal A
#define PIN_ENC1_B         22   // Canal B

// Encoder 2 (Motor Direito) - Leitura por Hardware PCNT
#define PIN_ENC2_A         33   // Canal A
#define PIN_ENC2_B         32   // Canal B

// ==========================================
// 2. PARÂMETROS MECÂNICOS E CINEMÁTICOS (CALIBRADOS)
// ==========================================
// Teste experimental confirmado: 1 rotação manual completa = ~2160 pulsos
#define PPR_MOTOR          12.0f
#define REDUCAO_MOTOR      90.0f                // 1:90
#define QUADRATURA_MULT    4.0f                 // Modo Quadratura 4x
#define PPR_EFETIVO        4320.0f              // 4320 pulsos por volta completa da roda (12 * 90 * 4)

#define DIAMETRO_RODA_MM   70.0f                // Diâmetro da roda em mm (7 cm)
#define CIRCUNF_RODA_MM    (PI * DIAMETRO_RODA_MM) // ~219.911 mm por volta da roda
#define MM_POR_PULSO       (CIRCUNF_RODA_MM / PPR_EFETIVO) // ~0.10181 mm por pulso
#define DIST_RODAS_MM      120.0f               // Distância entre eixos das rodas em mm (12 cm)

// ==========================================
// 3. PARÂMETROS DO ESC (MCPWM Hardware 50Hz)
// ==========================================
#define ESC_PWM_FREQ_HZ    50       // Frequência do PWM em Hz (período de 20ms)

// Limites em microssegundos (us)
#define ESC_NEUTRO_US      1500     // Ponto neutro (motor parado)
#define ESC_ARM_TEMPO_MS   3000     // Tempo segurando neutro no boot para armar o ESC
#define ESC_DEADBAND_US    50.0f    // Feedforward de compensação de zona morta do ESC

// Limites seguros operacionais
#define ESC_MIN_US         1150     // Potência máxima permitida em ré
#define ESC_MAX_US         1850     // Potência máxima permitida para frente

// ==========================================
// 4. PARÂMETROS DO CONTROLADOR PID E SINCRONISMO
// ==========================================
#define PID_KP_M1          1.8f
#define PID_KI_M1          0.6f
#define PID_KD_M1          0.02f

#define PID_KP_M2          1.8f
#define PID_KI_M2          0.6f
#define PID_KD_M2          0.02f

// Ganho de sincronismo cruzado entre rodas (Cross-Coupling Synchronization)
// Força ambas as rodas a andarem exatamente na mesma velocidade/distância em linha reta
#define K_SYNC             0.05f

#define CONTROL_LOOP_MS    20       // Período de amostragem do loop de controle (20ms = 50Hz)

// ==========================================
// 5. PARÂMETROS DA TRAJETÓRIA DE TESTE (QUADRADO)
// ==========================================
#define QUADRADO_LADOS     4
#define LADO_QUADRADO_MM   500.0f   // Comprimento de cada lado em mm (50 cm)
#define ANGULO_CURVA_GRAUS 90.0f    // Ângulo de cada curva em graus

#define VELOCIDADE_CRUZEIRO_RPM 65.0f // Velocidade alvo para avanço em linha reta (~238 mm/s, aumento de 44%)
#define VELOCIDADE_GIRO_RPM     40.0f // Velocidade alvo para curvas em pivô central (~1,4 s por giro de 90°)
#define PAUSA_ENTRE_MOVIMENTOS_MS 800 // Pausa de estabilização entre movimentos

// ==========================================
// 6. REGRAS DE SEGURANÇA E TELEMETRIA
// ==========================================
#define SERIAL_BAUD_RATE   115200
#define DESBAL_MAX_PORCENT 45.0f
#define DESBAL_TIMEOUT_MS  800

#endif // CONFIG_H
