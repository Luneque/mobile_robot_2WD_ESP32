#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// 1. MAPEAMENTO DE PINOS (GPIOs)
// ==========================================
// Barramento I2C Padrão (Liberado para IMU / Sensores)
#define PIN_I2C_SDA        21   // Linha de dados SDA
#define PIN_I2C_SCL        22   // Linha de clock SCL

// ESC - Controle dos Motores DC via MCPWM de Hardware
#define PIN_ESC_CH1        19   // Conectado ao ESC Canal 1 (Motor 1 - Esquerdo)
#define PIN_ESC_CH2        18   // Conectado ao ESC Canal 2 (Motor 2 - Direito)

// Encoder 1 (Motor Esquerdo) - Reorganizado para GPIO 16 e 17 (mesmo lado da placa)
#define PIN_ENC1_A         16   // Canal A (antigo GPIO 23)
#define PIN_ENC1_B         17   // Canal B (antigo GPIO 22)

// Encoder 2 (Motor Direito) - Leitura por Hardware PCNT
#define PIN_ENC2_A         33   // Canal A
#define PIN_ENC2_B         32   // Canal B

// Monitoramento de Bateria (Opcional - ADC1)
#define PIN_BATTERY_ADC    -1   // -1 se desabilitado, ou GPIO (ex: 34, 35) se houver divisor

// ==========================================
// 2. PARÂMETROS MECÂNICOS E CINEMÁTICOS (CALIBRADOS)
// ==========================================
#define PPR_MOTOR          12.0f
#define REDUCAO_MOTOR      90.0f                // Redução Real Homologada 1:90
#define QUADRATURA_MULT    4.0f                 // Modo Quadratura 4x
#define PPR_EFETIVO        4320.0f              // 4320 pulsos por volta completa da roda (12 * 90 * 4)

#define DIAMETRO_RODA_MM   66.0f                // Diâmetro real calibrado da roda em mm (6.6 cm)
#define CIRCUNF_RODA_MM    (PI * DIAMETRO_RODA_MM) // ~207.345 mm por volta da roda
#define MM_POR_PULSO       (CIRCUNF_RODA_MM / PPR_EFETIVO) // ~0.04799656 mm por pulso (10417 ticks para 500 mm)
#define DIST_RODAS_MM      120.0f               // Bitola entre rodas em mm (12 cm)

// ==========================================
// 3. PARÂMETROS DO ESC (MCPWM Hardware 50Hz)
// ==========================================
#define ESC_PWM_FREQ_HZ    50       // Frequência do PWM em Hz (período de 20ms)

// Limites em microssegundos (us)
#define ESC_NEUTRO_US      1500     // Ponto neutro (motor parado)
#define ESC_ARM_TEMPO_MS   3000     // Tempo segurando neutro no boot para armar o ESC
#define ESC_DEADBAND_US    110.0f   // Feedforward de compensação de zona morta do ESC (ajustado de 50 para 110us)

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
#define K_SYNC             0.08f

// ==========================================
// 4.1 FUSÃO SENSORIAL E CONTROLE ANGULAR COM IMU (BMI160)
// ==========================================
#define IMU_HABILITADA     true     // Habilita a correcao por giroscopio
#define IMU_KP_YAW_RETA    1.2f     // Ganho reforcado para manter rumo estavel em linha reta
#define CONTROL_LOOP_MS    20       // Período de amostragem do loop de controle (20ms = 50Hz)

// ==========================================
// 5. PARÂMETROS DA TRAJETÓRIA E PERFIL DE VELOCIDADE
// ==========================================
#define QUADRADO_LADOS     4
#define LADO_QUADRADO_MM   500.0f   // Comprimento de cada lado em mm (50 cm)
#define ANGULO_CURVA_GRAUS 90.0f    // Ângulo de cada curva em graus

#define VELOCIDADE_CRUZEIRO_RPM 55.0f // Velocidade de cruzeiro dinamica ideal (~190 mm/s alvo, ~40 RPM real com momento estavel)
#define VELOCIDADE_MIN_RPM      20.0f // Velocidade mínima de rampa (para não estolar)
#define VELOCIDADE_GIRO_RPM     35.0f // Velocidade alvo para curvas em pivô central suavizado (evita overshoot por inércia)
#define PAUSA_ENTRE_MOVIMENTOS_MS 800 // Pausa de estabilização entre movimentos

// Distâncias de aceleração e desaceleração para perfil trapezoidal (em mm)
#define RAMPA_ACEL_DIST_MM   50.0f  // Acelera de 20 a 80 RPM nos primeiros 50 mm (~0.25 s)
#define RAMPA_DESACEL_DIST_MM 60.0f // Desacelera nos últimos 60 mm para parada sem deslizamento (~0.30 s)

// Ângulos de aceleração e desaceleração para curva pivô (em graus)
#define RAMPA_CURVA_ACEL_GRAUS    10.0f // Acelera nos primeiros 10 graus
#define RAMPA_CURVA_DESACEL_GRAUS 15.0f // Desacelera nos ultimos 15 graus

// ==========================================
// 6. REGRAS DE SEGURANÇA, WATCHDOG E TELEMETRIA
// ==========================================
#define SERIAL_BAUD_RATE   115200
#define DESBAL_MAX_PORCENT 45.0f    // Limiar percentual de erro de velocidade entre rodas
#define DESBAL_TIMEOUT_MS  800      // Tempo máximo tolerado de desbalanceamento antes de parada

// Watchdog Timer por hardware
// V4: timeout aumentado para 5s pois a conexão Wi-Fi no boot pode demorar mais
#define WDT_TIMEOUT_SECONDS 5       // Se o loop travar por >5s, o ESP32 é reiniciado com segurança

// ==========================================
// 7. COMUNICAÇÃO SEM FIO (Wi-Fi + WebSocket)
// ==========================================
#define WIFI_MDNS_NAME          "robo"      // Acessível como robo.local
#define WIFI_HTTP_PORT          80          // Porta do servidor HTTP (dashboard)
#define WIFI_WS_PORT            81          // Porta do servidor WebSocket (telemetria)
#define WIFI_CONNECT_TIMEOUT_MS 10000       // Timeout de conexão Wi-Fi (10s)
#define TELEM_INTERVAL_MS       200         // Intervalo de envio de telemetria (5 Hz)

// ==========================================
// 8. ESP-NOW (Gateway ESP32-S3 Super Mini)
// ==========================================
// MAC do gateway ESP32-S3 (preencher após gravar gateway_espnow.ino e ler o MAC impresso no Serial)
// Enquanto não configurado, o envio ESP-NOW fica desabilitado.
#define ESPNOW_GATEWAY_MAC      {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}  // Broadcast (padrão)

#endif // CONFIG_H
