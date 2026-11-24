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
// O divisor R1=100k/R2=47k (GPIO 34 = ADC1_CH6) foi montado em 19/11/2026.
// ATENCAO: o GPIO34 e' apenas entrada (sem pull-up interno). O divisor entrega
// no maximo ~2,5 V, dentro do fundo de escala de 3,3 V do ADC.
#define PIN_BATTERY_ADC    34   // -1 se desabilitado, ou GPIO (ex: 34, 35) se houver divisor
#define VBAT_HABILITADO    (PIN_BATTERY_ADC >= 0)
// Divisor de tensao: Vbat = Vadc * (R1 + R2) / R2
#define VBAT_R1_OHMS       100000.0f
#define VBAT_R2_OHMS        47000.0f
#define VBAT_DIVIDER       ((VBAT_R1_OHMS + VBAT_R2_OHMS) / VBAT_R2_OHMS)  // ~3,128
// Calibracao do divisor: com atenuacao de 11 dB o ADC do ESP32 NAO e' linear
// e satura bem antes de 2,5 V, entao a curva nominal (adc/4095 * 2,5) erra
// por um fator de ~96. Medido em 19/11/2026 com o divisor R1=100k/R2=47k:
//   bateria 8,22 V  ->  adc 3013   =>  ganho real ~0,00273 V/ADC
// O ganho e' derivado do divisor (V/ADC = Vadc_por_tick * (R1+R2)/R2), mas o
// fundo de escala real do ADC com 11 dB e' ~0,84 V, nao 2,5 V.
// Para recalibrar: meca a bateria no multimetro e envie "vbat <tensao_real>".
#define VBAT_ADC_REF_VOLTS  0.002728f  // V por unidade de ADC (ganho real, medido)
#define VBAT_ADC_REF_ZERO   0.0f       // ADC em 0 V (offset)
#define VBAT_ADC_ATTEN     11      // atenuacao do ADC (11 dB)
#define VBAT_AMOSTRAS      16      // media para suavizar o ruido do ADC
#define VBAT_LIMIAR_ATENCAO 7.0f   // V: avisa no painel/telemetria
#define VBAT_LIMIAR_CORTE   6.6f   // V: 3,30 V/celula -> corta os motores
#define VBAT_LEITURA_MS     500     // intervalo de leitura da tensao

// ==========================================
// 2. PARÂMETROS MECÂNICOS E CINEMÁTICOS (CALIBRADOS)
// ==========================================
#define PPR_MOTOR          12.0f
#define REDUCAO_MOTOR      90.0f                // Redução Real Homologada 1:90
#define QUADRATURA_MULT    4.0f                 // Modo Quadratura 4x
// Derivado dos fatores acima (12 * 90 * 4 = 4320): a documentação aritmética
// e o valor não podem divergir.
#define PPR_EFETIVO        (PPR_MOTOR * REDUCAO_MOTOR * QUADRATURA_MULT)

#define DIAMETRO_RODA_MM   66.0f                // Diâmetro real calibrado da roda em mm (6.6 cm)
#define CIRCUNF_RODA_MM    (PI * DIAMETRO_RODA_MM) // ~207.345 mm por volta da roda
#define MM_POR_PULSO       (CIRCUNF_RODA_MM / PPR_EFETIVO) // ~0.04799656 mm por pulso (10417 ticks para 500 mm)
#define DIST_RODAS_MM      120.0f               // Bitola entre rodas em mm (12 cm)

// ==========================================
// 3. PARÂMETROS DO ESC (MCPWM Hardware 50Hz)
// ==========================================
// A frequência é fixa em 50 Hz: o MCPWM usa period_ticks = 20000 com timer de
// 1 MHz (20 ms). Não existe macro para ela.

// Limites em microssegundos (us)
#define ESC_NEUTRO_US      1500     // Ponto neutro (motor parado)
#define ESC_ARM_TEMPO_MS   3000     // Tempo segurando neutro no boot para armar o ESC
// Fator de carga da curva PWM x RPM: razao entre a velocidade sob carga (robo no solo)
// e a velocidade sem carga (medida pelo diag_motor_curve). Corrige o feedforward por velocidade.
// Medido no solo em 17/11/2026: com 0.80, pico de ~60 RPM (alvo 55).
#define CURVA_FATOR_CARGA 0.80f

// Limites seguros operacionais
#define ESC_MIN_US         1150     // Potência máxima permitida em ré
#define ESC_MAX_US         1850     // Potência máxima permitida para frente

// ==========================================
// 4. PARÂMETROS DO CONTROLADOR PID E SINCRONISMO
// ==========================================
#define PID_KP_M1          1.8f
#define PID_KI_M1          1.5f   // 55 RPM sob carga precisa de ~1740us; com Ki 0.6 o integral não alcançava esse teto
#define PID_KD_M1          0.02f

#define PID_KP_M2          1.8f
#define PID_KI_M2          1.5f   // idem M1
#define PID_KD_M2          0.02f

// Ganho de sincronismo cruzado entre rodas (Cross-Coupling Synchronization)
#define K_SYNC             0.15f   // Aumentado para 0.15 para reforçar o acoplamento contra a assimetria mecânica entre as rodas

// ==========================================
// 4.1 FUSÃO SENSORIAL E CONTROLE ANGULAR COM IMU (BMI160)
// ==========================================
#define IMU_KP_YAW_RETA    2.0f     // Reforçado de 1.2 para 2.0 para eliminar desvios angulares em retas com perturbação no solo
// Clampe da correção de sincronismo cruzado (µs) aplicada ao PWM de cada motor.
// Valor original era literal no loop; virou macro para ficar visível/configurável.
#define SYNC_CLAMP_US      50.0f
// Clampe proprio do pivo: la o sincronismo usa apenas os encoders (a IMU ja atua
// no criterio de parada e no perfil trapezoidal), e o valor calibrado em solo e'
// menor que o da reta. Mantido separado para nao alterar o comportamento validado.
#define SYNC_CLAMP_PIVO_US 35.0f
#define CONTROL_LOOP_MS    20       // Período de amostragem do loop de controle (20ms = 50Hz)

// ==========================================
// 5. PARÂMETROS DA TRAJETÓRIA E PERFIL DE VELOCIDADE
// ==========================================
#define QUADRADO_LADOS     4
#define LADO_QUADRADO_MM   500.0f   // Comprimento de cada lado em mm (50 cm)
#define ANGULO_CURVA_GRAUS 90.0f    // Ângulo de cada curva em graus
// Antecipação do corte do pivô (graus): corta o giro ANTES do alvo para
// acomodar a inércia de frenagem — o robô continua girando depois de os
// motores pararem.
//
// ATENCAO: calibrar pelo giro REAL no chão, nunca pelo yaw da IMU. O yaw
// registrado congela no instante do corte e por isso as calibragens antigas
// (0.2° -> 0.7°) mediam 90° na IMU enquanto o robô fazia ~95° de verdade.
//
// A inércia CRESCE quando se antecipa demais (o robô para mais rápido):
//   0.7° -> corte 89.3°, inercia ~5.7°, real ~95°
//   5.7° -> corte 84.3°, inercia 7.2°, real 91.5°   (melhor ponto)
//   6.8° -> corte 83.2°, inercia 9.0°, real 93.1°
// Por isso 5.7° é o valor vigente. Para baixar o resíduo, reduzir a
// VELOCIDADE_GIRO_RPM (menos inércia), não antecipar mais.
#define PIVO_ANTECIPACAO_GRAUS 5.7f

// Guarda mínima de ticks do pivô: só aceita o yaw da IMU depois que a
// odometria confirma movimento real (~9,6 mm de arco), evitando cortar por
// um pico de yaw antes de o robô sair do lugar.
#define MIN_TICKS_PIVO         200
// Rede de segurança do pivô: se a IMU falhar/travar, para por odometria
// quando as ticks atingem este fator do alvo (evita giro infinito).
#define PIVO_FALLBACK_FATOR    1.15f

#define VELOCIDADE_CRUZEIRO_RPM 55.0f // Alvo do controlador. Medido no solo: ~60 RPM pico (bateria 2S), ~190 mm/s
#define VELOCIDADE_MIN_RPM      20.0f // Velocidade mínima de rampa (para não estolar)
// Velocidade alvo do pivô. Reduzida de 35 para 25 RPM: a inércia de frenagem
// cresce com a velocidade, e era ela que fazia o giro passar de 90°.
#define VELOCIDADE_GIRO_RPM     25.0f
#define PAUSA_ENTRE_MOVIMENTOS_MS 800 // Pausa de estabilização entre movimentos

// Distâncias de aceleração e desaceleração para perfil trapezoidal (em mm)
#define RAMPA_ACEL_DIST_MM   50.0f  // Acelera de 20 a 55 RPM nos primeiros 50 mm (~0.25 s)
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
// Telemetria adaptativa: o loop de controle roda a 50 Hz (20 ms) e o envio
// WebSocket e bloqueante, então a taxa acompanha o estado do robo.
// Em movimento usa 20 Hz (resolucao para analise de pico/rampa/corte do pivo);
// em repouso, 2 Hz (so para o dashboard nao congelar).
#define TELEM_INTERVAL_MOV_MS   50          // 20 Hz durante RETA/PIVO
#define TELEM_INTERVAL_IDLE_MS  500         // 2 Hz em AGUARDO/CONCLUIDO

// ==========================================
// 8. ESP-NOW (Gateway ESP32 auxiliar)
// ==========================================
// Canal do enlace ESP-NOW: o robô NÃO fixa canal — ele usa o canal atual da
// interface Wi-Fi (peer registrado com canal 0), que é o canal do AP quando
// associado. Quem fixa canal é o GATEWAY (gateway_espnow.ino, ESPNOW_CHANNEL),
// que deve coincidir com o canal do AP. Registrar o peer com canal divergente
// gera "Peer channel is not equal to the home channel, send fail!".
// MAC do gateway (preencher apos gravar gateway_espnow.ino e ler o MAC
// impresso no Serial). Broadcast = envia para qualquer gateway no canal.
#define ESPNOW_GATEWAY_MAC      {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
// (Removido em 23/11/2026: ESPNOW_LINK_TIMEOUT_MS existia sem nenhum consumidor,
// enquanto o comentario prometia um watchdog de enlace que nunca foi
// implementado. O enlace e' tratado por falha de envio em EspNowLink::onSent.)

#endif // CONFIG_H
