#include <Arduino.h>
#include "config.h"
#include "motor_control_mcpwm.h"
#include "encoder_reader_pcnt.h"
#include "pid_controller.h"
#include "imu_bmi160.h"
#include "esp_task_wdt.h"

// =========================================================================
// FIRMWARE ROBO 2WD - VERSAO 3 (MCPWM + PCNT + IMU BMI160)
// Melhorias Implementadas:
//  1. Validacao estrita no setup() com aborto seguro em caso de falha de hardware.
//  2. Task Watchdog Timer do ESP32 ativo (WDT de 3s).
//  3. Perfil de velocidade trapezoidal (aceleracao e desaceleracao suaves).
//  4. Checagem de stall / desbalanceamento em curvas pivo (girar).
//  5. Maquina de estados nao-bloqueante no loop() com aguardo real de comando.
//  6. Integracao com IMU BMI160: correcao de rumo em retas e curvas pivo com
//     parada de alta precisao baseada no angulo real de Yaw.
// =========================================================================

MotorControlMCPWM motors;
EncoderReaderPCNT encoders;
IMU_BMI160 imu;

PIDController pid1(PID_KP_M1, PID_KI_M1, PID_KD_M1, ESC_MIN_US, ESC_MAX_US, ESC_NEUTRO_US);
PIDController pid2(PID_KP_M2, PID_KI_M2, PID_KD_M2, ESC_MIN_US, ESC_MAX_US, ESC_NEUTRO_US);

enum EstadoSistema {
    ESTADO_FALHA_HARDWARE,
    ESTADO_ARMANDO_ESC,
    ESTADO_AGUARDO_INICIO,
    ESTADO_EXECUTANDO_TRAJETORIA,
    ESTADO_CONCLUIDO,
    ESTADO_EMERGENCIA
};

EstadoSistema estadoAtual = ESTADO_ARMANDO_ESC;

// Estrutura de controle de movimento não-bloqueante
enum SubEstadoMovimento {
    SUB_INICIAR_SEGMENTO,
    SUB_EM_MOVIMENTO,
    SUB_PAUSA_ESTABILIZACAO
};

SubEstadoMovimento subEstado = SUB_INICIAR_SEGMENTO;

// Variáveis de controle de trajetória do quadrado
int g_ladoAtual = 1;
bool g_emLinhaReta = true; // true = reta, false = giro de 90°

uint32_t g_lastLoopTime = 0;
uint32_t g_lastPrintTime = 0;
uint32_t g_desbalStartTime = 0;
uint32_t g_pausaStartTime = 0;

int32_t g_targetTicks = 0;
float g_distanciaTotalMm = 0.0f;
float g_anguloTotalGraus = 0.0f;
int g_sentidoGiro = 1; // +1 = Direita (Horário)

// =========================================================================
// PERFIL TRAPEZOIDAL DE VELOCIDADE
// Calcula o RPM alvo em funcao do progresso para evitar patinagem / derrapagem
// =========================================================================
float calcularRPMTrapezoidalReta(int32_t currentTicks, int32_t targetTicks) {
    if (targetTicks <= 0) return VELOCIDADE_MIN_RPM;

    float distPercorridaMm = currentTicks * MM_POR_PULSO;
    float distRestanteMm = (targetTicks - currentTicks) * MM_POR_PULSO;

    float rpmAlvo = VELOCIDADE_CRUZEIRO_RPM;

    // Rampa de subida (Aceleração)
    if (distPercorridaMm < RAMPA_ACEL_DIST_MM) {
        float fatorAcel = distPercorridaMm / RAMPA_ACEL_DIST_MM;
        rpmAlvo = VELOCIDADE_MIN_RPM + (VELOCIDADE_CRUZEIRO_RPM - VELOCIDADE_MIN_RPM) * fatorAcel;
    }
    // Rampa de descida (Desaceleração)
    else if (distRestanteMm < RAMPA_DESACEL_DIST_MM) {
        float fatorDesacel = distRestanteMm / RAMPA_DESACEL_DIST_MM;
        if (fatorDesacel < 0.0f) fatorDesacel = 0.0f;
        rpmAlvo = VELOCIDADE_MIN_RPM + (VELOCIDADE_CRUZEIRO_RPM - VELOCIDADE_MIN_RPM) * fatorDesacel;
    }

    if (rpmAlvo < VELOCIDADE_MIN_RPM) rpmAlvo = VELOCIDADE_MIN_RPM;
    return rpmAlvo;
}

float calcularRPMTrapezoidalCurva(int32_t currentTicks, int32_t targetTicks) {
    if (targetTicks <= 0) return VELOCIDADE_MIN_RPM;

    float arcoPercorridoMm = currentTicks * MM_POR_PULSO;
    float grausPercorridos = (arcoPercorridoMm / (PI * DIST_RODAS_MM)) * 360.0f;
    float grausRestantes = ANGULO_CURVA_GRAUS - grausPercorridos;

    float rpmAlvo = VELOCIDADE_GIRO_RPM;

    if (grausPercorridos < RAMPA_CURVA_ACEL_GRAUS) {
        float fator = grausPercorridos / RAMPA_CURVA_ACEL_GRAUS;
        rpmAlvo = VELOCIDADE_MIN_RPM + (VELOCIDADE_GIRO_RPM - VELOCIDADE_MIN_RPM) * fator;
    } else if (grausRestantes < RAMPA_CURVA_DESACEL_GRAUS) {
        float fator = grausRestantes / RAMPA_CURVA_DESACEL_GRAUS;
        if (fator < 0.0f) fator = 0.0f;
        rpmAlvo = VELOCIDADE_MIN_RPM + (VELOCIDADE_GIRO_RPM - VELOCIDADE_MIN_RPM) * fator;
    }

    if (rpmAlvo < VELOCIDADE_MIN_RPM) rpmAlvo = VELOCIDADE_MIN_RPM;
    return rpmAlvo;
}

// =========================================================================
// MÁQUINA DE ESTADOS PRINCIPAL
// =========================================================================

void iniciarMovimentoReta(float distanciaMm) {
    g_distanciaTotalMm = distanciaMm;
    g_targetTicks = (int32_t)(distanciaMm / MM_POR_PULSO);
    g_emLinhaReta = true;
    encoders.reset();
    pid1.reset();
    pid2.reset();
    imu.resetYaw(); // Zera a orientação de referência para a reta
    g_desbalStartTime = 0;
    subEstado = SUB_EM_MOVIMENTO;
    Serial.printf("\n[V3-RETA] Lado %d de %d | Dist: %.1f mm | Alvo: %d ticks\n",
                  g_ladoAtual, QUADRADO_LADOS, distanciaMm, g_targetTicks);
}

void iniciarMovimentoCurva(float anguloGraus, int sentido) {
    g_anguloTotalGraus = anguloGraus;
    g_sentidoGiro = sentido;
    float arco_mm = (PI * DIST_RODAS_MM * (anguloGraus / 360.0f));
    g_targetTicks = (int32_t)(arco_mm / MM_POR_PULSO);
    g_emLinhaReta = false;
    encoders.reset();
    pid1.reset();
    pid2.reset();
    imu.resetYaw(); // Zera o ângulo para medir os 90° exatos via giroscópio
    g_desbalStartTime = 0;
    subEstado = SUB_EM_MOVIMENTO;
    Serial.printf("\n[V3-PIVO] Curva %d de %d | Angulo: %.1f deg (%s) | Alvo: %d ticks/roda\n",
                  g_ladoAtual, QUADRADO_LADOS, anguloGraus, (sentido > 0) ? "DIREITA" : "ESQUERDA", g_targetTicks);
}

void processarControleMovimento(uint32_t now) {
    float dt = (now - g_lastLoopTime) / 1000.0f;
    g_lastLoopTime = now;

    encoders.updateSpeed(dt);
    if (imu.isInitialized()) {
        imu.update(dt);
    }

    float rpm1 = encoders.getRPM1();
    float rpm2 = encoders.getRPM2();

    int32_t t1 = abs(encoders.getTicksM1());
    int32_t t2 = abs(encoders.getTicksM2());
    int32_t avgTicks = (t1 + t2) / 2;

    float yawAtual = imu.getYawAngleDeg();         // valor bruto com sinal
    float yawError = yawAtual;                     // erro relativo ao início do segmento (resetYaw zera o ângulo)
    float yawAbs   = fabsf(yawError);              // magnitude para critério de parada de curva

    // 1. Critério de Parada
    bool atingiuMeta = false;
    if (g_emLinhaReta) {
        // Na reta, o critério primário é a distância por encoders
        atingiuMeta = (avgTicks >= g_targetTicks);
    } else {
        // Na curva pivô, se a IMU estiver ativa, ela é a autoridade máxima do ângulo de 90°
        if (imu.isInitialized()) {
            // IMU é a autoridade de ângulo; ticks são fallback de segurança (+15%)
            atingiuMeta = (yawAbs >= (g_anguloTotalGraus - 2.0f)) || (avgTicks >= (int32_t)(g_targetTicks * 1.15f));
        } else {
            atingiuMeta = (avgTicks >= g_targetTicks);
        }
    }

    if (atingiuMeta) {
        motors.stopAll();
        subEstado = SUB_PAUSA_ESTABILIZACAO;
        g_pausaStartTime = now;
        Serial.printf("[V3-STOP] Segmento concluido. Ticks: (%d, %d) | Yaw IMU: %+.1f deg. Pausando %d ms...\n",
                      encoders.getTicksM1(), encoders.getTicksM2(), imu.getYawAngleDeg(), PAUSA_ENTRE_MOVIMENTOS_MS);
        return;
    }

    // 2. Checagem de Desbalanceamento / Roda Travada (Stall) em Ambos os Modos
    float diffRpm = abs(abs(rpm1) - abs(rpm2));
    float rpmRef = g_emLinhaReta ? VELOCIDADE_CRUZEIRO_RPM : VELOCIDADE_GIRO_RPM;

    if (avgTicks > (g_targetTicks / 10) && diffRpm > (rpmRef * (DESBAL_MAX_PORCENT / 100.0f))) {
        if (g_desbalStartTime == 0) g_desbalStartTime = now;
        if (now - g_desbalStartTime > DESBAL_TIMEOUT_MS) {
            Serial.printf("\n[EMERGENCIA] Desbalanceamento/Stall detectado! M1: %.1f RPM | M2: %.1f RPM | Diff: %.1f\n",
                          rpm1, rpm2, diffRpm);
            motors.stopAll();
            estadoAtual = ESTADO_EMERGENCIA;
            return;
        }
    } else {
        g_desbalStartTime = 0;
    }

    // 3. Cálculo de Velocidade via Perfil Trapezoidal
    float targetRpmM1 = 0.0f;
    float targetRpmM2 = 0.0f;

    if (g_emLinhaReta) {
        float rpmTrapezio = calcularRPMTrapezoidalReta(avgTicks, g_targetTicks);
        targetRpmM1 = rpmTrapezio;
        targetRpmM2 = rpmTrapezio;

        float out1 = pid1.compute(targetRpmM1, rpm1, dt);
        float out2 = pid2.compute(targetRpmM2, rpm2, dt);

        // Sincronismo Cruzado Híbrido: Encoders + Correção Giroscópica da IMU
        float sync_error = (float)(encoders.getTicksM1() - encoders.getTicksM2());
        float sync_corr = sync_error * K_SYNC;

        // Se a IMU estiver ativa, adiciona compensação de guinada (Yaw) para travar o rumo reto
        // yawError é sinalizado: positivo = desviou para esquerda, negativo = para direita
        if (imu.isInitialized()) {
            sync_corr += yawError * IMU_KP_YAW_RETA;
        }

        if (sync_corr > 35.0f) sync_corr = 35.0f;
        if (sync_corr < -35.0f) sync_corr = -35.0f;

        out1 -= sync_corr;
        out2 += sync_corr;

        motors.setMicrosecondsM1((uint32_t)out1);
        motors.setMicrosecondsM2((uint32_t)out2);

        if (now - g_lastPrintTime >= 250) {
            g_lastPrintTime = now;
            Serial.printf("[RETA] %d/%d ticks | Yaw: %+.1f deg | RPM Alvo: %.1f | Real: (%.1f, %.1f) | PWM: (%u, %u)\n",
                          avgTicks, g_targetTicks, imu.getYawAngleDeg(), rpmTrapezio, rpm1, rpm2, (uint32_t)out1, (uint32_t)out2);
        }
    } else {
        // Na curva pivô, o perfil trapezoidal desacelera conforme se aproxima dos 90° lidos pela IMU
        int32_t progressoTicks = avgTicks;
        if (imu.isInitialized() && g_anguloTotalGraus > 0.0f) {
            // Usa yawAbs (magnitude do erro relativo) para mapear o progresso da curva
            progressoTicks = (int32_t)((yawAbs / g_anguloTotalGraus) * (float)g_targetTicks);
        }

        float rpmTrapezio = calcularRPMTrapezoidalCurva(progressoTicks, g_targetTicks);
        targetRpmM1 = (g_sentidoGiro > 0) ?  rpmTrapezio : -rpmTrapezio;
        targetRpmM2 = (g_sentidoGiro > 0) ? -rpmTrapezio :  rpmTrapezio;

        float out1 = pid1.compute(targetRpmM1, rpm1, dt);
        float out2 = pid2.compute(targetRpmM2, rpm2, dt);

        // Sincronismo Cruzado em Pivô (apenas encoders):
        // A IMU já atua no pivô pelo critério de parada e pelo mapeamento do perfil trapezoidal.
        float sync_error_pivot = (float)(t1 - t2);
        float sync_corr = sync_error_pivot * K_SYNC;

        if (sync_corr > 35.0f) sync_corr = 35.0f;
        if (sync_corr < -35.0f) sync_corr = -35.0f;

        if (g_sentidoGiro > 0) {
            out1 -= sync_corr;
            out2 -= sync_corr;
        } else {
            out1 += sync_corr;
            out2 += sync_corr;
        }

        motors.setMicrosecondsM1((uint32_t)out1);
        motors.setMicrosecondsM2((uint32_t)out2);

        if (now - g_lastPrintTime >= 200) {
            g_lastPrintTime = now;
            Serial.printf("[PIVO] Yaw: %+5.1f/%4.1f deg | %d/%d ticks | RPM: (%.1f, %.1f) | PWM: (%u, %u)\n",
                          yawError, g_anguloTotalGraus, avgTicks, g_targetTicks, rpm1, rpm2, (uint32_t)out1, (uint32_t)out2);
        }
    }
}

// =========================================================================
// SETUP
// =========================================================================
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(1000);

    Serial.println("\n==============================================");
    Serial.println("  ROBO 2WD - ESP32 V3 (ROBUSTEZ & SEGURANCA)  ");
    Serial.println("==============================================");
    Serial.printf("Periféricos: MCPWM (1MHz) + PCNT (4x Hardware)\n");
    Serial.printf("PPR Efetivo: %.0f | Dist/Pulso: %.4f mm\n", PPR_EFETIVO, MM_POR_PULSO);
    Serial.printf("Perfil de Velocidade: Trapezoidal (Acel: %.0fmm, Desacel: %.0fmm)\n",
                  RAMPA_ACEL_DIST_MM, RAMPA_DESACEL_DIST_MM);
    Serial.println("----------------------------------------------");

    // 1. Inicializacao Estrita do Hardware com aborto seguro
    bool pcntOk = encoders.begin();
    bool mcpwmOk = motors.begin();

    if (!pcntOk || !mcpwmOk) {
        Serial.println("\n[ERRO CRITICO] Falha na inicializacao dos perifericos de hardware!");
        if (!pcntOk)  Serial.println(" -> Falha ao alocar unidades PCNT (Pulse Counter).");
        if (!mcpwmOk) Serial.println(" -> Falha ao configurar Timer/Geradores MCPWM.");
        Serial.println("[SISTEMA BLOQUEADO] Para seguranca, a operacao foi abortada.");
        estadoAtual = ESTADO_FALHA_HARDWARE;
        return; // Fica travado no loop() sem acionar motores
    }

    Serial.println("[OK] MCPWM e PCNT inicializados com sucesso.");

    // 2. Inicializacao e Calibracao da IMU Bosch BMI160
    if (imu.begin()) {
        Serial.println("[OK] IMU BMI160 detectada com sucesso (Chip ID 0xD1).");
        Serial.println("[IMU] Calibrando giroscopio estatico (mantenha o robo parado)...");
        imu.calibrateGyro(150);
        Serial.println("[OK] Calibracao de Giroscopio concluida!");
    } else {
        Serial.println("[AVISO] IMU BMI160 nao detectada no barramento I2C. Operando apenas com odometria PCNT.");
    }

    // 3. Procedimento de Armamento do ESC (antes de subscrever a task no WDT)
    Serial.printf("[STATUS] Arming ESC via MCPWM (%d ms em neutro 1500 us)...\n", ESC_ARM_TEMPO_MS);
    motors.stopAll();
    delay(ESC_ARM_TEMPO_MS);
    Serial.println("[STATUS] ESC Armado com sucesso!");

    // 3. Reconfiguracao e ativacao do Task Watchdog Timer (WDT)
    // No ESP-IDF v5, o TWDT ja vem ativo por padrao (5s). Usamos reconfigure para ajustar o timeout real.
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    esp_err_t wdt_err = esp_task_wdt_reconfigure(&wdt_config);
    if (wdt_err != ESP_OK) {
        // Fallback caso reconfigure falhe (ex: se TWDT nao estivesse inicializado)
        esp_task_wdt_init(&wdt_config);
    }
    esp_err_t add_err = esp_task_wdt_add(NULL); // Subscreve a loopTask agora que o boot terminou
    if (add_err == ESP_OK) {
        Serial.printf("[OK] Hardware Task Watchdog reconfigurado e ativo (%d segundos).\n", WDT_TIMEOUT_SECONDS);
    } else {
        Serial.printf("[AVISO] Nao foi possivel subscrever tarefa ao WDT (codigo: %d)\n", add_err);
    }
    Serial.println("\n=======================================================");
    Serial.println(" Envie 's' ou 'g' pelo Monitor Serial para iniciar...");
    Serial.println(" (Ou aguarde 5 segundos para início automático)");
    Serial.println("=======================================================\n");

    estadoAtual = ESTADO_AGUARDO_INICIO;
    g_pausaStartTime = millis();
}

// =========================================================================
// LOOP NÃO-BLOQUEANTE
// =========================================================================
void loop() {
    uint32_t now = millis();

    // Alimenta o Watchdog se o sistema estiver operando normalmente
    if (estadoAtual != ESTADO_FALHA_HARDWARE) {
        esp_task_wdt_reset();
    }

    switch (estadoAtual) {
        case ESTADO_FALHA_HARDWARE:
            motors.stopAll();
            delay(500);
            break;

        case ESTADO_EMERGENCIA:
            motors.stopAll();
            static uint32_t lastEmergAlert = 0;
            if (now - lastEmergAlert > 2000) {
                lastEmergAlert = now;
                Serial.println("[ALERTA] Robô em ESTADO_EMERGENCIA! Reinicie para operar.");
            }
            delay(50);
            break;

        case ESTADO_AGUARDO_INICIO:
            // Checa entrada via Serial
            if (Serial.available()) {
                char c = Serial.read();
                if (c == 's' || c == 'S' || c == 'g' || c == 'G' || c == '\n') {
                    Serial.println("\n>>> COMANDO RECEBIDO! INICIANDO TRAJETÓRIA DO QUADRADO <<<");
                    g_ladoAtual = 1;
                    subEstado = SUB_INICIAR_SEGMENTO;
                    g_emLinhaReta = true;
                    estadoAtual = ESTADO_EXECUTANDO_TRAJETORIA;
                }
            }
            // Timeout automático de 5 segundos
            if (now - g_pausaStartTime >= 5000) {
                Serial.println("\n>>> TIMEOUT DE 5s ATINGIDO! INICIANDO TRAJETÓRIA DO QUADRADO <<<");
                g_ladoAtual = 1;
                subEstado = SUB_INICIAR_SEGMENTO;
                g_emLinhaReta = true;
                estadoAtual = ESTADO_EXECUTANDO_TRAJETORIA;
            }
            break;

        case ESTADO_EXECUTANDO_TRAJETORIA:
            if (subEstado == SUB_INICIAR_SEGMENTO) {
                if (g_emLinhaReta) {
                    iniciarMovimentoReta(LADO_QUADRADO_MM);
                } else {
                    iniciarMovimentoCurva(ANGULO_CURVA_GRAUS, 1);
                }
                g_lastLoopTime = now;
            }
            else if (subEstado == SUB_EM_MOVIMENTO) {
                if (now - g_lastLoopTime >= CONTROL_LOOP_MS) {
                    processarControleMovimento(now);
                }
            }
            else if (subEstado == SUB_PAUSA_ESTABILIZACAO) {
                // Aguarda a inércia mecânica estabilizar
                if (now - g_pausaStartTime >= PAUSA_ENTRE_MOVIMENTOS_MS) {
                    if (g_emLinhaReta) {
                        // Concluiu a reta: inicia o giro de 90°
                        g_emLinhaReta = false;
                        subEstado = SUB_INICIAR_SEGMENTO;
                    } else {
                        // Concluiu o giro: avança para o próximo lado
                        g_ladoAtual++;
                        if (g_ladoAtual > QUADRADO_LADOS) {
                            estadoAtual = ESTADO_CONCLUIDO;
                            motors.stopAll();
                            Serial.println("\n==============================================");
                            Serial.println("  TRAJETORIA DO QUADRADO CONCLUIDA COM SUCESSO! ");
                            Serial.println("==============================================\n");
                        } else {
                            g_emLinhaReta = true;
                            subEstado = SUB_INICIAR_SEGMENTO;
                        }
                    }
                }
            }
            break;

        case ESTADO_CONCLUIDO:
            motors.stopAll();
            static uint32_t lastFinishMsg = 0;
            if (now - lastFinishMsg > 5000) {
                lastFinishMsg = now;
                Serial.println("[STATUS] Execução finalizada com perfeição. Envie 's' para repetir.");
            }
            if (Serial.available()) {
                char c = Serial.read();
                if (c == 's' || c == 'S' || c == 'g' || c == 'G') {
                    g_ladoAtual = 1;
                    g_emLinhaReta = true;
                    subEstado = SUB_INICIAR_SEGMENTO;
                    estadoAtual = ESTADO_EXECUTANDO_TRAJETORIA;
                }
            }
            delay(10);
            break;
    }
}
