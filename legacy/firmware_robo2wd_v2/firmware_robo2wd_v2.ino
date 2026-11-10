#include <Arduino.h>
#include "config.h"
#include "motor_control_mcpwm.h"
#include "encoder_reader_pcnt.h"
#include "pid_controller.h"

// Instâncias de controle usando periféricos dedicados de hardware do ESP32
MotorControlMCPWM motors;
EncoderReaderPCNT encoders;

PIDController pid1(PID_KP_M1, PID_KI_M1, PID_KD_M1, ESC_MIN_US, ESC_MAX_US, ESC_NEUTRO_US);
PIDController pid2(PID_KP_M2, PID_KI_M2, PID_KD_M2, ESC_MIN_US, ESC_MAX_US, ESC_NEUTRO_US);

enum EstadoTrajetoria {
    ESTADO_ARMANDO,
    ESTADO_AGUARDO_INICIO,
    ESTADO_EXECUTANDO_QUADRADO,
    ESTADO_CONCLUIDO,
    ESTADO_EMERGENCIA
};

EstadoTrajetoria estadoAtual = ESTADO_ARMANDO;

// Executa um segmento de linha reta usando odometria por hardware (PCNT) e controle PID
void moverFrente(float distancia_mm, float rpm_alvo) {
    Serial.printf("\n[MOV-HW] Iniciando avanco: %.1f mm (alvo: %.1f RPM)...\n", distancia_mm, rpm_alvo);
    
    encoders.reset();
    pid1.reset();
    pid2.reset();

    int32_t target_ticks = (int32_t)(distancia_mm / MM_POR_PULSO);
    Serial.printf("[MOV-HW] Target ticks calculados: %d (PCNT Hardware 4x)\n", target_ticks);

    uint32_t lastLoopTime = millis();
    uint32_t lastPrintTime = millis();
    uint32_t desbalStartTime = 0;

    while (true) {
        uint32_t now = millis();
        if (now - lastLoopTime >= CONTROL_LOOP_MS) {
            float dt = (now - lastLoopTime) / 1000.0f;
            lastLoopTime = now;

            encoders.updateSpeed(dt);
            float rpm1 = encoders.getRPM1();
            float rpm2 = encoders.getRPM2();

            int32_t t1 = encoders.getTicksM1();
            int32_t t2 = encoders.getTicksM2();
            int32_t avg_ticks = (abs(t1) + abs(t2)) / 2;

            // Critério de parada de distância
            if (avg_ticks >= target_ticks) {
                break;
            }

            // Verificação de segurança (desbalanceamento)
            float diff = abs(rpm1 - rpm2);
            if (avg_ticks > (target_ticks / 10) && diff > (rpm_alvo * (DESBAL_MAX_PORCENT / 100.0f))) {
                if (desbalStartTime == 0) desbalStartTime = now;
                if (now - desbalStartTime > DESBAL_TIMEOUT_MS) {
                    Serial.printf("[ERRO] Desbalanceamento detectado! M1: %.1f RPM, M2: %.1f RPM\n", rpm1, rpm2);
                    motors.stopAll();
                    estadoAtual = ESTADO_EMERGENCIA;
                    return;
                }
            } else {
                desbalStartTime = 0;
            }

            // Correção PID individual atuando no MCPWM
            float out1 = pid1.compute(rpm_alvo, rpm1, dt);
            float out2 = pid2.compute(rpm_alvo, rpm2, dt);

            // Acoplamento de Sincronismo Cruzado (Cross-Coupling):
            // e_sync > 0 se a roda esquerda (M1) estiver adiantada em relação à direita (M2)
            float sync_error = (float)(t1 - t2);
            float sync_correction = sync_error * K_SYNC;

            // Limita a correção de sincronismo a ±35 us para manter suavidade
            if (sync_correction > 35.0f) sync_correction = 35.0f;
            if (sync_correction < -35.0f) sync_correction = -35.0f;

            out1 -= sync_correction;
            out2 += sync_correction;

            // Limites do ESC
            if (out1 < ESC_MIN_US) out1 = ESC_MIN_US;
            if (out1 > ESC_MAX_US) out1 = ESC_MAX_US;
            if (out2 < ESC_MIN_US) out2 = ESC_MIN_US;
            if (out2 > ESC_MAX_US) out2 = ESC_MAX_US;

            motors.setMicrosecondsM1((uint32_t)out1);
            motors.setMicrosecondsM2((uint32_t)out2);

            // Telemetria periódica
            if (now - lastPrintTime >= 250) {
                lastPrintTime = now;
                Serial.printf("[FRENTE-HW] Progresso: %d/%d ticks (t1=%d, t2=%d) | RPM: (%.1f, %.1f) | PWM: (%u, %u) us\n",
                              avg_ticks, target_ticks, t1, t2,
                              rpm1, rpm2, (uint32_t)out1, (uint32_t)out2);
            }
        }
        delay(1);
    }

    motors.stopAll();
    delay(PAUSA_ENTRE_MOVIMENTOS_MS);
    Serial.printf("[FRENTE CONCLUIDO] Ticks M1: %d | Ticks M2: %d | Dist: %.1f mm\n",
                  encoders.getTicksM1(), encoders.getTicksM2(), encoders.getDistanceM1_mm());
}

// Executa uma curva pivô sobre o próprio eixo
// sentido: +1 = Horário (direita), -1 = Anti-horário (esquerda)
void girar(float angulo_graus, int sentido, float rpm_alvo) {
    float arco_mm = (PI * DIST_RODAS_MM * (angulo_graus / 360.0f));
    int32_t target_ticks = (int32_t)(arco_mm / MM_POR_PULSO);

    Serial.printf("\n[GIRO-HW] %.1f graus (%s) | Arco: %.1f mm | Target: %d ticks/roda (PCNT Hardware 4x)\n",
                  angulo_graus, (sentido > 0) ? "DIREITA" : "ESQUERDA", arco_mm, target_ticks);

    encoders.reset();
    pid1.reset();
    pid2.reset();

    float target_rpm1 = (sentido > 0) ?  rpm_alvo : -rpm_alvo;
    float target_rpm2 = (sentido > 0) ? -rpm_alvo :  rpm_alvo;

    uint32_t lastLoopTime = millis();
    uint32_t lastPrintTime = millis();

    while (true) {
        uint32_t now = millis();
        if (now - lastLoopTime >= CONTROL_LOOP_MS) {
            float dt = (now - lastLoopTime) / 1000.0f;
            lastLoopTime = now;

            encoders.updateSpeed(dt);
            float rpm1 = encoders.getRPM1();
            float rpm2 = encoders.getRPM2();

            int32_t t1 = abs(encoders.getTicksM1());
            int32_t t2 = abs(encoders.getTicksM2());
            int32_t avg_ticks = (t1 + t2) / 2;

            if (avg_ticks >= target_ticks) {
                break;
            }

            float out1 = pid1.compute(target_rpm1, rpm1, dt);
            float out2 = pid2.compute(target_rpm2, rpm2, dt);

            // Sincronismo Cruzado em Pivô (Garante que ambas as rodas percorram rigorosamente o mesmo arco):
            // Se t1 > t2, a roda esquerda avançou mais no giro que a direita
            float sync_error_pivot = (float)(t1 - t2);
            float sync_correction = sync_error_pivot * K_SYNC;

            // Limite de segurança na correção
            if (sync_correction > 35.0f) sync_correction = 35.0f;
            if (sync_correction < -35.0f) sync_correction = -35.0f;

            if (sentido > 0) {
                // Sentido horário: M1 avança (+), M2 retrocede (-)
                out1 -= sync_correction;
                out2 -= sync_correction; // Reduz a magnitude da ré em M2 para equilibrar
            } else {
                // Sentido anti-horário: M1 retrocede (-), M2 avança (+)
                out1 += sync_correction;
                out2 += sync_correction;
            }

            // Limites de segurança do ESC
            if (out1 < ESC_MIN_US) out1 = ESC_MIN_US;
            if (out1 > ESC_MAX_US) out1 = ESC_MAX_US;
            if (out2 < ESC_MIN_US) out2 = ESC_MIN_US;
            if (out2 > ESC_MAX_US) out2 = ESC_MAX_US;

            motors.setMicrosecondsM1((uint32_t)out1);
            motors.setMicrosecondsM2((uint32_t)out2);

            if (now - lastPrintTime >= 200) {
                lastPrintTime = now;
                Serial.printf("[GIRO-HW] avg=%d/%d | t1=%d t2=%d | RPM:(%.1f,%.1f) | PWM:(%u,%u)\n",
                              avg_ticks, target_ticks, encoders.getTicksM1(), encoders.getTicksM2(),
                              rpm1, rpm2, (uint32_t)out1, (uint32_t)out2);
            }
        }
        delay(1);
    }

    motors.stopAll();
    delay(PAUSA_ENTRE_MOVIMENTOS_MS);
    int32_t final_t1 = encoders.getTicksM1();
    int32_t final_t2 = encoders.getTicksM2();
    float angulo_real = (((float)(abs(final_t1) + abs(final_t2))) / 2.0f * MM_POR_PULSO) / (PI * DIST_RODAS_MM / 360.0f);
    Serial.printf("[GIRO CONCLUIDO] Ticks brutos: (%d, %d) | Angulo estimado: %.1f graus\n",
                  final_t1, final_t2, angulo_real);
}

void executarQuadrado() {
    Serial.println("\n==============================================");
    Serial.println("  INICIANDO TRAJETORIA DO QUADRADO (4 LADOS) ");
    Serial.println("==============================================");

    uint32_t startTime = millis();

    for (int lado = 1; lado <= QUADRADO_LADOS; lado++) {
        Serial.printf("\n>>> LADO %d DE %d <<<\n", lado, QUADRADO_LADOS);
        moverFrente(LADO_QUADRADO_MM, VELOCIDADE_CRUZEIRO_RPM);

        if (estadoAtual == ESTADO_EMERGENCIA) {
            Serial.println("[ABORTADO] Trajetoria cancelada por seguranca.");
            return;
        }

        if (lado <= QUADRADO_LADOS) {
            Serial.printf(">>> CURVA %d DE %d (90 GRAUS DIREITA) <<<\n", lado, QUADRADO_LADOS);
            girar(ANGULO_CURVA_GRAUS, 1, VELOCIDADE_GIRO_RPM);

            if (estadoAtual == ESTADO_EMERGENCIA) {
                Serial.println("[ABORTADO] Trajetoria cancelada por seguranca.");
                return;
            }
        }
    }

    uint32_t totalTime = millis() - startTime;
    estadoAtual = ESTADO_CONCLUIDO;

    Serial.println("\n==============================================");
    Serial.println("         TRAJETORIA CONCLUIDA COM SUCESSO!    ");
    Serial.printf("[ROBO_FIN] quadrado=OK tempo_total_ms=%u\n", totalTime);
    Serial.println("==============================================\n");
}

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(1000);

    Serial.println("\n==============================================");
    Serial.println("  ROBO 2WD - ESP32 V2 (MCPWM + PCNT HARDWARE) ");
    Serial.println("==============================================");
    Serial.printf("Modo: Hardware PCNT 4x | PPR Efetivo: %.0f | Dist/Pulso: %.4f mm\n", PPR_EFETIVO, MM_POR_PULSO);
    Serial.printf("Circunferencia Roda: %.2f mm | Bitola (Track): %.2f mm\n", CIRCUNF_RODA_MM, DIST_RODAS_MM);
    Serial.println("----------------------------------------------");
    Serial.println("[STATUS] Inicializando Hardware MCPWM e PCNT...");

    if (!encoders.begin()) {
        Serial.println("[FALHA] Erro ao inicializar unidades PCNT!");
    }
    if (!motors.begin()) {
        Serial.println("[FALHA] Erro ao inicializar gerador MCPWM!");
    }

    Serial.printf("[STATUS] Arming ESC via MCPWM (%d ms de sinal neutro 1500 us)...\n", ESC_ARM_TEMPO_MS);
    motors.stopAll();
    delay(ESC_ARM_TEMPO_MS);

    Serial.println("[STATUS] ESC Armado! Aguardando 3 segundos antes do inicio...");
    for (int i = 3; i > 0; i--) {
        Serial.printf("Iniciando em %d...\n", i);
        delay(1000);
    }

    estadoAtual = ESTADO_EXECUTANDO_QUADRADO;
    executarQuadrado();
}

void loop() {
    motors.stopAll();
    delay(1000);
}
