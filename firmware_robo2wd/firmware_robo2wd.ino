#include <Arduino.h>
#include "config.h"
#include "motor_control.h"
#include "encoder_reader.h"
#include "pid_controller.h"

MotorControl motors;
EncoderReader encoders;

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

// Executa um segmento de linha reta usando odometria e PID de velocidade
void moverFrente(float distancia_mm, float rpm_alvo) {
    Serial.printf("\n[MOV] Iniciando avanco: %.1f mm (alvo: %.1f RPM)...\n", distancia_mm, rpm_alvo);
    
    encoders.reset();
    pid1.reset();
    pid2.reset();

    int32_t target_ticks = (int32_t)(distancia_mm / MM_POR_PULSO);
    Serial.printf("[MOV] Target ticks calculados: %d (Quadratura 4x)\n", target_ticks);

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

            // Verificação de desbalanceamento severo (Segurança)
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

            // Correção PID de velocidade para avanço
            float out1 = pid1.compute(rpm_alvo, rpm1, dt);
            float out2 = pid2.compute(rpm_alvo, rpm2, dt);

            motors.setMicrosecondsM1((uint32_t)out1);
            motors.setMicrosecondsM2((uint32_t)out2);

            // Telemetria periódica
            if (now - lastPrintTime >= 250) {
                lastPrintTime = now;
                Serial.printf("[FRENTE] Progresso: %d/%d ticks (t1=%d, t2=%d) | RPM: (%.1f, %.1f) | PWM: (%u, %u) us\n",
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

    Serial.printf("\n[GIRO] %.1f graus (%s) | Arco: %.1f mm | Target: %d ticks/roda (Quadratura 4x)\n",
                  angulo_graus, (sentido > 0) ? "DIREITA" : "ESQUERDA", arco_mm, target_ticks);

    encoders.reset();
    pid1.reset();
    pid2.reset();

    // No pivô: Motor 1 e Motor 2 giram em sentidos opostos
    // sentido +1 (direita): M1 frente (+rpm), M2 ré (-rpm)
    // sentido -1 (esquerda): M1 ré (-rpm), M2 frente (+rpm)
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

            motors.setMicrosecondsM1((uint32_t)out1);
            motors.setMicrosecondsM2((uint32_t)out2);

            if (now - lastPrintTime >= 200) {
                lastPrintTime = now;
                Serial.printf("[GIRO] avg=%d/%d | t1=%d t2=%d | RPM:(%.1f,%.1f) | PWM:(%u,%u)\n",
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
    Serial.println("  ROBO 2WD - ESP32 CONTROLADOR DE TRAJETORIA  ");
    Serial.println("==============================================");
    Serial.printf("Modo: Quadratura 4x | PPR Efetivo: %.0f | Distancia/Pulso: %.4f mm\n", PPR_EFETIVO, MM_POR_PULSO);
    Serial.printf("Circunferencia Roda: %.2f mm | Bitola (Track): %.2f mm\n", CIRCUNF_RODA_MM, DIST_RODAS_MM);
    Serial.println("----------------------------------------------");
    Serial.println("[STATUS] Inicializando Hardware e Encoders em 4x...");

    encoders.begin();
    motors.begin();

    Serial.printf("[STATUS] Arming ESC (%d ms de sinal neutro 1500 us)...\n", ESC_ARM_TEMPO_MS);
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
