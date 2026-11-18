#include <Arduino.h>
#include "config.h"
#include "motor_control_mcpwm.h"
#include "encoder_reader_pcnt.h"
#include "pid_controller.h"
#include "robo_matematica.h"
#include "imu_bmi160.h"
#include "esp_task_wdt.h"
#include "wifi_telemetry.h"
#include "telemetry_recorder.h"
#include "espnow_link.h"
#include "secrets.h"

// =========================================================================
// FIRMWARE ROBO 2WD - VERSAO 4 (V3 + Wi-Fi + WebSocket Dashboard)
// Novidades V4:
//  7. Wi-Fi Station Mode com dashboard WebSocket em tempo real (robo.local).
//  8. Comandos de início/parada recebidos via WebSocket (sem cabo serial).
//  9. Telemetria JSON a 5 Hz: RPM, Yaw, ticks, PWM, estado da FSM.
// 10. Preparado para ESP-NOW (gateway ESP32-S3 como fallback sem Wi-Fi).
// =========================================================================

MotorControlMCPWM motors;
EncoderReaderPCNT encoders;
IMU_BMI160 imu;

PIDController pid1(PID_KP_M1, PID_KI_M1, PID_KD_M1, ESC_MIN_US, ESC_MAX_US, ESC_NEUTRO_US);
PIDController pid2(PID_KP_M2, PID_KI_M2, PID_KD_M2, ESC_MIN_US, ESC_MAX_US, ESC_NEUTRO_US);

WifiTelemetry wirelessTelem;
bool g_wifiOk = false;
uint32_t g_lastTelemTime = 0;

// Enlace ESP-NOW com o gateway ESP32-S3 (redundancia/alternativa ao Wi-Fi)
EspNowLink espnow;
bool g_espnowOk = false;
uint32_t g_lastEspnowTelem = 0;
uint32_t g_lastEspnowRx = 0;   // ultimo recebimento do gateway (watchdog de enlace)

// Gravador de telemetria em RAM (para testes sem cabo serial, ex.: robô no solo)
TelemetryRecorder recorder;

// Dump incremental via WebSocket / ESP-NOW (robô sem cabo)
static bool     _dumpParaWs = false;
static bool     _dumpParaEspNow = false;
static bool     _dumpCanalEspNow = false;
static uint16_t _dumpIdx = 0;
static uint16_t _dumpTotal = 0;
static uint32_t _dumpUltimoEnvio = 0;
static char     _dumpLinha[96];
#define DUMP_WS_INTERVAL_MS 50
#define DUMP_WS_POR_LOTE    10
// ESP-NOW: lote menor e intervalo maior (rajadas descartam pacotes no gateway)
#define DUMP_ESPNOW_INTERVAL_MS 100
#define DUMP_ESPNOW_POR_LOTE    1

uint32_t g_lastPwm1 = ESC_NEUTRO_US;
uint32_t g_lastPwm2 = ESC_NEUTRO_US;

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
uint32_t g_desbalStartTime = 0;
uint32_t g_pausaStartTime = 0;

int32_t g_targetTicks = 0;
float g_distanciaTotalMm = 0.0f;
float g_anguloTotalGraus = 0.0f;
int g_sentidoGiro = 1; // +1 = Direita (Horário)
float g_yawStartSeg = 0.0f; // Ângulo de yaw exato no instante em que o movimento inicia
float g_yawAbs = 0.0f;      // Rumo GLOBAL (graus, horário) acumulado entre segmentos
float g_yawPrevAmostra = 0.0f; // Yaw relativo da amostra anterior (para integrar o delta)

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
    g_targetTicks = ticksParaMm(distanciaMm);
    g_emLinhaReta = true;
    encoders.reset();
    pid1.reset();
    pid2.reset();
    imu.resetYaw(); // Zera a orientação de referência para a reta
    g_yawStartSeg = 0.0f;
    g_yawPrevAmostra = 0.0f; // o rumo global (g_yawAbs) é acumulado entre segmentos
    g_desbalStartTime = 0;
    subEstado = SUB_EM_MOVIMENTO;
    Serial.printf("\n[V4-RETA] Lado %d de %d | Dist: %.1f mm | Alvo: %d ticks\n",
                  g_ladoAtual, QUADRADO_LADOS, distanciaMm, g_targetTicks);
}

void iniciarMovimentoCurva(float anguloGraus, int sentido) {
    g_anguloTotalGraus = anguloGraus;
    g_sentidoGiro = sentido;
    g_targetTicks = ticksParaArcoPivo(anguloGraus);
    g_emLinhaReta = false;
    encoders.reset();
    pid1.reset();
    pid2.reset();
    imu.resetYaw(); // Zera o acumulador de yaw
    g_yawStartSeg = imu.getYawAngleDeg(); // Captura o valor exato no instante zero
    g_yawPrevAmostra = 0.0f;
    g_desbalStartTime = 0;
    subEstado = SUB_EM_MOVIMENTO;
    Serial.printf("\n[V4-PIVO] Curva %d de %d | Angulo: %.1f deg (%s) | Alvo: %d ticks/roda\n",
                  g_ladoAtual, QUADRADO_LADOS, anguloGraus, (sentido > 0) ? "DIREITA" : "ESQUERDA", g_targetTicks);
}

void processarControleMovimento(uint32_t now) {
    float dt = (now - g_lastLoopTime) / 1000.0f;
    g_lastLoopTime = now;

    encoders.updateSpeed(dt);
    if (imu.isInitialized()) {
        imu.update(dt);
    }

    // Integra o rumo global: o yaw da IMU e' relativo ao inicio do segmento
    // (resetYaw a cada segmento), entao soma-se o delta entre amostras.
    if (imu.isInitialized()) {
        float yawAmostra = imu.getYawAngleDeg();
        g_yawAbs += (yawAmostra - g_yawPrevAmostra);
        g_yawPrevAmostra = yawAmostra;
    }

    float rpm1 = encoders.getRPM1();
    float rpm2 = encoders.getRPM2();

    int32_t t1 = abs(encoders.getTicksM1());
    int32_t t2 = abs(encoders.getTicksM2());
    int32_t avgTicks = (t1 + t2) / 2;

    float yawAtual = imu.getYawAngleDeg();         // valor bruto com sinal
    float deltaYaw = yawAtual - g_yawStartSeg;      // ângulo EFETIVAMENTE girado desde o início deste segmento
    float yawError = deltaYaw;                      // erro relativo ao início do segmento
    float yawAbs   = fabsf(deltaYaw);               // magnitude real da rotação executada

    // 1. Critério de Parada (robo_matematica.h)
    bool atingiuMeta = g_emLinhaReta
        ? atingiuMetaReta(avgTicks, g_targetTicks)
        : atingiuMetaPivo(avgTicks, yawAbs, g_targetTicks, g_anguloTotalGraus,
                          imu.isInitialized());

    if (atingiuMeta) {
        motors.stopAll();
        g_lastPwm1 = ESC_NEUTRO_US;
        g_lastPwm2 = ESC_NEUTRO_US;
        subEstado = SUB_PAUSA_ESTABILIZACAO;
        g_pausaStartTime = now;
        Serial.printf("[V4-STOP] Segmento concluido. Ticks: (%d, %d) | Yaw IMU: %+.1f deg. Pausando %d ms...\n",
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
            g_lastPwm1 = ESC_NEUTRO_US;
            g_lastPwm2 = ESC_NEUTRO_US;
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

        if (sync_corr > SYNC_CLAMP_US) sync_corr = SYNC_CLAMP_US;
        if (sync_corr < -SYNC_CLAMP_US) sync_corr = -SYNC_CLAMP_US;

        out1 -= sync_corr;
        out2 += sync_corr;

        motors.setMicrosecondsM1((uint32_t)out1);
        motors.setMicrosecondsM2((uint32_t)out2);
        g_lastPwm1 = (uint32_t)out1;
        g_lastPwm2 = (uint32_t)out2;
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
        g_lastPwm1 = (uint32_t)out1;
        g_lastPwm2 = (uint32_t)out2;
    }
}

// =========================================================================
// TELEMETRIA SEM FIO (WEBSOCKET)
// =========================================================================
void enviarTelemetria(uint32_t now) {
    if (!g_wifiOk || !wirelessTelem.hasClients()) return;

    // Taxa adaptativa: alta durante o movimento (analise de pico/rampa/corte),
    // baixa em repouso (so para o dashboard nao congelar).
    bool emMovimento = (estadoAtual == ESTADO_EXECUTANDO_TRAJETORIA);
    uint32_t intervalo = emMovimento ? TELEM_INTERVAL_MOV_MS : TELEM_INTERVAL_IDLE_MS;
    if (now - g_lastTelemTime < intervalo) return;
    g_lastTelemTime = now;

    // Lê valores atuais dos sensores
    float rpm1 = encoders.getRPM1();
    float rpm2 = encoders.getRPM2();
    int32_t t1  = abs(encoders.getTicksM1());
    int32_t t2  = abs(encoders.getTicksM2());
    float yaw   = imu.isInitialized() ? imu.getYawAngleDeg() : 0.0f;

    // Determina estado como string
    const char* estadoStr = "AGUARDO";
    if (estadoAtual == ESTADO_EXECUTANDO_TRAJETORIA) estadoStr = g_emLinhaReta ? "RETA" : "PIVO";
    else if (estadoAtual == ESTADO_CONCLUIDO) {
        estadoStr = "CONCLUIDO";
        rpm1 = 0.0f; // Zera leitura residual do filtro EMA com motor parado
        rpm2 = 0.0f;
    }
    else if (estadoAtual == ESTADO_EMERGENCIA) {
        estadoStr = "EMERGENCIA";
        rpm1 = 0.0f;
        rpm2 = 0.0f;
    }
    else if (estadoAtual == ESTADO_FALHA_HARDWARE) {
        estadoStr = "FALHA";
        rpm1 = 0.0f;
        rpm2 = 0.0f;
    }
    else if (estadoAtual == ESTADO_ARMANDO_ESC) {
        estadoStr = "ARMANDO";
        rpm1 = 0.0f;
        rpm2 = 0.0f;
    }

    TelemetryPacket pkt;
    pkt.ts         = now;
    pkt.estado     = estadoStr;
    pkt.lado       = g_ladoAtual;
    pkt.totalLados = QUADRADO_LADOS;
    pkt.yaw        = yaw;
    pkt.yawAbs     = g_yawAbs;   // rumo global integrado (horário = positivo)
    pkt.rpm1       = rpm1;
    pkt.rpm2       = rpm2;
    pkt.ticks1     = t1;
    pkt.ticks2     = t2;
    pkt.ticksAlvo  = g_targetTicks;
    pkt.pwm1       = g_lastPwm1;
    pkt.pwm2       = g_lastPwm2;
    pkt.rpmAlvo    = g_emLinhaReta ? VELOCIDADE_CRUZEIRO_RPM : VELOCIDADE_GIRO_RPM;

    wirelessTelem.sendTelemetry(pkt);
}

// =========================================================================
// TELEMETRIA ESP-NOW (redundancia/alternativa ao Wi-Fi)
// =========================================================================
// Serializa o mesmo pacote em JSON e envia ao gateway ESP32-S3.
void enviarTelemEspNow(uint32_t now) {
    if (!g_espnowOk) return;
    // Mesma taxa adaptativa da telemetria WebSocket (20 Hz em movimento,
    // 2 Hz em repouso).
    bool emMovimento = (estadoAtual == ESTADO_EXECUTANDO_TRAJETORIA);
    uint32_t intervalo = emMovimento ? TELEM_INTERVAL_MOV_MS : TELEM_INTERVAL_IDLE_MS;
    if (now - g_lastEspnowTelem < intervalo) return;
    g_lastEspnowTelem = now;

    float rpm1 = encoders.getRPM1();
    float rpm2 = encoders.getRPM2();
    int32_t t1  = abs(encoders.getTicksM1());
    int32_t t2  = abs(encoders.getTicksM2());
    float yaw   = imu.isInitialized() ? imu.getYawAngleDeg() : 0.0f;

    const char* estadoStr = "AGUARDO";
    if (estadoAtual == ESTADO_EXECUTANDO_TRAJETORIA) estadoStr = g_emLinhaReta ? "RETA" : "PIVO";
    else if (estadoAtual == ESTADO_CONCLUIDO)   estadoStr = "CONCLUIDO";
    else if (estadoAtual == ESTADO_EMERGENCIA)  estadoStr = "EMERGENCIA";
    else if (estadoAtual == ESTADO_FALHA_HARDWARE) estadoStr = "FALHA";
    else if (estadoAtual == ESTADO_ARMANDO_ESC) estadoStr = "ARMANDO";

    // JSON da telemetria ESP-NOW (~185 bytes no pior caso)
    char buf[200];
    int n = snprintf(buf, sizeof(buf),
        "{\"ts\":%lu,\"st\":\"%s\",\"sd\":%d,\"of\":%d,\"yaw\":%.2f,\"yawAbs\":%.2f,"
        "\"r1\":%.1f,\"r2\":%.1f,\"t1\":%ld,\"t2\":%ld,\"tg\":%ld,"
        "\"w1\":%u,\"w2\":%u,\"rt\":%.1f}",
        (unsigned long)now, estadoStr, g_ladoAtual, QUADRADO_LADOS, yaw, g_yawAbs,
        rpm1, rpm2, (long)t1, (long)t2, (long)g_targetTicks,
        (unsigned)g_lastPwm1, (unsigned)g_lastPwm2,
        g_emLinhaReta ? VELOCIDADE_CRUZEIRO_RPM : VELOCIDADE_GIRO_RPM);

    if (n > 0 && n < (int)sizeof(buf)) espnow.sendTelemetry(buf);
}

// =========================================================================
// GRAVADOR DE TELEMETRIA EM RAM (replay para testes sem cabo serial)
// =========================================================================
uint8_t codigoEstadoRecorder() {
    switch (estadoAtual) {
        case ESTADO_FALHA_HARDWARE:     return REC_EST_FALHA;
        case ESTADO_ARMANDO_ESC:        return REC_EST_ARMANDO;
        case ESTADO_AGUARDO_INICIO:     return REC_EST_AGUARDO;
        case ESTADO_EXECUTANDO_TRAJETORIA: return g_emLinhaReta ? REC_EST_RETA : REC_EST_PIVO;
        case ESTADO_CONCLUIDO:          return REC_EST_CONCLUIDO;
        case ESTADO_EMERGENCIA:         return REC_EST_EMERGENCIA;
        default:                        return REC_EST_AGUARDO;
    }
}

void gravarAmostraRecorder(uint32_t now) {
    // Não grava o estado CONCLUIDO/EMERGENCIA repetido: sem isso o buffer
    // circular (100 s) enche de amostras idênticas e descarta o teste real.
    static uint32_t _ultimoMov = 0;
    if (estadoAtual == ESTADO_CONCLUIDO || estadoAtual == ESTADO_EMERGENCIA) {
        // Grava no máximo 1 amostra de estado final a cada 2 s
        if (now - _ultimoMov < 2000) return;
        _ultimoMov = now;
    }

    float rpm1 = encoders.getRPM1();
    float rpm2 = encoders.getRPM2();
    int32_t t1  = abs(encoders.getTicksM1());
    int32_t t2  = abs(encoders.getTicksM2());
    float yaw   = imu.isInitialized() ? imu.getYawAngleDeg() : 0.0f;
    float rpmAlvo = (estadoAtual == ESTADO_EXECUTANDO_TRAJETORIA)
                        ? (g_emLinhaReta ? VELOCIDADE_CRUZEIRO_RPM : VELOCIDADE_GIRO_RPM)
                        : 0.0f;

    recorder.sample(now, codigoEstadoRecorder(), (uint8_t)g_ladoAtual,
                   yaw, g_yawAbs, rpm1, rpm2, t1, t2, g_targetTicks,
                   g_lastPwm1, g_lastPwm2, rpmAlvo);
}

// =========================================================================
// SETUP
// =========================================================================
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(1000);

    Serial.println("\n==============================================");
    Serial.println("  ROBO 2WD - ESP32 V4 (Wi-Fi + Dashboard)     ");
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

    // 3. Inicialização do Wi-Fi e WebSocket (NÃO bloqueia o boot em caso de falha)
    Serial.println("[WIFI] Conectando a rede Wi-Fi...");
    g_wifiOk = wirelessTelem.begin();
    if (g_wifiOk) {
        Serial.printf("[OK] Wi-Fi conectado! IP: %s | Dashboard: http://robo.local/\n", wirelessTelem.getIP().c_str());
    } else {
        Serial.println(F("[AVISO] Wi-Fi nao conectou. Operando sem telemetria sem fio."));
    }

    // 4. Procedimento de Armamento do ESC (antes de subscrever a task no WDT)
    Serial.printf("[STATUS] Arming ESC via MCPWM (%d ms em neutro 1500 us)...\n", ESC_ARM_TEMPO_MS);
    motors.stopAll();
    delay(ESC_ARM_TEMPO_MS);
    Serial.println("[STATUS] ESC Armado com sucesso!");

    // 5. Reconfiguracao e ativacao do Task Watchdog Timer (WDT)
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
    Serial.println(" Acesse http://robo.local/ ou http://[IP]/ no browser.");
    Serial.println(" Clique em START no dashboard para iniciar a trajetória.");
    Serial.println(" (Ou aguarde 5 segundos para início automático)");
    Serial.println("=======================================================\n");

    estadoAtual = ESTADO_AGUARDO_INICIO;
    g_pausaStartTime = millis();

    // 6. Inicialização do gravador de telemetria em RAM (replay de teste)
    recorder.begin();
    Serial.printf("[REC] Gravador de telemetria ativo: %d amostras a %d Hz (max %.0f s).\n",
                  REC_MAX_SAMPLES, 1000 / REC_INTERVAL_MS, (float)REC_MAX_SAMPLES * REC_INTERVAL_MS / 1000.0f);
    Serial.println(F("[REC] Envie 'd' (serial) ou 'dump' (WebSocket) para despejar o buffer em CSV."));

    // 7. Enlace ESP-NOW: o init e o registro do peer são feitos DEPOIS da
    //    conexão Wi-Fi, porque o peer precisa nascer com o canal atual da
    //    interface (registrá-lo antes usa um canal que o connect sobrescreve).
    g_espnowOk = espnow.begin();
    if (g_espnowOk) {
        espnow.fixChannel();
        espnow.addPeerAfterChannel();
    } else {
        Serial.println(F("[AVISO] ESP-NOW indisponivel. Operando apenas com Wi-Fi."));
    }
    g_lastEspnowRx = millis();
}

// =========================================================================
// LOOP NÃO-BLOQUEANTE
// =========================================================================
void loop() {
    uint32_t now = millis();

    // Processar eventos Wi-Fi/WebSocket
    if (g_wifiOk) {
        wirelessTelem.handle();
    }

    // Alimenta o Watchdog se o sistema estiver operando normalmente
    if (estadoAtual != ESTADO_FALHA_HARDWARE) {
        esp_task_wdt_reset();
    }

    // Envio periódico de telemetria sem fio (5 Hz)
    if (estadoAtual != ESTADO_FALHA_HARDWARE) {
        enviarTelemetria(now);
        enviarTelemEspNow(now);
        gravarAmostraRecorder(now);
    }

    // Comando de dump do gravador via Serial ('d') ou WebSocket ("dump")
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'd' || c == 'D') {
            _dumpParaWs = false;
            recorder.dump();
        }
    }

    // Envio incremental do dump via WebSocket (não bloqueia o loop)
    if (_dumpParaWs && _dumpIdx < _dumpTotal) {
        if (now - _dumpUltimoEnvio >= DUMP_WS_INTERVAL_MS) {
            _dumpUltimoEnvio = now;
            uint16_t fim = (uint16_t)min((uint16_t)(_dumpIdx + DUMP_WS_POR_LOTE), _dumpTotal);
            for (uint16_t i = _dumpIdx; i < fim; i++) {
                recorder.dumpSample(i, _dumpLinha, sizeof(_dumpLinha));
                wirelessTelem.sendRaw(_dumpLinha);
            }
            _dumpIdx = fim;
        }
    } else if (_dumpParaWs && _dumpIdx >= _dumpTotal) {
        _dumpParaWs = false;
        wirelessTelem.sendRaw("[REC] Dump concluido.");
    }

    // Envio incremental do dump via ESP-NOW (o gateway reencaminha ao PC).
    // Lote pequeno e intervalo maior: rajadas de ESP-NOW descartam pacotes no
    // gateway (ele trata o payload como telemetria e imprime, mas o excesso
    // transborda o buffer de recepção).
    if (_dumpParaEspNow && _dumpIdx < _dumpTotal) {
            if (now - _dumpUltimoEnvio >= DUMP_ESPNOW_INTERVAL_MS) {
                _dumpUltimoEnvio = now;
                uint16_t fim = (uint16_t)min((uint16_t)(_dumpIdx + DUMP_ESPNOW_POR_LOTE), _dumpTotal);
                for (uint16_t i = _dumpIdx; i < fim; i++) {
                    recorder.dumpSample(i, _dumpLinha, sizeof(_dumpLinha));
                    espnow.sendTelemetry(_dumpLinha);
                }
                _dumpIdx = fim;
            }
        } else if (_dumpParaEspNow && _dumpIdx >= _dumpTotal) {
            _dumpParaEspNow = false;
            espnow.sendTelemetry("[REC] Dump concluido.");
        }

    // Tratamento centralizado dos comandos de texto (start/stop/pause/reset/dump).
    // Um UNICO ponto de consumo de cada flag: se dois blocos chamarem
    // hasNewCommand()/hasCommand(), o primeiro zera a flag e o segundo nunca vê
    // o comando. Dispatch central (um switch no topo do loop()).
    String cmdTxt;
    bool _comandoVeioPorEspNow = false;
    if (g_wifiOk && wirelessTelem.hasNewCommand()) {
        char c = wirelessTelem.getLastCommand();
        Serial.print(F("[LOOP] comando WS: "));
        Serial.println(c);
        _comandoVeioPorEspNow = false;
        switch (c) {
            case 's': cmdTxt = "start"; break;
            case 'x': cmdTxt = "stop";  break;
            case 'p': cmdTxt = "pause"; break;
            case 'r': cmdTxt = "reset"; break;
            case 'd': cmdTxt = "dump";  break;
            default: break;
        }
    }
    if (g_espnowOk && espnow.hasCommand()) {
        const char* e = espnow.getLastCommand();
        g_lastEspnowRx = now;   // enlace vivo
        Serial.print(F("[LOOP] comando ESP-NOW: "));
        Serial.println(e);
        if (cmdTxt.isEmpty()) {
            cmdTxt = String(e);
            _comandoVeioPorEspNow = true;
        }
    }

    if (!cmdTxt.isEmpty()) {
        if (cmdTxt == "start") {
            if (estadoAtual == ESTADO_AGUARDO_INICIO || estadoAtual == ESTADO_CONCLUIDO) {
                Serial.println("\n>>> COMANDO RECEBIDO! INICIANDO TRAJETÓRIA DO QUADRADO <<<");
                recorder.clear();
                // Ancora o referencial de rumo no início do trajeto: sem isso, a
                // deriva do giroscópio acumulada enquanto o robô ficou parado
                // (tipicamente 2–3°) apareceria como erro lateral no mapa.
                g_yawAbs = 0.0f;
                g_ladoAtual = 1;
                subEstado = SUB_INICIAR_SEGMENTO;
                g_emLinhaReta = true;
                estadoAtual = ESTADO_EXECUTANDO_TRAJETORIA;
            }
        } else if (cmdTxt == "dump") {
            _dumpIdx = 0;
            _dumpTotal = recorder.count();
            _dumpCanalEspNow = _comandoVeioPorEspNow;
            if (_dumpCanalEspNow) {
                _dumpParaEspNow = true;
                Serial.println(F("[REC] Dump remoto via ESP-NOW solicitado."));
            } else {
                _dumpParaWs = true;
                Serial.println(F("[REC] Dump remoto via WebSocket solicitado."));
            }
        } else if (cmdTxt == "stop" || cmdTxt == "pause") {
            if (estadoAtual == ESTADO_EXECUTANDO_TRAJETORIA) {
                motors.stopAll();
                g_lastPwm1 = ESC_NEUTRO_US;
                g_lastPwm2 = ESC_NEUTRO_US;
                estadoAtual = ESTADO_CONCLUIDO;
                Serial.println("\n[PARADA REMOTA] Trajetoria interrompida!");
            }
        } else if (cmdTxt == "reset") {
            recorder.clear();
            g_yawAbs = 0.0f;   // reancora o referencial de rumo
            g_ladoAtual = 1;
            g_emLinhaReta = true;
            subEstado = SUB_INICIAR_SEGMENTO;
            estadoAtual = ESTADO_EXECUTANDO_TRAJETORIA;
        }
    }

    switch (estadoAtual) {
        case ESTADO_FALHA_HARDWARE:
            motors.stopAll();
            g_lastPwm1 = ESC_NEUTRO_US;
            g_lastPwm2 = ESC_NEUTRO_US;
            delay(500);
            break;

        case ESTADO_EMERGENCIA:
            motors.stopAll();
            g_lastPwm1 = ESC_NEUTRO_US;
            g_lastPwm2 = ESC_NEUTRO_US;
            static uint32_t lastEmergAlert = 0;
            if (now - lastEmergAlert > 2000) {
                lastEmergAlert = now;
                Serial.println("[ALERTA] Robô em ESTADO_EMERGENCIA! Reinicie para operar.");
            }
            delay(50);
            break;

        case ESTADO_AGUARDO_INICIO:
            // Início por Serial (debug). O início remoto via WebSocket é tratado
            // de forma centralizada no topo do loop().
            {
                bool comandoRecebido = false;
                if (Serial.available()) {
                    char c = Serial.read();
                    if (c == 's' || c == 'S' || c == 'g' || c == 'G' || c == '\n') {
                        comandoRecebido = true;
                    }
                }
                if (comandoRecebido) {
                    Serial.println("\n>>> COMANDO RECEBIDO! INICIANDO TRAJETÓRIA DO QUADRADO <<<");
                    recorder.clear(); // Descarta amostras do teste anterior
                    g_yawAbs = 0.0f;   // ancora o referencial de rumo no início
                    g_ladoAtual = 1;
                    subEstado = SUB_INICIAR_SEGMENTO;
                    g_emLinhaReta = true;
                    estadoAtual = ESTADO_EXECUTANDO_TRAJETORIA;
                }
            }
            // Timeout automático de 5 segundos (apenas sem Wi-Fi: com Wi-Fi o
            // início é remoto e o buffer do gravador deve ser preservado)
            if (!g_wifiOk && now - g_pausaStartTime >= 5000) {
                Serial.println("\n>>> TIMEOUT DE 5s ATINGIDO! INICIANDO TRAJETÓRIA DO QUADRADO <<<");
                recorder.clear();
                g_yawAbs = 0.0f;   // ancora o referencial de rumo no início
                g_ladoAtual = 1;
                subEstado = SUB_INICIAR_SEGMENTO;
                g_emLinhaReta = true;
                estadoAtual = ESTADO_EXECUTANDO_TRAJETORIA;
            }
            break;

        case ESTADO_EXECUTANDO_TRAJETORIA:
            // Stop/pause remotos são tratados de forma centralizada no topo do loop().

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
                            g_lastPwm1 = ESC_NEUTRO_US;
                            g_lastPwm2 = ESC_NEUTRO_US;
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
            g_lastPwm1 = ESC_NEUTRO_US;
            g_lastPwm2 = ESC_NEUTRO_US;
            static uint32_t lastFinishMsg = 0;
            if (now - lastFinishMsg > 5000) {
                lastFinishMsg = now;
                Serial.println("[STATUS] Execução finalizada com perfeição. Envie 's' ou clique START no dashboard para repetir.");
            }
            {
                bool comandoRecebido = false;
                if (Serial.available()) {
                    char c = Serial.read();
                    if (c == 's' || c == 'S' || c == 'g' || c == 'G') {
                        comandoRecebido = true;
                    }
                }
                if (comandoRecebido) {
                    Serial.println("\n>>> COMANDO RECEBIDO! REINICIANDO TRAJETÓRIA DO QUADRADO <<<");
                    recorder.clear(); // Descarta amostras do teste anterior
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
