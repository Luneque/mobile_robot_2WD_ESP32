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
#include "battery_monitor.h"
#include "secrets.h"

// =========================================================================
// FIRMWARE ROBO 2WD - VERSAO 4 (V3 + Wi-Fi + WebSocket Dashboard)
// Novidades V4:
//  7. Wi-Fi Station Mode com dashboard WebSocket em tempo real (robo.local).
//  8. Comandos de início/parada recebidos via WebSocket (sem cabo serial).
//  9. Telemetria JSON adaptativa (20 Hz em movimento, 2 Hz parado), publicada
//     por tarefa dedicada fora do laço: RPM, Yaw, ticks, PWM, estado da FSM.
// 10. Preparado para ESP-NOW (gateway ESP32 como fallback sem Wi-Fi).
// =========================================================================

MotorControlMCPWM motors;
EncoderReaderPCNT encoders;
IMU_BMI160 imu;

PIDController pid1(PID_KP_M1, PID_KI_M1, PID_KD_M1, ESC_MIN_US, ESC_MAX_US, ESC_NEUTRO_US);
PIDController pid2(PID_KP_M2, PID_KI_M2, PID_KD_M2, ESC_MIN_US, ESC_MAX_US, ESC_NEUTRO_US);

WifiTelemetry wirelessTelem;
bool g_wifiOk = false;
uint32_t g_ultimaTentativaWifi = 0;   // controle da reconexao automatica
uint32_t g_lastTelemTime = 0;

// Enlace ESP-NOW com o gateway ESP32 (redundancia/alternativa ao Wi-Fi)
EspNowLink espnow;
bool g_espnowOk = false;
uint32_t g_lastEspnowTelem = 0;

// Gravador de telemetria em RAM (para testes sem cabo serial, ex.: robô no solo)
TelemetryRecorder recorder;

// Monitor de bateria LiPo 2S (divisor R1=100k/R2=47k no GPIO 34)
BatteryMonitor battery;
float g_vbat = -1.0f;          // tensao da bateria (V); -1 = desabilitado/sem leitura
uint8_t g_vbatNivel = 0;       // 0 ok, 1 atencao, 2 critico
uint32_t g_vbatUltimoAviso = 0;
// RPM alvo INSTANTANEO do perfil trapezoidal (publicado na telemetria). Ate
// 23/11/2026 a telemetria enviava sempre o valor fixo de cruzeiro/giro.
float g_rpmAlvo = 0.0f;

// Dump incremental via WebSocket / ESP-NOW (robô sem cabo)
static bool     _dumpParaWs = false;
static bool     _dumpParaEspNow = false;
static bool     _dumpCanalEspNow = false;
static uint16_t _dumpIdx = 0;
static uint16_t _dumpTotal = 0;
static uint32_t _dumpUltimoEnvio = 0;
static char     _dumpLinha[DUMP_LINE_MAX];
static bool     _dumpTruncadoAvisado = false;
#define DUMP_WS_INTERVAL_MS 50
#define DUMP_WS_POR_LOTE    10
// ESP-NOW: lote menor e intervalo maior (rajadas descartam pacotes no gateway)
#define DUMP_ESPNOW_INTERVAL_MS 100
#define DUMP_ESPNOW_POR_LOTE    1

// Detecta linha de dump truncada: dumpSample() devolve um valor >= do tamanho
// do buffer quando o snprintf nao coube. Avisa uma unica vez por dump.
static void avisarTruncamentoDump(size_t len) {
    if (len >= sizeof(_dumpLinha) && !_dumpTruncadoAvisado) {
        _dumpTruncadoAvisado = true;
        Serial.println(F("[REC] AVISO: linha de dump truncada (buffer menor que a linha)."));
    }
}

// Cessao usada pelo despejo do gravador: alimenta o Task Watchdog e cede o
// processador. Sem isso um dump completo (~13 s) reiniciaria o robo.
static void cederEAlimentarWdt() {
    esp_task_wdt_reset();
}

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

// Diagnostico de travamento do laco: maior intervalo entre duas iteracoes
// consecutivas de loop(). Publicado como "lm" na telemetria e zerado a cada
// amostra. Um valor alto significa que o controle ficou congelado por aquele
// periodo com os motores no ultimo PWM aplicado.
uint32_t g_loopAnteriorMs = 0;
uint32_t g_loopMaxMs = 0;
uint32_t g_pausaStartTime = 0;

// Diagnostico de secao lenta: maior duracao de um bloco do loop() desde a
// ultima amostra de telemetria, e qual bloco foi (id). Publicado como "sr"
// (ms) e "si" (id). So' com isto e' possivel localizar a origem de um "lm"
// alta sem adivinhar: Wi-Fi=1, telemetria (fila)=2, gravador=3,
// bateria=4, controle/FSM=5.
uint32_t g_secMaxUs = 0;
uint8_t  g_secId    = 0;

static inline void marcarSecao(uint8_t id, uint32_t t0) {
    uint32_t d = (uint32_t)(micros() - t0);
    if (d > g_secMaxUs) { g_secMaxUs = d; g_secId = id; }
}

int32_t g_targetTicks = 0;
float g_distanciaTotalMm = 0.0f;
float g_anguloTotalGraus = 0.0f;
// Sentido do giro. Convencao de sinal do BMI160 (eixo Z para cima): yaw
// POSITIVO = rotacao ANTI-HORARIA. O mapa do dashboard renderiza -yaw.
int g_sentidoGiro = 1; // +1 = anti-horario (yaw positivo)
float g_yawStartSeg = 0.0f; // Ângulo de yaw exato no instante em que o movimento inicia
float g_yawAbs = 0.0f;      // Rumo GLOBAL (graus, anti-horario = positivo) acumulado entre segmentos
float g_yawPrevAmostra = 0.0f; // Yaw relativo da amostra anterior (para integrar o delta)
// Rumo global por ODOMETRIA (ticks com sinal): fonte independente da IMU, usada para
// detectar divergencia entre o giroscopio e os encoders.
float g_yawOdom = 0.0f;
int32_t g_odomLastT1 = 0, g_odomLastT2 = 0;
// Instrumentacao de diagnostico: total de ciclos e soma de (dT1 - dT2) integrados.
uint32_t g_odomCiclos = 0;
int32_t  g_odomSoma = 0;
// Inercia: o giro que o robo ainda faz DEPOIS do corte (medido na pausa).
// Fica fora do rumo global de proposito — a IMU deriva quando os motores
// param, e somar isso ao rumo global corromperia o mapa e a telemetria.
float g_inerciaImu = 0.0f;
float g_inerciaOdom = 0.0f;
// Soma de TODAS as inercias (a do segmento atual e as anteriores). E' o que
// permite o "rumo real" global: yawReal = g_yawAbs + g_inerciaTotal.
float g_inerciaTotal = 0.0f;
// Autoteste de calibragem do giroscopio (feito no boot, com o robô parado)
float g_gyroBiasDps = 0.0f;
float g_gyroPeakDps = 0.0f;
// Modo de diagnostico: executa apenas UMA reta (LADO_QUADRADO_MM) e conclui.
// Usado para medir no chao o comprimento real do lado (calibragem de PPR/roda).
bool g_modoRetaUnica = false;
// Modo de diagnostico: gira ANGULO_CURVA_GRAUS no lugar e conclui. Serve para
// medir no chao o angulo real do pivo (separar "IMU errada" de "bitola errada").
bool g_modoPivoUnico = false;

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
    g_odomLastT1 = encoders.getTicksM1(); // re-sincroniza a odometria com o reset
    g_odomLastT2 = encoders.getTicksM2();
    g_inerciaImu = 0.0f;   // a inercia e' de cada segmento
    g_inerciaOdom = 0.0f;
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
    g_odomLastT1 = encoders.getTicksM1(); // re-sincroniza a odometria com o reset
    g_odomLastT2 = encoders.getTicksM2();
    g_inerciaImu = 0.0f;   // a inercia e' de cada segmento
    g_inerciaOdom = 0.0f;
    g_desbalStartTime = 0;
    subEstado = SUB_EM_MOVIMENTO;
    Serial.printf("\n[V4-PIVO] Curva %d de %d | Angulo: %.1f deg (%s) | Alvo: %d ticks/roda\n",
                  g_ladoAtual, QUADRADO_LADOS, anguloGraus,
                  (sentido > 0) ? "ANTI-HORARIO" : "HORARIO", g_targetTicks);
}

// =========================================================================
// REANCORAGEM DE TRAJETORIA
// =========================================================================
// Unico ponto que zera os referenciais de rumo/odometria/inercia, a
// instrumentacao de diagnostico e arma a FSM. Antes eram 6 blocos copiados,
// e o reinicio por Serial do ESTADO_CONCLUIDO não reancorava nada (o segundo
// quadrado consecutivo herdava o rumo do anterior).
//
// Ancora o referencial de rumo: a deriva do giroscopio acumulada enquanto o
// robo fica parado (tipicamente 2-3 graus) apareceria como erro lateral.
void reancorarTrajetoria() {
    recorder.clear();
    g_yawAbs = 0.0f;
    g_yawOdom = 0.0f;
    g_inerciaTotal = 0.0f;
    g_odomSoma = 0;
    g_odomCiclos = 0;
    g_ladoAtual = 1;
    g_modoRetaUnica = false;
    g_modoPivoUnico = false;
    g_emLinhaReta = true;
    subEstado = SUB_INICIAR_SEGMENTO;
    estadoAtual = ESTADO_EXECUTANDO_TRAJETORIA;
}

// Integra o rumo (IMU + odometria de ticks com sinal).
//
// naPausa = false: alimenta o rumo global (g_yawAbs/g_yawOdom), usado pelo
//   controle e pelo mapa.
// naPausa = true : alimenta os acumuladores de INERCIA (g_inerciaImu/
//   g_inerciaOdom) — o giro que o robo ainda faz depois de os motores
//   pararem. Fora do rumo global de proposito (a IMU deriva ao parar).
void integrarRumo(uint32_t now, float dt, bool naPausa) {
    g_lastLoopTime = now;

    if (imu.isInitialized()) {
        imu.update(dt);
        float yawAmostra = imu.getYawAngleDeg();
        float delta = yawAmostra - g_yawPrevAmostra;
        if (naPausa) { g_inerciaImu += delta; g_inerciaTotal += delta; }
        else g_yawAbs += delta;
        g_yawPrevAmostra = yawAmostra;
    }

    int32_t s1 = encoders.getTicksM1();
    int32_t s2 = encoders.getTicksM2();
    int32_t d1 = s1 - g_odomLastT1;
    int32_t d2 = s2 - g_odomLastT2;
    // Se os contadores foram zerados (novo segmento), re-sincroniza sem integrar.
    if (abs(d1) < 10000 && abs(d2) < 10000) {
        float dOdo = ticksDeltaParaGraus(d1, d2);
        if (naPausa) g_inerciaOdom += dOdo; else g_yawOdom += dOdo;
        g_odomCiclos++;
        g_odomSoma += (d1 - d2);
    }
    g_odomLastT1 = s1;
    g_odomLastT2 = s2;
}

void processarControleMovimento(uint32_t now) {
    float dt = (now - g_lastLoopTime) / 1000.0f;
    encoders.updateSpeed(dt);
    integrarRumo(now, dt, false);

    // RPM alvo instantaneo começa zerado: se o ramo de emergencia fizer return
    // antes do perfil trapezoidal, a telemetria nao publica um valor velho.
    g_rpmAlvo = 0.0f;

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

        if (sync_corr > SYNC_CLAMP_PIVO_US) sync_corr = SYNC_CLAMP_PIVO_US;
        if (sync_corr < -SYNC_CLAMP_PIVO_US) sync_corr = -SYNC_CLAMP_PIVO_US;

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

    // O RPM alvo que a telemetria deve reportar e' o do perfil trapezoidal
    // (rampa 20->55 RPM na reta, 20->25 no pivo), nao o fixo de cruzeiro/giro.
    g_rpmAlvo = (fabsf(targetRpmM1) + fabsf(targetRpmM2)) / 2.0f;
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
    pkt.yawAbs     = g_yawAbs;   // rumo global integrado (anti-horario = positivo)
    pkt.yawOdom    = g_yawOdom;  // rumo global por odometria (fonte independente)
    pkt.diverg     = g_yawAbs - g_yawOdom;
    pkt.gyroBias   = g_gyroBiasDps;
    pkt.odomSoma   = g_odomSoma;
    pkt.odomCiclos = g_odomCiclos;
    pkt.inercia    = g_inerciaImu;
    pkt.inerciaOdom = g_inerciaOdom;
    pkt.yawReal    = g_yawAbs + g_inerciaTotal;   // rumo real (cortes + todas as inercias)
    pkt.rpm1       = rpm1;
    pkt.rpm2       = rpm2;
    pkt.ticks1     = t1;
    pkt.ticks2     = t2;
    pkt.ticksAlvo  = g_targetTicks;
    pkt.pwm1       = g_lastPwm1;
    pkt.pwm2       = g_lastPwm2;
    // Fora de execucao o alvo e' zero (senao a telemetria publica o ultimo
    // valor do perfil, ex.: 55 RPM com o robo parado).
    pkt.rpmAlvo    = (estadoAtual == ESTADO_EXECUTANDO_TRAJETORIA) ? g_rpmAlvo : 0.0f;
    pkt.vbat       = g_vbat;
    pkt.vbatNivel  = g_vbatNivel;
    pkt.loopMaxMs  = g_loopMaxMs;
    pkt.secMs      = g_secMaxUs / 1000.0f;   // pior bloco desde a última amostra
    pkt.secId      = g_secId;                // 1 Wi-Fi, 2 telemetria (fila), 3 gravador,
                                             // 4 bateria, 5 controle/FSM

    // Enfileira para a tarefa de rede: o laço nao escreve no socket.
    wirelessTelem.agendarTelemetria(pkt);
    g_loopMaxMs = 0;   // a janela recomeça na próxima amostra
    g_secMaxUs = 0;
    g_secId    = 0;
}

// =========================================================================
// MONITOR DE BATERIA (LiPo 2S)
// =========================================================================
// Le a tensao, publica na telemetria e, se critica, corta os motores.
// A leitura e' feita aqui (e nao dentro de enviarTelemetria) para que o
// intervalo seja fixo (VBAT_LEITURA_MS), independente da taxa de telemetria.
void monitorarBateria(uint32_t now) {
    if (!battery.isEnabled()) { g_vbat = -1.0f; g_vbatNivel = 0; return; }

    g_vbat = battery.readVolts();
    if (g_vbat < 0.0f) return;
    g_vbatNivel = battery.nivel();

    // Aviso periodico no Serial (nao inunda: 1x/2 s)
    if (g_vbatNivel >= 1 && now - g_vbatUltimoAviso >= 2000) {
        g_vbatUltimoAviso = now;
        Serial.print(F("[BATERIA] "));
        Serial.print(g_vbat, 2);
        Serial.print(F(" V (adc "));
        Serial.print(battery.lastAdc(), 0);
        Serial.print(F(") — "));
        Serial.println(g_vbatNivel == 2 ? F("CRITICO! Cortando motores.")
                                       : F("atencao (limiar 7,0 V)."));
    }

    // Corte de seguranca: abaixo do limiar, para os motores e encerra a
    // trajetoria. A LiPo 2S abaixo de 3,30 V/celula e' danificada.
    if (g_vbatNivel == 2 && estadoAtual == ESTADO_EXECUTANDO_TRAJETORIA) {
        motors.stopAll();
        g_lastPwm1 = ESC_NEUTRO_US;
        g_lastPwm2 = ESC_NEUTRO_US;
        estadoAtual = ESTADO_EMERGENCIA;
        Serial.println(F("\n[BATERIA] Tensao critica! Trajetoria interrompida."));
    }
}

// =========================================================================
// TELEMETRIA ESP-NOW (redundancia/alternativa ao Wi-Fi)
// =========================================================================
// Serializa o mesmo pacote em JSON e envia ao gateway ESP32.
// Executada pela tarefa de rede (registrada com setHookEspNow), nao pelo
// laço: esp_now_send tambem escreve no radio e pode bloquear.
void enviarTelemEspNow() {
    uint32_t now = millis();
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

    // JSON da telemetria ESP-NOW (~210 bytes no pior caso)
    char buf[300];
    int n = snprintf(buf, sizeof(buf),
        "{\"ts\":%lu,\"st\":\"%s\",\"sd\":%d,\"of\":%d,\"yaw\":%.2f,\"yawAbs\":%.2f,"
        "\"yawOdom\":%.2f,\"diverg\":%.2f,\"gyroBias\":%.3f,"
        "\"yawReal\":%.2f,\"inercia\":%.2f,\"inerciaOdom\":%.2f,"
        "\"r1\":%.1f,\"r2\":%.1f,\"t1\":%ld,\"t2\":%ld,\"tg\":%ld,"
        "\"w1\":%u,\"w2\":%u,\"rt\":%.1f,\"vbat\":%.2f,\"vbatNivel\":%u}",
        (unsigned long)now, estadoStr, g_ladoAtual, QUADRADO_LADOS, yaw, g_yawAbs,
        g_yawOdom, g_yawAbs - g_yawOdom, g_gyroBiasDps,
        g_yawAbs + g_inerciaTotal, g_inerciaImu, g_inerciaOdom,
        rpm1, rpm2, (long)t1, (long)t2, (long)g_targetTicks,
        (unsigned)g_lastPwm1, (unsigned)g_lastPwm2,
        emMovimento ? g_rpmAlvo : 0.0f,
        g_vbat, (unsigned)g_vbatNivel);

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
    float rpmAlvo = (estadoAtual == ESTADO_EXECUTANDO_TRAJETORIA) ? g_rpmAlvo : 0.0f;

    recorder.sample(now, codigoEstadoRecorder(), (uint8_t)g_ladoAtual,
                   yaw, g_yawAbs, g_yawOdom, g_yawAbs - g_yawOdom, g_gyroBiasDps,
                   rpm1, rpm2, t1, t2, g_targetTicks,
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

        // Autoteste de calibragem: com o corpo parado, a taxa de yaw deve ser ~0.
        // Usa a taxa SEM a zona morta de ruido (getRawYawRateDps), que e' justamente
        // o que a zona morta esconde. Boot contaminado (corpo em movimento na
        // calibragem) aparece aqui como bias alto.
        {
            float soma = 0.0f, pico = 0.0f;
            const int N = 200;   // 200 x 10 ms = 2 s parado
            for (int i = 0; i < N; i++) {
                float r = imu.getRawYawRateDps();
                soma += r;
                if (fabsf(r) > pico) pico = fabsf(r);
                delay(10);
            }
            g_gyroBiasDps = soma / (float)N;
            g_gyroPeakDps = pico;
            Serial.printf("[IMU] Autoteste: bias %+.3f graus/s | pico %.3f graus/s\n",
                          g_gyroBiasDps, g_gyroPeakDps);
            if (fabsf(g_gyroBiasDps) > 0.5f) {
                Serial.println("[IMU][ALERTA] Bias alto — o corpo se moveu durante o boot. "
                               "Desligue e ligue o robo parado para recalibrar.");
            }
        }
    } else {
        Serial.println("[AVISO] IMU BMI160 nao detectada no barramento I2C. Operando apenas com odometria PCNT.");
    }

    // 3. Inicialização do Wi-Fi e WebSocket (NÃO bloqueia o boot em caso de falha)
    Serial.println("[WIFI] Conectando a rede Wi-Fi...");
    g_wifiOk = wirelessTelem.begin();
    if (g_wifiOk) {
        // Telemetria ESP-NOW passa a ser emitida pela tarefa de rede.
        wirelessTelem.setHookEspNow(enviarTelemEspNow);
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
    // O despejo por Serial e' feito em lotes; entre um lote e outro o gravador
    // chama esta cessao para alimentar o Task Watchdog.
    recorder.setYieldFn(cederEAlimentarWdt);
    Serial.printf("[REC] Gravador de telemetria ativo: %d amostras a %d Hz (max %.0f s).\n",
                  REC_MAX_SAMPLES, 1000 / REC_INTERVAL_MS, (float)REC_MAX_SAMPLES * REC_INTERVAL_MS / 1000.0f);
    Serial.println(F("[REC] Envie 'd' (serial) ou 'dump' (WebSocket) para despejar o buffer em CSV."));

    // 8. Monitor de bateria (divisor R1=100k/R2=47k no GPIO 34)
    battery.begin();
    battery.carregarCalibracao();   // restaura a calibragem gravada na NVS
    if (battery.isEnabled()) {
        g_vbat = battery.readVolts();
        Serial.print(F("[BATERIA] Monitor ativo no GPIO "));
        Serial.print(PIN_BATTERY_ADC);
        Serial.print(F(" | Tensao: "));
        if (g_vbat >= 0.0f) {
            Serial.print(g_vbat, 2);
            Serial.print(F(" V (adc "));
            Serial.print(battery.lastAdc(), 0);
            Serial.print(F(", limiar atencao "));
            Serial.print(VBAT_LIMIAR_ATENCAO, 1);
            Serial.print(F(" V / corte "));
            Serial.print(VBAT_LIMIAR_CORTE, 1);
            Serial.println(F(" V)"));
            if (!battery.isCalibrated()) {
                Serial.println(F("[BATERIA] SEM CALIBRACAO: usando a curva nominal do ADC."));
                Serial.println(F("[BATERIA] Mecca a bateria no multimetro e envie 'vbat <tensao>'."));
            } else {
                Serial.println(F("[BATERIA] Calibragem carregada da NVS."));
            }
        } else {
            Serial.println(F("sem leitura (divisor nao montado?)."));
        }
    } else {
        Serial.println(F("[BATERIA] Monitor desabilitado (PIN_BATTERY_ADC = -1)."));
    }

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
}

// =========================================================================
// LOOP NÃO-BLOQUEANTE
// =========================================================================
void loop() {
    uint32_t now = millis();

    // Mede o periodo da iteracao anterior (overflow de millis() e' seguro).
    if (g_loopAnteriorMs != 0) {
        uint32_t periodo = now - g_loopAnteriorMs;
        if (periodo > g_loopMaxMs) g_loopMaxMs = periodo;
    }
    g_loopAnteriorMs = now;

    uint32_t _tSec;

    // Mantem o Wi-Fi vivo. O g_wifiOk era setado uma unica vez no setup e nunca
    // re-verificado: uma queda de Wi-Fi deixava o dashboard "desconectado" ate
    // um reboot, e o ESP32 nao reconecta de forma confiavel sozinho.
    _tSec = micros();
    if (WiFi.status() != WL_CONNECTED) {
        if (g_wifiOk) {
            g_wifiOk = false;
            Serial.println("[WiFi] Conexao perdida.");
        }
        if (now - g_ultimaTentativaWifi >= 5000) {
            g_ultimaTentativaWifi = now;
            WiFi.reconnect();
            Serial.println("[WiFi] Tentando reconectar...");
        }
    } else if (!g_wifiOk) {
        g_wifiOk = true;
        Serial.print("[WiFi] Reconectado! IP: ");
        Serial.println(WiFi.localIP());
    }

    // Processar eventos Wi-Fi/WebSocket fica a cargo da tarefa de rede
    // (wifi_telemetry.h), para nao bloquear o laço de controle.
    marcarSecao(1, _tSec);   // seção 1: Wi-Fi

    // Alimenta o Watchdog se o sistema estiver operando normalmente
    if (estadoAtual != ESTADO_FALHA_HARDWARE) {
        esp_task_wdt_reset();
    }

    // Envio periódico de telemetria sem fio (adaptativo: 20 Hz em movimento)
    if (estadoAtual != ESTADO_FALHA_HARDWARE) {
        _tSec = micros();
        enviarTelemetria(now);          // so' enfileira; a tarefa de rede publica
        marcarSecao(2, _tSec);          // seção 2: telemetria (enfileirada)

        _tSec = micros();
        gravarAmostraRecorder(now);
        marcarSecao(3, _tSec);       // seção 3: gravador em RAM

        _tSec = micros();
        monitorarBateria(now);
        marcarSecao(4, _tSec);       // seção 4: monitor de bateria
    }

    // Comando de dump do gravador via Serial ('d') ou WebSocket ("dump")
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'd' || c == 'D') {
            _dumpParaWs = false;
            _dumpTruncadoAvisado = false;
            recorder.dump();
        }
    }

    // Envio incremental do dump via WebSocket (não bloqueia o loop)
    if (_dumpParaWs && _dumpIdx < _dumpTotal) {
        if (now - _dumpUltimoEnvio >= DUMP_WS_INTERVAL_MS) {
            _dumpUltimoEnvio = now;
            uint16_t fim = (uint16_t)min((uint16_t)(_dumpIdx + DUMP_WS_POR_LOTE), _dumpTotal);
            for (uint16_t i = _dumpIdx; i < fim; i++) {
                avisarTruncamentoDump(
                    recorder.dumpSample(i, _dumpLinha, sizeof(_dumpLinha)));
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
                    avisarTruncamentoDump(
                        recorder.dumpSample(i, _dumpLinha, sizeof(_dumpLinha)));
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
    // Parametro do comando (ex.: "vbat 8.22" -> _paramComando = "8.22")
    static char _paramComando[32] = {0};
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
            case 't': cmdTxt = "reta";  break;   // 't' de "teste de uma reta"
            case 'g': cmdTxt = "pivo";  break;   // 'g' de "giro" (pivo unico)
            case 'v': cmdTxt = "vbat"; strncpy(_paramComando, wirelessTelem.getLastParam(), sizeof(_paramComando) - 1); break;
            default: break;
        }
    }
    if (g_espnowOk && espnow.hasCommand()) {
        const char* e = espnow.getLastCommand();
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
                reancorarTrajetoria();
            }
        } else if (cmdTxt == "reta") {
            // Diagnostico: uma unica reta, para no fim. Serve para medir no chao
            // o comprimento real do lado e assim separar "IMU errada" de "PPR errado".
            if (estadoAtual == ESTADO_AGUARDO_INICIO || estadoAtual == ESTADO_CONCLUIDO) {
                Serial.println("\n>>> MODO RETA UNICA: 500 mm (diagnostico) <<<");
                reancorarTrajetoria();
                g_modoRetaUnica = true;   // reancorarTrajetoria() zera os modos
            }
        } else if (cmdTxt == "pivo") {
            // Diagnostico: um unico pivo no lugar, para no fim. Serve para medir
            // no chao o angulo real e assim separar "IMU errada" de "bitola errada".
            if (estadoAtual == ESTADO_AGUARDO_INICIO || estadoAtual == ESTADO_CONCLUIDO) {
                Serial.println("\n>>> MODO PIVO UNICO: 90 graus no lugar (diagnostico) <<<");
                reancorarTrajetoria();
                g_modoPivoUnico = true;   // reancorarTrajetoria() zera os modos
                g_emLinhaReta = false;    // comeca ja no giro
            }
        } else if (cmdTxt == "vbat") {
            // Calibragem de bateria: "vbat <tensao_real>". A tensao real e' a
            // medida no multimetro nos terminais da bateria. Aplica uma reta de
            // dois pontos: o ponto medido + o vazio (adc 0 -> 0 V).
            float alvo = atof(_paramComando);
            if (alvo > 0.0f && battery.isEnabled()) {
                float adc = battery.lastAdc();
                battery.setCalibracao(adc, alvo, 0.0f, 0.0f);
                battery.salvarCalibracao();   // persiste na NVS (sobrevive ao reboot)
                g_vbat = battery.readVolts();
                Serial.print(F("[BATERIA] Calibrada: adc "));
                Serial.print(adc, 0);
                Serial.print(F(" -> "));
                Serial.print(alvo, 2);
                Serial.print(F(" V | leitura atual: "));
                Serial.print(g_vbat, 2);
                Serial.println(F(" V (gravada na NVS)"));
            } else {
                Serial.println(F("[BATERIA] Calibragem invalida (use 'vbat <tensao>')."));
            }
        } else if (cmdTxt == "dump") {
            _dumpIdx = 0;
            _dumpTotal = recorder.count();
            _dumpTruncadoAvisado = false;
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
                reancorarTrajetoria();
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
                    reancorarTrajetoria();
                }
            }
            // Timeout automático de 5 segundos (apenas sem Wi-Fi: com Wi-Fi o
            // início é remoto e o buffer do gravador deve ser preservado)
            if (!g_wifiOk && now - g_pausaStartTime >= 5000) {
                Serial.println("\n>>> TIMEOUT DE 5s ATINGIDO! INICIANDO TRAJETÓRIA DO QUADRADO <<<");
                reancorarTrajetoria();
            }
            break;

        case ESTADO_EXECUTANDO_TRAJETORIA:
            // Stop/pause remotos são tratados de forma centralizada no topo do loop().
            _tSec = micros();

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
                // Integra apenas os acumuladores de INERCIA (nao o rumo global:
                // a IMU deriva quando os motores param e corromperia o mapa).
                if (now - g_lastLoopTime >= CONTROL_LOOP_MS) {
                    integrarRumo(now, (now - g_lastLoopTime) / 1000.0f, true);
                }
                // Aguarda a inércia mecânica estabilizar
                if (now - g_pausaStartTime >= PAUSA_ENTRE_MOVIMENTOS_MS) {
                    if (g_emLinhaReta) {
                        if (g_modoRetaUnica) {
                            // Uma reta basta: conclui em vez de iniciar o giro.
                            g_modoRetaUnica = false;
                            estadoAtual = ESTADO_CONCLUIDO;
                            motors.stopAll();
                            g_lastPwm1 = ESC_NEUTRO_US;
                            g_lastPwm2 = ESC_NEUTRO_US;
                            Serial.println("\n[RETA UNICA] Concluida. Mecha no chao e meca o lado.");
                        } else {
                            // Concluiu a reta: inicia o giro de 90°
                            g_emLinhaReta = false;
                            subEstado = SUB_INICIAR_SEGMENTO;
                        }
                    } else {
                        // Concluiu o giro: avança para o próximo lado
                        if (g_modoPivoUnico) {
                            // Um pivo basta: conclui em vez de seguir para o lado.
                            g_modoPivoUnico = false;
                            estadoAtual = ESTADO_CONCLUIDO;
                            motors.stopAll();
                            g_lastPwm1 = ESC_NEUTRO_US;
                            g_lastPwm2 = ESC_NEUTRO_US;
                            Serial.println("\n[PIVO UNICO] Concluido. Mecha o eixo do robo e meca o angulo.");
                        } else {
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
            }
            marcarSecao(5, _tSec);   // seção 5: controle/FSM (inclui leitura da IMU)
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
                    reancorarTrajetoria();
                }
            }
            delay(10);
            break;
    }
}
