#ifndef ROBO_MATEMATICA_H
#define ROBO_MATEMATICA_H

// =========================================================================
// Conversoes cinematicas usadas pelo controle de trajetoria.
//
// Extraidas do loop de controle (que ficava embutido em
// iniciarMovimentoReta/iniciarMovimentoCurvo) para poderem ser testadas
// fora do microcontrolador em tests/native — sem mudar o comportamento:
// as formulas sao exatamente as mesmas.
// =========================================================================
#include "config.h"   // MM_POR_PULSO, DIST_RODAS_MM, PI

// Milimetros -> ticks de encoder (truncamento igual ao do firmware).
static inline int32_t ticksParaMm(float mm) {
    return (int32_t)(mm / MM_POR_PULSO);
}

// Angulo de pivô (graus) -> ticks por roda, pelo arco percorrido.
static inline int32_t ticksParaArcoPivo(float anguloGraus) {
    float arco_mm = (PI * DIST_RODAS_MM * (anguloGraus / 360.0f));
    return (int32_t)(arco_mm / MM_POR_PULSO);
}

// Rotacao do chassis (graus) a partir de um par de deltas de ticks COM SINAL.
// Tracao diferencial: omega = (v1 - v2)/L. Sentido: dTicks1 > dTicks2 gira
// anti-horario (mesma convencao do yaw do BMI160, eixo Z para cima).
static inline float ticksDeltaParaGraus(int32_t dTicks1, int32_t dTicks2) {
    return (float)(dTicks1 - dTicks2) * MM_POR_PULSO / DIST_RODAS_MM * (180.0f / PI);
}

// ────────────────────────────────────────────────────────────────────────────
// Criterios de parada dos segmentos.
//
// Extraidos de processarControleMovimento() para serem testaveis fora do
// microcontrolador (tests/native/test_pivo.cpp) — as expressoes sao
// exatamente as mesmas.
// ────────────────────────────────────────────────────────────────────────────

// Reta: a distancia medida pelos encoders e o criterio primario.
static inline bool atingiuMetaReta(int32_t avgTicks, int32_t targetTicks) {
    return avgTicks >= targetTicks;
}

// Pivo: com IMU ativa ela e a autoridade do angulo, mas a odometria precisa
// confirmar movimento (guarda MIN_TICKS_PIVO) e serve de rede de seguranca
// (fallback PIVO_FALLBACK_FATOR) caso a IMU falhe. Sem IMU, para por ticks.
static inline bool atingiuMetaPivo(int32_t avgTicks, float yawAbs,
                                   int32_t targetTicks, float anguloTotalGraus,
                                   bool imuAtiva) {
    if (!imuAtiva) return avgTicks >= targetTicks;
    bool movimentoConfirmado = (avgTicks >= MIN_TICKS_PIVO);
    return movimentoConfirmado &&
           ((yawAbs >= (anguloTotalGraus - PIVO_ANTECIPACAO_GRAUS)) ||
            (avgTicks >= (int32_t)(targetTicks * PIVO_FALLBACK_FATOR)));
}

#endif
