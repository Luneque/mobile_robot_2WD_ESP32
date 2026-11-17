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

#endif
