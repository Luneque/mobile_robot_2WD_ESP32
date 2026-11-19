// Testes nativos do criterio de parada dos segmentos (reta e pivo).
//
// Inclui os headers REAIS do firmware; o stub tests/native/Arduino.h supre a
// unica dependencia de Arduino. Os casos marcados como "fixture real" usam os
// valores observados em teste no solo (dump de 17/11/2026) — se alguem
// mudar a antecipacao ou a guarda, o teste mostra o deslocamento.
//
// Roda em tests/native/rodar.py.

#include <cstdio>

#include "config.h"
#include "robo_matematica.h"

static int _falhas = 0;
static int _testes = 0;

static void verificar(const char* nome, bool cond) {
    _testes++;
    std::printf("  %s %s\n", cond ? "ok  " : "FALHA", nome);
    if (!cond) _falhas++;
}

// alvo do pivo de 90 graus para a bitola/distancia do projeto
static const int32_t ALVO_PIVO = 1963;
static const int32_t ALVO_RETA = 10417;

int main() {
    std::printf("== criterios de parada (limiares derivados) ==\n");
    std::printf("  MIN_TICKS_PIVO = %d | anticipacao = %.1f deg | corte = %.2f deg | fallback = %d ticks\n\n",
                (int)MIN_TICKS_PIVO, (double)PIVO_ANTECIPACAO_GRAUS,
                (double)(ANGULO_CURVA_GRAUS - PIVO_ANTECIPACAO_GRAUS),
                (int)(ALVO_PIVO * PIVO_FALLBACK_FATOR));

    std::printf("== pivo com IMU ==\n");
    // Limiar de corte por yaw: 90° menos a antecipacao (compensa a inercia).
    const float LIMIAR = ANGULO_CURVA_GRAUS - PIVO_ANTECIPACAO_GRAUS;
    // A guarda vem primeiro: sem movimento confirmado, nem yaw cheio para.
    verificar("guarda: 150 ticks com yaw 89.9 nao para", !atingiuMetaPivo(150, 89.9f, ALVO_PIVO, ANGULO_CURVA_GRAUS, true));
    verificar("guarda: 199 ticks com yaw 95.0 nao para", !atingiuMetaPivo(199, 95.0f, ALVO_PIVO, ANGULO_CURVA_GRAUS, true));
    verificar("guarda exata: 200 ticks com yaw no limiar para", atingiuMetaPivo(200, LIMIAR + 0.01f, ALVO_PIVO, ANGULO_CURVA_GRAUS, true));

    // Corte por yaw (limiar derivado da antecipacao vigente).
    verificar("corte por yaw: 1500 ticks, yaw no limiar para", atingiuMetaPivo(1500, LIMIAR + 0.01f, ALVO_PIVO, ANGULO_CURVA_GRAUS, true));
    verificar("corte por yaw: 1500 ticks, yaw 0.01 abaixo nao para", !atingiuMetaPivo(1500, LIMIAR - 0.01f, ALVO_PIVO, ANGULO_CURVA_GRAUS, true));
    verificar("sem corte: 500 ticks, 45 deg nao para", !atingiuMetaPivo(500, 45.0f, ALVO_PIVO, ANGULO_CURVA_GRAUS, true));

    // Rede de seguranca por odometria (fallback 1.15x = 2257 ticks).
    verificar("fallback: 2257 ticks com yaw 80 deg para", atingiuMetaPivo(2257, 80.0f, ALVO_PIVO, ANGULO_CURVA_GRAUS, true));
    verificar("fallback: 2256 ticks com yaw 80 deg nao para", !atingiuMetaPivo(2256, 80.0f, ALVO_PIVO, ANGULO_CURVA_GRAUS, true));

    std::printf("\n== pivo sem IMU ==\n");
    verificar("sem IMU: 1963 ticks para", atingiuMetaPivo(ALVO_PIVO, 0.0f, ALVO_PIVO, ANGULO_CURVA_GRAUS, false));
    verificar("sem IMU: 1962 ticks nao para", !atingiuMetaPivo(ALVO_PIVO - 1, 0.0f, ALVO_PIVO, ANGULO_CURVA_GRAUS, false));

    std::printf("\n== reta ==\n");
    verificar("reta: 10417 ticks para", atingiuMetaReta(ALVO_RETA, ALVO_RETA));
    verificar("reta: 10416 ticks nao para", !atingiuMetaReta(ALVO_RETA - 1, ALVO_RETA));

    std::printf("\n== fixtures reais (pivos medidos no solo, 17/11/2026) ==\n");
    // Cada linha e o yaw congelado no instante do corte, com a antecipacao
    // vigente na epoca (0.7°). Com o limiar atual (~83.2°) todos continuam
    // acima, logo param — o que importa e' que o limiar nao esteja alto demais.
    verificar("real L1: 1969 ticks, 89.79 deg para", atingiuMetaPivo(1969, 89.79f, ALVO_PIVO, ANGULO_CURVA_GRAUS, true));
    verificar("real L2: 1973 ticks, 90.41 deg para", atingiuMetaPivo(1973, 90.41f, ALVO_PIVO, ANGULO_CURVA_GRAUS, true));
    verificar("real L3: 1988 ticks, 91.17 deg para", atingiuMetaPivo(1988, 91.17f, ALVO_PIVO, ANGULO_CURVA_GRAUS, true));
    verificar("real L4: 1983 ticks, 90.24 deg para", atingiuMetaPivo(1983, 90.24f, ALVO_PIVO, ANGULO_CURVA_GRAUS, true));
    // O criterio nao pode disparar antes do limiar.
    verificar("real L1 nao dispara antes: 1969 ticks, yaw 0.01 abaixo do limiar nao para",
              !atingiuMetaPivo(1969, LIMIAR - 0.01f, ALVO_PIVO, ANGULO_CURVA_GRAUS, true));

    std::printf("\n%d testes, %d falhas\n", _testes, _falhas);
    return _falhas == 0 ? 0 : 1;
}
