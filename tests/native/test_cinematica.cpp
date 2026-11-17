// Testes nativos (sem hardware) da cinematica e do feedforward do firmware.
//
// Inclui os headers REAIS do firmware (config.h, robo_matematica.h,
// pid_controller.h) — o stub tests/native/Arduino.h supre a unica
// dependencia de Arduino. Nada de copia de formula: o que falha aqui
// e o codigo que vai para o robo.
//
// Roda em tests/native/rodar.py (ou direto: g++ -I tests/native -I
// firmware_robo2wd_v4 test_cinematica.cpp -o t && ./t).

#include <cstdio>
#include <cmath>

#include "config.h"
#include "robo_matematica.h"
#include "pid_controller.h"

static int _falhas = 0;
static int _testes = 0;

static void verificar(const char* nome, bool cond, const char* detalhe = "") {
    _testes++;
    if (cond) {
        std::printf("  ok   %s\n", nome);
    } else {
        _falhas++;
        std::printf("  FALHA %s  %s\n", nome, detalhe);
    }
}

static bool perto(float a, float b, float tol = 0.01f) {
    return std::fabs(a - b) <= tol;
}

// -------------------------------------------------------------------------
int main() {
    std::printf("== cinematica ==\n");

    // mm por pulso: circunferencia / PPR
    float esperado = (PI * DIAMETRO_RODA_MM) / PPR_EFETIVO;
    verificar("MM_POR_PULSO calculado", perto(MM_POR_PULSO, esperado, 1e-6f),
              "diametro ou PPR divergente");
    verificar("MM_POR_PULSO na faixa esperada", MM_POR_PULSO > 0.047f && MM_POR_PULSO < 0.049f,
              "valor muito fora do calibrado (roda 66 mm, 4320 ppr)");

    // Uma reta de 500 mm deve dar ~10417 ticks (homologado).
    verificar("500 mm -> 10417 ticks", ticksParaMm(500.0f) == 10417,
              "truncamento diferente do esperado");

    // Pivo de 90 graus: arco = pi * 120 * 0.25 = 94,248 mm -> 1963 ticks.
    verificar("pivo 90 graus -> 1963 ticks", ticksParaArcoPivo(90.0f) == 1963,
              "bitola ou truncamento divergente");

    // Consistencia: os dois lados do quadrado tem que "fechar" em ticks.
    verificar("4 x lado = 4 x 500 mm em ticks",
              4 * ticksParaMm(500.0f) == 41668, "perimetro inconsistente");

    std::printf("== feedforward (curva PWM x RPM) ==\n");

    // Fora da curva: satura nos extremos.
    verificar("curva satura em 1600 us abaixo de 0 RPM", curvaPWM_paraRPM(-10.0f) == 1600.0f);
    verificar("curva satura em 1850 us acima do fim", curvaPWM_paraRPM(200.0f) == 1850.0f);

    // Pontos medidos em bancada devem ser reproduzidos exatamente.
    verificar("curva no ponto 35 RPM -> 1650 us", perto(curvaPWM_paraRPM(35.0f), 1650.0f));
    verificar("curva no ponto 65 RPM -> 1700 us", perto(curvaPWM_paraRPM(65.0f), 1700.0f));
    verificar("curva no ponto 86.7 RPM -> 1750 us", perto(curvaPWM_paraRPM(86.7f), 1750.0f));

    // Interpolacao linear entre 35 (1650) e 65 (1700): 55 RPM -> 1683,33 us.
    float em55 = curvaPWM_paraRPM(55.0f);
    verificar("55 RPM interpolado", perto(em55, 1683.333f, 0.05f), "interpolacao divergente");

    // Feedforward real usado pelo PID para 55 RPM com fator de carga 0.80:
    // alvo na curva = 55/0.8 = 68.75 RPM -> ~1708,6 us -> ~208,6 us acima do neutro.
    float ff = curvaPWM_paraRPM(VELOCIDADE_CRUZEIRO_RPM / CURVA_FATOR_CARGA) - ESC_NEUTRO_US;
    verificar("feedforward de 55 RPM sob carga", perto(ff, 208.6f, 1.5f),
              "feedforward fora do calibrado (afeta a velocidade final)");

    std::printf("\n%d testes, %d falhas\n", _testes, _falhas);
    return _falhas == 0 ? 0 : 1;
}
