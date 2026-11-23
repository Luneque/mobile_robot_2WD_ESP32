// Testes nativos do gravador de telemetria (telemetry_recorder.h).
//
// Foca no formato do dump e no tratamento de truncamento — a classe de bug que
// ja' corrompeu o CSV remoto sem aviso (revisao de engenharia, itens 2.2/2.3):
// o buffer da linha era 96 bytes, a linha real tem ~98-141, e o snprintf
// truncava a ultima coluna (rpmAlvo).
#include <stdio.h>
#include <string.h>

#include "telemetry_recorder.h"

static int falhas = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FALHA: %s\n", msg); falhas++; } } while (0)

int main() {
    // 1) O cabecalho CSV tem de ter exatamente 16 campos.
    int campos = 1;
    for (const char* p = REC_CSV_HEADER; *p; p++) {
        if (*p == ',') campos++;
    }
    CHECK(campos == 16, "cabecalho deve ter 16 campos");

    // 2) Amostra com os valores mais largos possiveis tem de caber em
    //    DUMP_LINE_MAX. Se um dia a linha passar do buffer, o teste aponta.
    TelemetryRecorder rec;
    rec.begin();
    rec.sample(4294967295u,               // ts com 10 digitos
               REC_EST_EMERGENCIA,         // estado com o maior nome
               4,
               -90.0f,                     // yaw
               3276.0f, -3276.0f,         // yawAbs, yawOdom
               3276.0f,                    // diverg
               327.67f,                    // gyroBias
               -3276.8f, 3276.8f,          // rpm
               -2147483647, 2147483647, 2147483647,  // ticks
               1150, 1850,                 // pwm
               3276.6f);                   // rpmAlvo

    char linha[DUMP_LINE_MAX];
    size_t n = rec.dumpSample(0, linha, sizeof(linha));
    CHECK(n > 0, "dumpSample nao escreveu nada");
    CHECK(n < sizeof(linha), "linha extrema nao cabe em DUMP_LINE_MAX (truncaria)");

    // 3) A linha tem de fechar com o cabecalho: "REC," + 16 campos.
    int partes = 1;
    for (size_t i = 0; i < n; i++) {
        if (linha[i] == ',') partes++;
    }
    CHECK(partes == 17, "linha deve ter REC + 16 campos");
    CHECK(strncmp(linha, "REC,", 4) == 0, "linha deve comecar com REC,");
    CHECK(strstr(linha, ",EMERGENCIA,") != NULL, "estado EMERGENCIA deveria aparecer na linha");

    // 4) Com buffer pequeno, o retorno sinaliza truncamento (>= outLen).
    //    Antes o codigo devolvia outLen-1 e o chamador achava que estava bom.
    char pequeno[40];
    size_t t = rec.dumpSample(0, pequeno, sizeof(pequeno));
    CHECK(t >= sizeof(pequeno), "retorno deve sinalizar truncamento quando nao coube");

    // 5) A ultima coluna e' o rpmAlvo formatado com 1 casa decimal.
    //    (rpmAlvo e' gravado como RPM*10: 3276.6 RPM vira 32766 e imprime 3276.6)
    char* p = linha + (n - 1);
    while (p > linha && *p != ',') p--;
    double ultimo = strtod(p + 1, NULL);
    CHECK(fabs(ultimo - 3276.6) < 0.05,
          "ultima coluna deveria ser o rpmAlvo (3276.6), veio outro valor");

    printf("test_recorder: %d falhas\n", falhas);
    return falhas == 0 ? 0 : 1;
}
