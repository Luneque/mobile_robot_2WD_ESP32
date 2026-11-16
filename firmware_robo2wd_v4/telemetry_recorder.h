/**
 * telemetry_recorder.h — Gravador de telemetria em RAM com replay (V4)
 *
 * Motivação: no teste com o robô no solo não há como manter o cabo serial
 * conectado. Este módulo grava amostras em RAM durante a execução e as
 * despeja via serial (CSV) quando solicitado, permitindo registrar o teste
 * completo e analisá-lo depois.
 *
 * Uso:
 *   TelemetryRecorder rec;
 *   rec.begin();                       // no setup()
 *   rec.sample(pkt, estadoStr);        // no loop, a cada REC_INTERVAL_MS
 *   rec.dump();                        // despeja o buffer via Serial
 *
 * Comandos que disparam o dump:
 *   - Serial: caractere 'd'
 *   - WebSocket: "dump"
 *
 * Formato do dump (uma linha por amostra):
 *   REC,<ts>,<estado>,<lado>,<yaw>,<rpm1>,<rpm2>,<ticks1>,<ticks2>,<ticksAlvo>,<pwm1>,<pwm2>,<rpmAlvo>
 */

#ifndef TELEMETRY_RECORDER_H
#define TELEMETRY_RECORDER_H

#include <Arduino.h>
#include "config.h"

// ─────────────────────────────────────────────────────────────────────────────
// Configuração do gravador
// ─────────────────────────────────────────────────────────────────────────────
#ifndef REC_INTERVAL_MS
#define REC_INTERVAL_MS     50      // 20 Hz de gravação
#endif
#ifndef REC_MAX_SAMPLES
#define REC_MAX_SAMPLES     2000    // 2000 amostras a 20 Hz = 100 s de teste
#endif

// Cabeçalho CSV do dump (compatível com o logger WebSocket)
static const char REC_CSV_HEADER[] =
    "timestamp_ms,estado,lado,yaw_deg,rpm_esq,rpm_dir,ticks_esq,ticks_dir,"
    "ticks_alvo,pwm_esq_us,pwm_dir_us,rpm_alvo";

// ─────────────────────────────────────────────────────────────────────────────
// Amostra gravada (compacta: 40 bytes)
// ─────────────────────────────────────────────────────────────────────────────
struct RecSample {
    uint32_t ts;        // millis() desde o boot
    uint8_t  estado;    // código de estado (ver REC_EST_*)
    uint8_t  lado;      // segmento atual (1–4)
    int16_t  yaw;       // graus * 100 (precisão de 0,01°)
    int16_t  rpm1;      // RPM * 10
    int16_t  rpm2;      // RPM * 10
    int32_t  ticks1;
    int32_t  ticks2;
    int32_t  ticksAlvo;
    uint16_t pwm1;      // µs
    uint16_t pwm2;      // µs
    int16_t  rpmAlvo;   // RPM * 10
};

// Códigos de estado (compatíveis com o texto da telemetria)
enum RecEstado {
    REC_EST_FALHA = 0,
    REC_EST_ARMANDO,
    REC_EST_AGUARDO,
    REC_EST_RETA,
    REC_EST_PIVO,
    REC_EST_PAUSA,
    REC_EST_CONCLUIDO,
    REC_EST_EMERGENCIA
};

static inline const char* recEstadoTxt(uint8_t e) {
    switch (e) {
        case REC_EST_FALHA:     return "FALHA";
        case REC_EST_ARMANDO:   return "ARMANDO";
        case REC_EST_AGUARDO:   return "AGUARDO";
        case REC_EST_RETA:      return "RETA";
        case REC_EST_PIVO:      return "PIVO";
        case REC_EST_PAUSA:     return "PAUSA";
        case REC_EST_CONCLUIDO: return "CONCLUIDO";
        case REC_EST_EMERGENCIA:return "EMERGENCIA";
        default:                return "AGUARDO";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Classe gravadora
// ─────────────────────────────────────────────────────────────────────────────
class TelemetryRecorder {
public:
    void begin() {
        _count = 0;
        _head = 0;
        _overflow = false;
        _lastSample = 0;
        _recording = true;
    }

    // Grava uma amostra se o intervalo mínimo já passou.
    void sample(uint32_t now, uint8_t estado, uint8_t lado,
                float yaw, float rpm1, float rpm2,
                int32_t ticks1, int32_t ticks2, int32_t ticksAlvo,
                uint32_t pwm1, uint32_t pwm2, float rpmAlvo) {
        if (!_recording) return;
        if (now - _lastSample < REC_INTERVAL_MS) return;
        _lastSample = now;

        RecSample& s = _buf[_head];
        s.ts        = now;
        s.estado    = estado;
        s.lado      = lado;
        s.yaw       = (int16_t)lroundf(yaw * 100.0f);
        s.rpm1      = (int16_t)lroundf(rpm1 * 10.0f);
        s.rpm2      = (int16_t)lroundf(rpm2 * 10.0f);
        s.ticks1    = ticks1;
        s.ticks2    = ticks2;
        s.ticksAlvo = ticksAlvo;
        s.pwm1      = (uint16_t)pwm1;
        s.pwm2      = (uint16_t)pwm2;
        s.rpmAlvo   = (int16_t)lroundf(rpmAlvo * 10.0f);

        _head++;
        if (_head >= REC_MAX_SAMPLES) {
            _head = 0;
            _overflow = true;
        }
        if (_count < REC_MAX_SAMPLES) _count++;
    }

    // Limpa o buffer (usado ao reiniciar a trajetória)
    void clear() {
        _count = 0;
        _head = 0;
        _overflow = false;
        _lastSample = 0;
    }

    // Reinicia a gravação a partir de "agora" (evita gravar a pausa pós-teste)
    void restart(uint32_t now) {
        _count = 0;
        _head = 0;
        _overflow = false;
        _lastSample = now;
    }

    // Despeja o buffer via Serial em CSV, uma amostra por linha.
    uint16_t dump() {
        uint16_t n = _count;
        Serial.println(F("\n[REC] Iniciando dump do gravador de telemetria..."));
        Serial.print(F("[REC] Amostras: "));
        Serial.print(n);
        if (_overflow) Serial.print(F(" (buffer circular: descartou as mais antigas)"));
        Serial.println();
        Serial.println(REC_CSV_HEADER);

        uint16_t start = _overflow ? _head : 0;
        for (uint16_t i = 0; i < n; i++) {
            uint16_t idx = (uint16_t)((start + i) % REC_MAX_SAMPLES);
            const RecSample& s = _buf[idx];
            Serial.print(F("REC,"));
            Serial.print(s.ts);        Serial.print(',');
            Serial.print(recEstadoTxt(s.estado)); Serial.print(',');
            Serial.print(s.lado);      Serial.print(',');
            Serial.print(s.yaw / 100.0f, 2);  Serial.print(',');
            Serial.print(s.rpm1 / 10.0f, 1);  Serial.print(',');
            Serial.print(s.rpm2 / 10.0f, 1);  Serial.print(',');
            Serial.print(s.ticks1);    Serial.print(',');
            Serial.print(s.ticks2);    Serial.print(',');
            Serial.print(s.ticksAlvo); Serial.print(',');
            Serial.print(s.pwm1);      Serial.print(',');
            Serial.print(s.pwm2);      Serial.print(',');
            Serial.println(s.rpmAlvo / 10.0f, 1);
        }
        Serial.println(F("[REC] Dump concluido."));
        return n;
    }

    // Serializa uma amostra em CSV dentro do buffer fornecido (para envio via WS).
    // Retorna o número de caracteres escritos.
    size_t dumpSample(uint16_t pos, char* out, size_t outLen) {
        if (outLen < 2) return 0;
        uint16_t n = _count;
        uint16_t idx;
        if (_overflow) {
            idx = (uint16_t)((_head + pos) % REC_MAX_SAMPLES);
        } else {
            idx = pos;
        }
        const RecSample& s = _buf[idx];
        int k = snprintf(out, outLen,
            "REC,%lu,%s,%u,%.2f,%.1f,%.1f,%ld,%ld,%ld,%u,%u,%.1f",
            (unsigned long)s.ts, recEstadoTxt(s.estado), (unsigned)s.lado,
            s.yaw / 100.0f, s.rpm1 / 10.0f, s.rpm2 / 10.0f,
            (long)s.ticks1, (long)s.ticks2, (long)s.ticksAlvo,
            (unsigned)s.pwm1, (unsigned)s.pwm2, s.rpmAlvo / 10.0f);
        if (k < 0) return 0;
        if ((size_t)k >= outLen) return outLen - 1;
        return (size_t)k;
    }

    uint16_t count() const { return _count; }
    bool     overflow() const { return _overflow; }
    void     setRecording(bool on) { _recording = on; }

private:
    RecSample _buf[REC_MAX_SAMPLES];
    uint16_t  _head = 0;
    uint16_t  _count = 0;
    bool      _overflow = false;
    uint32_t  _lastSample = 0;
    bool      _recording = true;
};

#endif // TELEMETRY_RECORDER_H
