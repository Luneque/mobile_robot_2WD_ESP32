#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include "config.h"

// =========================================================================
// DECODIFICAÇÃO EM QUADRATURA 4X COMPLETA
// Ambos os canais A e B disparam interrupções em CHANGE.
// Cada transição é decodificada através da tabela de estados de quadratura:
// estado = (A_anterior << 3) | (B_anterior << 2) | (A_atual << 1) | B_atual
// Tabela de transição padrão:
// +1 = avanço horário
// -1 = recuo anti-horário
//  0 = estado inválido / ruído / sem alteração
// =========================================================================

static volatile int32_t enc1_ticks = 0;
static volatile int32_t enc2_ticks = 0;

static volatile uint8_t enc1_state = 0;
static volatile uint8_t enc2_state = 0;

// Tabela de transição de quadratura (lookup table)
// Estados válidos de avanço: 0->1->3->2->0 (00 -> 01 -> 11 -> 10 -> 00)
static const int8_t QUAD_TABLE[16] = {
     0,  // 0000: sem mudança
    +1,  // 0001: 00 -> 01 (+1)
    -1,  // 0010: 00 -> 10 (-1)
     0,  // 0011: erro (dupla mudança)
    -1,  // 0100: 01 -> 00 (-1)
     0,  // 0101: sem mudança
     0,  // 0110: erro (dupla mudança)
    +1,  // 0111: 01 -> 11 (+1)
    +1,  // 1000: 10 -> 00 (+1)
     0,  // 1001: erro (dupla mudança)
     0,  // 1010: sem mudança
    -1,  // 1011: 10 -> 11 (-1)
     0,  // 1100: erro (dupla mudança)
    -1,  // 1101: 11 -> 01 (-1)
    +1,  // 1110: 11 -> 10 (+1)
     0   // 1111: sem mudança
};

// ISR do Motor 1 (Esquerdo) - Acionada em CHANGE de A ou B
static void IRAM_ATTR isr_enc1_change() {
    uint8_t a = digitalRead(PIN_ENC1_A);
    uint8_t b = digitalRead(PIN_ENC1_B);
    uint8_t current = (a << 1) | b;
    uint8_t index = ((enc1_state & 0x03) << 2) | current;
    enc1_state = current;
    
    // Incrementa ou decrementa os ticks
    enc1_ticks += QUAD_TABLE[index];
}

// ISR do Motor 2 (Direito) - Montagem física invertida em relação ao esquerdo
static void IRAM_ATTR isr_enc2_change() {
    uint8_t a = digitalRead(PIN_ENC2_A);
    uint8_t b = digitalRead(PIN_ENC2_B);
    uint8_t current = (a << 1) | b;
    uint8_t index = ((enc2_state & 0x03) << 2) | current;
    enc2_state = current;

    // Invertido mecanicamente (-QUAD_TABLE) para que avanço gere valor positivo
    enc2_ticks -= QUAD_TABLE[index];
}

class EncoderReader {
public:
    void begin() {
        pinMode(PIN_ENC1_A, INPUT_PULLUP);
        pinMode(PIN_ENC1_B, INPUT_PULLUP);
        pinMode(PIN_ENC2_A, INPUT_PULLUP);
        pinMode(PIN_ENC2_B, INPUT_PULLUP);

        // Estado inicial
        enc1_state = (digitalRead(PIN_ENC1_A) << 1) | digitalRead(PIN_ENC1_B);
        enc2_state = (digitalRead(PIN_ENC2_A) << 1) | digitalRead(PIN_ENC2_B);

        // Interrupções em CHANGE em ambos os pinos A e B (Quadratura 4x)
        attachInterrupt(digitalPinToInterrupt(PIN_ENC1_A), isr_enc1_change, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_ENC1_B), isr_enc1_change, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_ENC2_A), isr_enc2_change, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_ENC2_B), isr_enc2_change, CHANGE);

        reset();
    }

    void reset() {
        noInterrupts();
        enc1_ticks = 0;
        enc2_ticks = 0;
        interrupts();
        _lastTicks1 = 0;
        _lastTicks2 = 0;
        _currentRPM1 = 0.0f;
        _currentRPM2 = 0.0f;
    }

    void updateSpeed(float dt_sec) {
        int32_t current1, current2;
        noInterrupts();
        current1 = enc1_ticks;
        current2 = enc2_ticks;
        interrupts();

        int32_t delta1 = current1 - _lastTicks1;
        int32_t delta2 = current2 - _lastTicks2;
        _lastTicks1 = current1;
        _lastTicks2 = current2;

        if (dt_sec > 0.001f) {
            // RPM = (delta_pulsos / PPR_EFETIVO) * (60.0 / dt_sec)
            _currentRPM1 = (((float)delta1) / PPR_EFETIVO) * (60.0f / dt_sec);
            _currentRPM2 = (((float)delta2) / PPR_EFETIVO) * (60.0f / dt_sec);
        }
    }

    int32_t getTicksM1() {
        noInterrupts();
        int32_t t = enc1_ticks;
        interrupts();
        return t;
    }

    int32_t getTicksM2() {
        noInterrupts();
        int32_t t = enc2_ticks;
        interrupts();
        return t;
    }

    float getRPM1() const { return _currentRPM1; }
    float getRPM2() const { return _currentRPM2; }

    float getDistanceM1_mm() { return ((float)getTicksM1()) * MM_POR_PULSO; }
    float getDistanceM2_mm() { return ((float)getTicksM2()) * MM_POR_PULSO; }

private:
    int32_t _lastTicks1 = 0;
    int32_t _lastTicks2 = 0;
    float _currentRPM1 = 0.0f;
    float _currentRPM2 = 0.0f;
};

#endif // ENCODER_READER_H
