#ifndef ENCODER_READER_PCNT_H
#define ENCODER_READER_PCNT_H

#include <Arduino.h>
#include "config.h"
#include "driver/pulse_cnt.h"

// =========================================================================
// DECODIFICAÇÃO DE ENCODERS VIA HARDWARE PCNT (Pulse Counter)
// Os contadores do PCNT operam diretamente no silício do ESP32:
// - Modo Quadratura 4x: Canal 0 e Canal 1 monitoram as transições de A e B.
// - Nenhum overhead de CPU / sem interrupções por software para cada pulso.
// - Filtro digital de ruído (Glitch Filter) embutido por hardware.
// =========================================================================

class EncoderReaderPCNT {
public:
    EncoderReaderPCNT() : _unit1(NULL), _unit2(NULL), _chan1_a(NULL), _chan1_b(NULL),
                          _chan2_a(NULL), _chan2_b(NULL), _lastTicks1(0), _lastTicks2(0),
                          _currentRPM1(0.0f), _currentRPM2(0.0f), _initialized(false) {}

    bool begin() {
        // Configuração da Unidade PCNT para o Motor 1 (Esquerdo)
        pcnt_unit_config_t unit1_config = {
            .low_limit = -30000,
            .high_limit = 30000,
            .flags = {
                .accum_count = true,
            },
        };
        esp_err_t err = pcnt_new_unit(&unit1_config, &_unit1);
        if (err != ESP_OK) return false;

        // Configuração da Unidade PCNT para o Motor 2 (Direito)
        pcnt_unit_config_t unit2_config = {
            .low_limit = -30000,
            .high_limit = 30000,
            .flags = {
                .accum_count = true,
            },
        };
        err = pcnt_new_unit(&unit2_config, &_unit2);
        if (err != ESP_OK) return false;

        // Filtro de ruído digital de hardware (1000 ns = 1 us)
        pcnt_glitch_filter_config_t filter_config = {
            .max_glitch_ns = 1000,
        };
        err = pcnt_unit_set_glitch_filter(_unit1, &filter_config);
        if (err != ESP_OK) return false;

        err = pcnt_unit_set_glitch_filter(_unit2, &filter_config);
        if (err != ESP_OK) return false;

        // --- CANAIS DO MOTOR 1 (Esquerdo) ---
        pcnt_chan_config_t chan1_a_config = {
            .edge_gpio_num = PIN_ENC1_A,
            .level_gpio_num = PIN_ENC1_B,
        };
        err = pcnt_new_channel(_unit1, &chan1_a_config, &_chan1_a);
        if (err != ESP_OK) return false;

        pcnt_chan_config_t chan1_b_config = {
            .edge_gpio_num = PIN_ENC1_B,
            .level_gpio_num = PIN_ENC1_A,
        };
        err = pcnt_new_channel(_unit1, &chan1_b_config, &_chan1_b);
        if (err != ESP_OK) return false;

        pcnt_channel_set_edge_action(_chan1_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        pcnt_channel_set_level_action(_chan1_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

        pcnt_channel_set_edge_action(_chan1_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
        pcnt_channel_set_level_action(_chan1_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

        // --- CANAIS DO MOTOR 2 (Direito) ---
        pcnt_chan_config_t chan2_a_config = {
            .edge_gpio_num = PIN_ENC2_A,
            .level_gpio_num = PIN_ENC2_B,
        };
        err = pcnt_new_channel(_unit2, &chan2_a_config, &_chan2_a);
        if (err != ESP_OK) return false;

        pcnt_chan_config_t chan2_b_config = {
            .edge_gpio_num = PIN_ENC2_B,
            .level_gpio_num = PIN_ENC2_A,
        };
        err = pcnt_new_channel(_unit2, &chan2_b_config, &_chan2_b);
        if (err != ESP_OK) return false;

        pcnt_channel_set_edge_action(_chan2_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
        pcnt_channel_set_level_action(_chan2_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

        pcnt_channel_set_edge_action(_chan2_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        pcnt_channel_set_level_action(_chan2_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

        // Habilita resistores de pull-up internos nos GPIOs dos encoders
        pinMode(PIN_ENC1_A, INPUT_PULLUP);
        pinMode(PIN_ENC1_B, INPUT_PULLUP);
        pinMode(PIN_ENC2_A, INPUT_PULLUP);
        pinMode(PIN_ENC2_B, INPUT_PULLUP);

        err = pcnt_unit_enable(_unit1);
        if (err != ESP_OK) return false;

        err = pcnt_unit_enable(_unit2);
        if (err != ESP_OK) return false;

        reset();

        err = pcnt_unit_start(_unit1);
        if (err != ESP_OK) return false;

        err = pcnt_unit_start(_unit2);
        if (err != ESP_OK) return false;

        _initialized = true;
        return true;
    }

    void reset() {
        if (_unit1) pcnt_unit_clear_count(_unit1);
        if (_unit2) pcnt_unit_clear_count(_unit2);
        _lastTicks1 = 0;
        _lastTicks2 = 0;
        _currentRPM1 = 0.0f;
        _currentRPM2 = 0.0f;
    }

    void updateSpeed(float dt_sec) {
        if (!_initialized) return;
        int current1 = 0, current2 = 0;
        pcnt_unit_get_count(_unit1, &current1);
        pcnt_unit_get_count(_unit2, &current2);

        int delta1 = current1 - _lastTicks1;
        int delta2 = current2 - _lastTicks2;
        _lastTicks1 = current1;
        _lastTicks2 = current2;

        if (dt_sec > 0.001f) {
            float rawRPM1 = (((float)delta1) / PPR_EFETIVO) * (60.0f / dt_sec);
            float rawRPM2 = (((float)delta2) / PPR_EFETIVO) * (60.0f / dt_sec);

            // Filtro Passa-Baixas Exponencial (EMA)
            _currentRPM1 = (_currentRPM1 * 0.70f) + (rawRPM1 * 0.30f);
            _currentRPM2 = (_currentRPM2 * 0.70f) + (rawRPM2 * 0.30f);
        }
    }

    int32_t getTicksM1() {
        if (!_initialized || !_unit1) return 0;
        int count = 0;
        pcnt_unit_get_count(_unit1, &count);
        return count;
    }

    int32_t getTicksM2() {
        if (!_initialized || !_unit2) return 0;
        int count = 0;
        pcnt_unit_get_count(_unit2, &count);
        return count;
    }

    float getRPM1() const { return _currentRPM1; }
    float getRPM2() const { return _currentRPM2; }

    float getDistanceM1_mm() { return ((float)getTicksM1()) * MM_POR_PULSO; }
    float getDistanceM2_mm() { return ((float)getTicksM2()) * MM_POR_PULSO; }

    bool isInitialized() const { return _initialized; }

private:
    pcnt_unit_handle_t    _unit1, _unit2;
    pcnt_channel_handle_t _chan1_a, _chan1_b;
    pcnt_channel_handle_t _chan2_a, _chan2_b;
    int                   _lastTicks1, _lastTicks2;
    float                 _currentRPM1, _currentRPM2;
    bool                  _initialized;
};

#endif // ENCODER_READER_PCNT_H
