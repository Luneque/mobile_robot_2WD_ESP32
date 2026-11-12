#ifndef MOTOR_CONTROL_MCPWM_H
#define MOTOR_CONTROL_MCPWM_H

#include <Arduino.h>
#include "config.h"
#include "driver/mcpwm_prelude.h"

// =========================================================================
// CONTROLE DE MOTORES VIA MCPWM (Motor Control Pulse Width Modulator)
// Hardware dedicado de geração de pulsos com timer de 1 MHz (1 tick = 1 us).
// Para 50 Hz, o período é exatamente 20.000 us (20.000 ticks).
// O comparator ajusta a largura do pulso diretamente em microssegundos (us).
// =========================================================================

class MotorControlMCPWM {
public:
    MotorControlMCPWM() : _timer(NULL), _oper1(NULL), _oper2(NULL),
                          _cmpr1(NULL), _cmpr2(NULL), _gen1(NULL), _gen2(NULL), _initialized(false) {}

    bool begin() {
        // 1. Configurar o Timer do MCPWM (Resolução de 1 MHz -> 1 us por tick)
        mcpwm_timer_config_t timer_config = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 1000000,           // 1 MHz = 1 tick por microssegundo
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = 20000,              // 20.000 ticks = 20 ms = 50 Hz
        };
        esp_err_t err = mcpwm_new_timer(&timer_config, &_timer);
        if (err != ESP_OK) return false;

        // 2. Configurar os Operadores do MCPWM (um para cada motor)
        mcpwm_operator_config_t oper_config = {
            .group_id = 0,
        };
        err = mcpwm_new_operator(&oper_config, &_oper1);
        if (err != ESP_OK) return false;

        err = mcpwm_new_operator(&oper_config, &_oper2);
        if (err != ESP_OK) return false;

        err = mcpwm_operator_connect_timer(_oper1, _timer);
        if (err != ESP_OK) return false;

        err = mcpwm_operator_connect_timer(_oper2, _timer);
        if (err != ESP_OK) return false;

        // 3. Configurar Comparadores para cada canal
        mcpwm_comparator_config_t cmpr_config = {
            .flags = {
                .update_cmp_on_tez = true,
            },
        };
        err = mcpwm_new_comparator(_oper1, &cmpr_config, &_cmpr1);
        if (err != ESP_OK) return false;

        err = mcpwm_new_comparator(_oper2, &cmpr_config, &_cmpr2);
        if (err != ESP_OK) return false;

        // Inicializa comparadores em neutro (1500 us)
        mcpwm_comparator_set_compare_value(_cmpr1, ESC_NEUTRO_US);
        mcpwm_comparator_set_compare_value(_cmpr2, ESC_NEUTRO_US);

        // 4. Configurar Geradores ligados aos GPIOs do ESC
        mcpwm_generator_config_t gen1_config = {
            .gen_gpio_num = PIN_ESC_CH1,
        };
        mcpwm_generator_config_t gen2_config = {
            .gen_gpio_num = PIN_ESC_CH2,
        };
        err = mcpwm_new_generator(_oper1, &gen1_config, &_gen1);
        if (err != ESP_OK) return false;

        err = mcpwm_new_generator(_oper2, &gen2_config, &_gen2);
        if (err != ESP_OK) return false;

        // Ações: Nível Alto ao zerar o timer (TEZ), Nível Baixo ao atingir o comparador (CMP)
        mcpwm_generator_set_action_on_timer_event(_gen1,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        mcpwm_generator_set_action_on_compare_event(_gen1,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, _cmpr1, MCPWM_GEN_ACTION_LOW));

        mcpwm_generator_set_action_on_timer_event(_gen2,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        mcpwm_generator_set_action_on_compare_event(_gen2,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, _cmpr2, MCPWM_GEN_ACTION_LOW));

        // 5. Habilitar e iniciar o Timer
        err = mcpwm_timer_enable(_timer);
        if (err != ESP_OK) return false;

        err = mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP);
        if (err != ESP_OK) return false;

        _initialized = true;
        return true;
    }

    void setMicrosecondsM1(uint32_t us) {
        if (!_initialized || !_cmpr1) return;
        if (us < ESC_MIN_US) us = ESC_MIN_US;
        if (us > ESC_MAX_US) us = ESC_MAX_US;
        mcpwm_comparator_set_compare_value(_cmpr1, us);
    }

    void setMicrosecondsM2(uint32_t us) {
        if (!_initialized || !_cmpr2) return;
        if (us < ESC_MIN_US) us = ESC_MIN_US;
        if (us > ESC_MAX_US) us = ESC_MAX_US;
        mcpwm_comparator_set_compare_value(_cmpr2, us);
    }

    void stopAll() {
        if (!_initialized) return;
        if (_cmpr1) mcpwm_comparator_set_compare_value(_cmpr1, ESC_NEUTRO_US);
        if (_cmpr2) mcpwm_comparator_set_compare_value(_cmpr2, ESC_NEUTRO_US);
    }

    bool isInitialized() const { return _initialized; }

private:
    mcpwm_timer_handle_t      _timer;
    mcpwm_oper_handle_t       _oper1, _oper2;
    mcpwm_cmpr_handle_t       _cmpr1, _cmpr2;
    mcpwm_gen_handle_t        _gen1, _gen2;
    bool                      _initialized;
};

#endif // MOTOR_CONTROL_MCPWM_H
