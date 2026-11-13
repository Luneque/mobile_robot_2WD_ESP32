/*
 * diag_motor_curve.ino - Levantamento da curva PWM x RPM dos motores (bancada)
 *
 * Proposito:
 *   Medir a velocidade real de cada motor em funcao do pulso PWM aplicado,
 *   para responder a questao central da primeira prioridade: o motor atinge
 *   os 55 RPM de cruzeiro com o ESC calibrado em 1850 us, ou o limite e
 *   o hardware (ESC/bateria) e nao o controle (PID)?
 *
 * Procedimento:
 *   1. Coloque o robô SUSPENSO (rodas livres, sem contato com o solo).
 *   2. Grave este sketch no ESP32 e abra o monitor serial em 115200 baud.
 *   3. O sketch arma o ESC (3 s em neutro) e executa a varredura automatica.
 *   4. Copie a tabela PWM x RPM (saida em CSV) para uma planilha e trace
 *      o grafico. O valor de PWM que cruza 55 RPM e o dado que decide a
 *      correção: se estiver abaixo de 1850 us, o problema e o PID; se
 *      estiver acima, o ESC precisa ser recalibrado (ou o alvo reduzido).
 *
 * Conexoes (mesmas do firmware V4):
 *   ESC CH1 (Motor 1 - Esquerdo): GPIO 19
 *   ESC CH2 (Motor 2 - Direito):  GPIO 18
 *   Encoder 1 (Esq) A/B: GPIO 16 / 17
 *   Encoder 2 (Dir) A/B: GPIO 33 / 32
 *
 * Comandos via Serial:
 *   'r' -> Reexecuta a varredura completa.
 *   'z' -> Zera os contadores de encoder.
 *
 * Taxa da Serial: 115200 baud
 */

#include <Arduino.h>
#include "driver/pulse_cnt.h"
#include "driver/mcpwm_prelude.h"

// ==================== MAPEAMENTO DE PINOS ====================
#define PIN_ESC_CH1 19
#define PIN_ESC_CH2 18
#define PIN_ENC1_A  16
#define PIN_ENC1_B  17
#define PIN_ENC2_A  33
#define PIN_ENC2_B  32

// ==================== CONFIGURACAO ====================
#define PPR_EFETIVO   4320.0f   // 12 PPR * 90 (reducao) * 4 (quadratura)
#define ESC_NEUTRO_US 1500
#define ESC_MIN_US    1150
#define ESC_MAX_US    1850
#define ARM_MS        3000      // tempo de armado do ESC em neutro
#define SETTLE_MS     2000      // tempo para a velocidade estabilizar
#define MEASURE_MS    1000      // janela de medicao do RPM
#define REST_MS       800       // repouso entre medidas

// ==================== MCPWM (geracao de pulso) ====================
static mcpwm_timer_handle_t _timer = NULL;
static mcpwm_oper_handle_t  _oper1 = NULL, _oper2 = NULL;
static mcpwm_cmpr_handle_t  _cmpr1 = NULL, _cmpr2 = NULL;
static mcpwm_gen_handle_t   _gen1 = NULL, _gen2 = NULL;
static bool _pwmOk = false;

static bool setupPWM() {
    mcpwm_timer_config_t timer_cfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,           // 1 MHz = 1 tick por microssegundo
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = 20000,              // 20 ms = 50 Hz
    };
    if (mcpwm_new_timer(&timer_cfg, &_timer) != ESP_OK) return false;

    mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
    if (mcpwm_new_operator(&oper_cfg, &_oper1) != ESP_OK) return false;
    if (mcpwm_new_operator(&oper_cfg, &_oper2) != ESP_OK) return false;
    if (mcpwm_operator_connect_timer(_oper1, _timer) != ESP_OK) return false;
    if (mcpwm_operator_connect_timer(_oper2, _timer) != ESP_OK) return false;

    mcpwm_comparator_config_t cmpr_cfg = { .flags = { .update_cmp_on_tez = true } };
    if (mcpwm_new_comparator(_oper1, &cmpr_cfg, &_cmpr1) != ESP_OK) return false;
    if (mcpwm_new_comparator(_oper2, &cmpr_cfg, &_cmpr2) != ESP_OK) return false;
    mcpwm_comparator_set_compare_value(_cmpr1, ESC_NEUTRO_US);
    mcpwm_comparator_set_compare_value(_cmpr2, ESC_NEUTRO_US);

    mcpwm_generator_config_t gen1_cfg = { .gen_gpio_num = PIN_ESC_CH1 };
    mcpwm_generator_config_t gen2_cfg = { .gen_gpio_num = PIN_ESC_CH2 };
    if (mcpwm_new_generator(_oper1, &gen1_cfg, &_gen1) != ESP_OK) return false;
    if (mcpwm_new_generator(_oper2, &gen2_cfg, &_gen2) != ESP_OK) return false;

    mcpwm_generator_set_action_on_timer_event(_gen1, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(_gen1, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, _cmpr1, MCPWM_GEN_ACTION_LOW));
    mcpwm_generator_set_action_on_timer_event(_gen2, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(_gen2, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, _cmpr2, MCPWM_GEN_ACTION_LOW));

    if (mcpwm_timer_enable(_timer) != ESP_OK) return false;
    if (mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP) != ESP_OK) return false;

    _pwmOk = true;
    return true;
}

static void setPWM(uint32_t us) {
    if (!_pwmOk) return;
    if (us < ESC_MIN_US) us = ESC_MIN_US;
    if (us > ESC_MAX_US) us = ESC_MAX_US;
    mcpwm_comparator_set_compare_value(_cmpr1, us);
    mcpwm_comparator_set_compare_value(_cmpr2, us);
}

static void stopMotors() {
    if (!_pwmOk) return;
    mcpwm_comparator_set_compare_value(_cmpr1, ESC_NEUTRO_US);
    mcpwm_comparator_set_compare_value(_cmpr2, ESC_NEUTRO_US);
}

// ==================== PCNT (leitura de encoder) ====================
static pcnt_unit_handle_t _pcnt1 = NULL, _pcnt2 = NULL;
static bool _pcntOk = false;

static bool setupPCNT() {
    pcnt_unit_config_t unit_cfg = {
        .low_limit = -30000,
        .high_limit = 30000,
        .flags = { .accum_count = true },
    };
    if (pcnt_new_unit(&unit_cfg, &_pcnt1) != ESP_OK) return false;
    if (pcnt_new_unit(&unit_cfg, &_pcnt2) != ESP_OK) return false;

    pcnt_glitch_filter_config_t filter_cfg = { .max_glitch_ns = 1000 };
    if (pcnt_unit_set_glitch_filter(_pcnt1, &filter_cfg) != ESP_OK) return false;
    if (pcnt_unit_set_glitch_filter(_pcnt2, &filter_cfg) != ESP_OK) return false;

    // --- Motor 1 (Esquerdo) ---
    pcnt_chan_config_t c1a_cfg = { .edge_gpio_num = PIN_ENC1_A, .level_gpio_num = PIN_ENC1_B };
    pcnt_chan_config_t c1b_cfg = { .edge_gpio_num = PIN_ENC1_B, .level_gpio_num = PIN_ENC1_A };
    pcnt_channel_handle_t ch1a = NULL, ch1b = NULL, ch2a = NULL, ch2b = NULL;
    if (pcnt_new_channel(_pcnt1, &c1a_cfg, &ch1a) != ESP_OK) return false;
    if (pcnt_new_channel(_pcnt1, &c1b_cfg, &ch1b) != ESP_OK) return false;

    pcnt_channel_set_edge_action(ch1a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(ch1a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(ch1b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(ch1b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    // --- Motor 2 (Direito) ---
    pcnt_chan_config_t c2a_cfg = { .edge_gpio_num = PIN_ENC2_A, .level_gpio_num = PIN_ENC2_B };
    pcnt_chan_config_t c2b_cfg = { .edge_gpio_num = PIN_ENC2_B, .level_gpio_num = PIN_ENC2_A };
    if (pcnt_new_channel(_pcnt2, &c2a_cfg, &ch2a) != ESP_OK) return false;
    if (pcnt_new_channel(_pcnt2, &c2b_cfg, &ch2b) != ESP_OK) return false;

    pcnt_channel_set_edge_action(ch2a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(ch2a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(ch2b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(ch2b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pinMode(PIN_ENC1_A, INPUT_PULLUP);
    pinMode(PIN_ENC1_B, INPUT_PULLUP);
    pinMode(PIN_ENC2_A, INPUT_PULLUP);
    pinMode(PIN_ENC2_B, INPUT_PULLUP);

    if (pcnt_unit_enable(_pcnt1) != ESP_OK) return false;
    if (pcnt_unit_enable(_pcnt2) != ESP_OK) return false;
    pcnt_unit_clear_count(_pcnt1);
    pcnt_unit_clear_count(_pcnt2);
    if (pcnt_unit_start(_pcnt1) != ESP_OK) return false;
    if (pcnt_unit_start(_pcnt2) != ESP_OK) return false;

    _pcntOk = true;
    return true;
}

static void resetCounters() {
    if (!_pcntOk) return;
    pcnt_unit_clear_count(_pcnt1);
    pcnt_unit_clear_count(_pcnt2);
}

static void getTicks(int32_t &t1, int32_t &t2) {
    int c1 = 0, c2 = 0;
    if (_pcntOk) {
        pcnt_unit_get_count(_pcnt1, &c1);
        pcnt_unit_get_count(_pcnt2, &c2);
    }
    t1 = c1;
    t2 = c2;
}

// ==================== VARREDURA PWM x RPM ====================
static void runSweep() {
    const uint32_t pwms[] = {1500, 1550, 1600, 1650, 1700, 1750, 1800, 1850};
    const int n = sizeof(pwms) / sizeof(pwms[0]);

    Serial.println("\n=============================================");
    Serial.println("   CURVA PWM x RPM  (bancada, robo suspenso)");
    Serial.println("=============================================");
    Serial.println("PWM_us,RPM_esq,RPM_dir");

    for (int i = 0; i < n; i++) {
        uint32_t pwm = pwms[i];

        resetCounters();
        setPWM(pwm);
        delay(SETTLE_MS);                 // estabiliza a velocidade

        int32_t t0a, t0b;
        getTicks(t0a, t0b);
        unsigned long tStart = millis();
        delay(MEASURE_MS);                // janela de medicao
        int32_t t1a, t1b;
        getTicks(t1a, t1b);
        unsigned long tEnd = millis();

        float dt  = (tEnd - tStart) / 1000.0f;
        float rpm1 = ((float)(t1a - t0a) / PPR_EFETIVO) * (60.0f / dt);
        float rpm2 = ((float)(t1b - t0b) / PPR_EFETIVO) * (60.0f / dt);

        Serial.printf("%lu,%.1f,%.1f\n", pwm, rpm1, rpm2);

        stopMotors();
        delay(REST_MS);
    }

    Serial.println("=============================================");
    Serial.println("Varredura concluida. Envie 'r' para repetir.");
    Serial.println("=============================================\n");
}

// ==================== SETUP / LOOP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=============================================");
    Serial.println("   DIAGNOSTICO DE CURVA PWM x RPM");
    Serial.println("=============================================");
    Serial.println("O robo deve estar SUSPENSO (rodas livres).");
    Serial.println("Comandos: 'r' reexecuta | 'z' zera contadores");
    Serial.println("=============================================\n");

    if (!setupPWM()) {
        Serial.println("[ERRO] Falha ao inicializar o MCPWM!");
        return;
    }
    if (!setupPCNT()) {
        Serial.println("[ERRO] Falha ao inicializar o PCNT!");
        return;
    }
    Serial.println("[OK] MCPWM e PCNT inicializados.");

    Serial.printf("Armando o ESC por %d ms em neutro...\n", ARM_MS);
    setPWM(ESC_NEUTRO_US);
    delay(ARM_MS);
    Serial.println("[OK] ESC armado.\n");

    runSweep();
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'r' || c == 'R') {
            runSweep();
        } else if (c == 'z' || c == 'Z') {
            resetCounters();
            Serial.println(">>> Contadores zerados.");
        }
    }
}
