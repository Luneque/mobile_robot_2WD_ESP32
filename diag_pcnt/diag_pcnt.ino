/*
 * diag_pcnt.ino - Validador de Contagem e Polaridade com Hardware PCNT (ESP32)
 *
 * Mapeamento:
 *  Motor 1 (Esquerda): Fase A = GPIO 23, Fase B = GPIO 22
 *  Motor 2 (Direita):  Fase A = GPIO 33, Fase B = GPIO 32
 *
 * Taxa da Serial: 115200 baud
 */

#include <Arduino.h>
#include "driver/pulse_cnt.h"

#define PIN_ENC1_A 23
#define PIN_ENC1_B 22
#define PIN_ENC2_A 33
#define PIN_ENC2_B 32

// Calibração: 12 PPR * Redução 1:90 * Quadratura 4x = 4320 pulsos por volta
#define PULSOS_POR_VOLTA 4320.0f

#define PCNT_HIGH_LIMIT 30000
#define PCNT_LOW_LIMIT  -30000

pcnt_unit_handle_t pcnt_unit1 = NULL;
pcnt_unit_handle_t pcnt_unit2 = NULL;
pcnt_channel_handle_t chan1_a = NULL;
pcnt_channel_handle_t chan1_b = NULL;
pcnt_channel_handle_t chan2_a = NULL;
pcnt_channel_handle_t chan2_b = NULL;

int64_t g_accum1 = 0;
int64_t g_accum2 = 0;
int g_last_raw1 = 0;
int g_last_raw2 = 0;

void setupPCNT() {
    pcnt_unit_config_t unit1_config = {
        .low_limit = PCNT_LOW_LIMIT,
        .high_limit = PCNT_HIGH_LIMIT,
        .flags = { .accum_count = true }
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit1_config, &pcnt_unit1));

    pcnt_unit_config_t unit2_config = {
        .low_limit = PCNT_LOW_LIMIT,
        .high_limit = PCNT_HIGH_LIMIT,
        .flags = { .accum_count = true }
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit2_config, &pcnt_unit2));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit1, &filter_config));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit2, &filter_config));

    // Motor 1 (Esquerdo)
    pcnt_chan_config_t chan1_a_config = {
        .edge_gpio_num = PIN_ENC1_A,
        .level_gpio_num = PIN_ENC1_B,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit1, &chan1_a_config, &chan1_a));

    pcnt_chan_config_t chan1_b_config = {
        .edge_gpio_num = PIN_ENC1_B,
        .level_gpio_num = PIN_ENC1_A,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit1, &chan1_b_config, &chan1_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan1_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan1_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan1_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan1_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Motor 2 (Direito)
    pcnt_chan_config_t chan2_a_config = {
        .edge_gpio_num = PIN_ENC2_A,
        .level_gpio_num = PIN_ENC2_B,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit2, &chan2_a_config, &chan2_a));

    pcnt_chan_config_t chan2_b_config = {
        .edge_gpio_num = PIN_ENC2_B,
        .level_gpio_num = PIN_ENC2_A,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit2, &chan2_b_config, &chan2_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan2_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan2_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan2_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan2_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit1));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit2));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit1));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit2));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit1));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit2));
}

void updateCounters() {
    int cur1 = 0, cur2 = 0;
    pcnt_unit_get_count(pcnt_unit1, &cur1);
    pcnt_unit_get_count(pcnt_unit2, &cur2);

    int diff1 = cur1 - g_last_raw1;
    int diff2 = cur2 - g_last_raw2;
    g_last_raw1 = cur1;
    g_last_raw2 = cur2;

    g_accum1 += diff1;
    g_accum2 += diff2;
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=============================================");
    Serial.println("   DIAGNOSTICO DE ENCODERS COM HARDWARE PCNT");
    Serial.println("=============================================");
    Serial.println("Modo: Decodificacao de Quadratura 4x via Hardware PCNT");
    Serial.println("Gire as rodas manualmente para frente.");
    Serial.println("Comandos via Serial:");
    Serial.println("  'z' ou 'r' -> Zera os contadores.");
    Serial.println("=============================================\n");

    setupPCNT();

    g_accum1 = 0;
    g_accum2 = 0;
    g_last_raw1 = 0;
    g_last_raw2 = 0;
}

unsigned long lastPrint = 0;

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'z' || c == 'r' || c == 'Z' || c == 'R') {
            pcnt_unit_clear_count(pcnt_unit1);
            pcnt_unit_clear_count(pcnt_unit2);
            g_accum1 = 0;
            g_accum2 = 0;
            g_last_raw1 = 0;
            g_last_raw2 = 0;
            Serial.println("\n>>> CONTADORES ZERADOS! <<<\n");
        }
    }

    updateCounters();

    if (millis() - lastPrint >= 150) {
        lastPrint = millis();
        float voltas1 = (float)g_accum1 / PULSOS_POR_VOLTA;
        float voltas2 = (float)g_accum2 / PULSOS_POR_VOLTA;

        Serial.printf("[PCNT] ESQ (Enc1): %6lld pulsos (%+5.2f v)  |  DIR (Enc2): %6lld pulsos (%+5.2f v)\n",
                      g_accum1, voltas1, g_accum2, voltas2);
    }
}
