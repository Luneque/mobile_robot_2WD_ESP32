#include <Arduino.h>

// Pinos dos Encoders conforme especificação
#define PIN_ENC1_A 23
#define PIN_ENC1_B 22
#define PIN_ENC2_A 33
#define PIN_ENC2_B 32

volatile uint32_t count1_a = 0;
volatile uint32_t count1_b = 0;
volatile uint32_t count2_a = 0;
volatile uint32_t count2_b = 0;

void IRAM_ATTR isr1_a() { count1_a++; }
void IRAM_ATTR isr1_b() { count1_b++; }
void IRAM_ATTR isr2_a() { count2_a++; }
void IRAM_ATTR isr2_b() { count2_b++; }

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n--- TESTE DIAGNOSTICO DE LEITURA DOS ENCODERS ---");
    Serial.println("Gire as rodas manualmente para verificar se os pulsos sao detectados!");

    pinMode(PIN_ENC1_A, INPUT_PULLUP);
    pinMode(PIN_ENC1_B, INPUT_PULLUP);
    pinMode(PIN_ENC2_A, INPUT_PULLUP);
    pinMode(PIN_ENC2_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC1_A), isr1_a, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC1_B), isr1_b, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC2_A), isr2_a, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC2_B), isr2_b, CHANGE);
}

void loop() {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500) {
        lastPrint = millis();

        int state1_a = digitalRead(PIN_ENC1_A);
        int state1_b = digitalRead(PIN_ENC1_B);
        int state2_a = digitalRead(PIN_ENC2_A);
        int state2_b = digitalRead(PIN_ENC2_B);

        Serial.printf("ENC1 (GPIO 23,22): PulsoA=%u, PulsoB=%u | NivelA=%d, NivelB=%d  ||  ENC2 (GPIO 33,32): PulsoA=%u, PulsoB=%u | NivelA=%d, NivelB=%d\n",
                      count1_a, count1_b, state1_a, state1_b,
                      count2_a, count2_b, state2_a, state2_b);
    }
}
