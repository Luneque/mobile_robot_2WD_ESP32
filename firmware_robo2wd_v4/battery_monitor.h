/**
 * battery_monitor.h — Leitura e protecao de bateria LiPo 2S (V4)
 *
 * O divisor de tensao (R1=100k/R2=47k) entrega no GPIO 34 (ADC1_CH6) uma
 * fracao da tensao da bateria. Como a curva do ADC do ESP32 com atenuacao
 * de 11 dB nao e' linear, o valor em volts e' obtido por CALIBRACAO com
 * multimetro: mede-se a tensao real da bateria e o valor cru lido no ADC,
 * e os dois pontos resultantes viram a reta de calibragem.
 *
 * Sem calibracao o firmware usa a conversao nominal (fundo de escala 2,5 V),
 * que serve para o comportamento geral mas nao para um limiar confiavel.
 *
 * Uso:
 *   BatteryMonitor bat;
 *   bat.begin();
 *   float v = bat.readVolts();      // tensao da bateria
 *   bool critica = bat.isCritical();
 */

#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

class BatteryMonitor {
public:
    void begin() {
#if VBAT_HABILITADO
        _habilitado = (PIN_BATTERY_ADC >= 0);
        if (!_habilitado) return;
        analogSetPinAttenuation(PIN_BATTERY_ADC, (adc_attenuation_t)VBAT_ADC_ATTEN);
        analogSetWidth(12);
        pinMode(PIN_BATTERY_ADC, INPUT);
        // O ADC precisa de algumas dezenas de ms para estabilizar depois da
        // configuracao da atenuacao. Sem isso, o primeiro pacote de telemetria
        // do boot trazia um valor espúrio (ja' visto: 796 V).
        for (int i = 0; i < 8; i++) { analogRead(PIN_BATTERY_ADC); delay(5); }
        _ultimaLeitura = millis();
        // Forca uma leitura valida ja' no boot, para que a telemetria nao
        // publique o valor residual (-1) nem um lixo de conversao.
        _volts = adcParaVolts(lerAdc());
#else
        _habilitado = false;
#endif
    }

    bool isEnabled() const { return _habilitado; }

    // Tensao da bateria em volts. Retorna -1.0f se desabilitado/sem leitura.
    float readVolts() {
#if VBAT_HABILITADO
        if (!_habilitado) return -1.0f;
        if (millis() - _ultimaLeitura < VBAT_LEITURA_MS) return _volts;
        _ultimaLeitura = millis();
        _volts = adcParaVolts(lerAdc());
        return _volts;
#else
        return -1.0f;
#endif
    }

    // Conversao ADC -> volts da bateria. Usa a reta de calibracao quando
    // disponivel; caso contrario, a conversao nominal do fundo de escala.
    float adcParaVolts(float adc) const {
#if VBAT_HABILITADO
        if (_calibrado) {
            return _adc0 + (adc - _adcRef0) * (_adc1 - _adc0) / (_adcRef1 - _adcRef0);
        }
        // Reta de um so ponto com o ganho real do divisor (V por unidade de
        // ADC), medido com multimetro. Mais fiel que a curva nominal, que
        // satura bem antes do fundo de escala e superestima.
        return VBAT_ADC_REF_ZERO + adc * VBAT_ADC_REF_VOLTS;
#else
        (void)adc;
        return -1.0f;
#endif
    }

    // Calibracao de dois pontos (multimetro):
    //   adcRef0 = leitura do ADC com a bateria em voltsRef0
    //   adcRef1 = leitura do ADC com a bateria em voltsRef1
    void setCalibracao(float adcRef0, float voltsRef0,
                       float adcRef1, float voltsRef1) {
        _adcRef0 = adcRef0; _adc0 = voltsRef0;
        _adcRef1 = adcRef1; _adc1 = voltsRef1;
        _calibrado = true;
    }

    bool isCalibrated() const { return _calibrado; }

    float lastAdc() const { return _adcBruto; }

    // ── Persistencia da calibragem na NVS (flash) ──────────────────────────
    // Guarda o ponto medido (adc -> volts) para sobreviver ao reboot. Sem isso
    // era preciso reenviar "vbat <tensao>" a cada ligacao.
    void salvarCalibracao() {
#if VBAT_HABILITADO
        if (!_calibrado) return;
        Preferences prefs;
        if (!prefs.begin("bateria", false)) return;
        prefs.putFloat("adcRef", _adcRef0);
        prefs.putFloat("vRef",   _adc0);
        prefs.putFloat("ganho",  VBAT_ADC_REF_VOLTS);
        prefs.end();
#endif
    }

    void carregarCalibracao() {
#if VBAT_HABILITADO
        Preferences prefs;
        if (!prefs.begin("bateria", true)) return;   // read-only
        if (prefs.isKey("adcRef") && prefs.isKey("vRef")) {
            float adcRef = prefs.getFloat("adcRef", 0.0f);
            float vRef   = prefs.getFloat("vRef",   0.0f);
            prefs.end();
            // Reta de dois pontos: o ponto medido + o vazio (adc 0 -> 0 V).
            setCalibracao(adcRef, vRef, 0.0f, 0.0f);
            return;
        }
        prefs.end();
#endif
    }

    // Estado de carga segundo os limiares homologados.
    // 0 = ok, 1 = atencao, 2 = critico (corte).
    uint8_t nivel() const {
        if (!_habilitado || _volts < 0.0f) return 0;
        if (_volts <= VBAT_LIMIAR_CORTE)  return 2;
        if (_volts <= VBAT_LIMIAR_ATENCAO) return 1;
        return 0;
    }

    bool isCritical() const { return nivel() == 2; }
    bool isWarning()  const { return nivel() == 1; }

private:
    // Media das amostras do ADC (valor cru, 0-4095).
    float lerAdc() {
        long soma = 0;
        for (int i = 0; i < VBAT_AMOSTRAS; i++) soma += analogRead(PIN_BATTERY_ADC);
        float adc = (float)soma / (float)VBAT_AMOSTRAS;
        _adcBruto = adc;
        return adc;
    }

    bool     _habilitado = false;
    bool     _calibrado  = false;
    float    _volts      = -1.0f;
    float    _adcBruto   = 0.0f;
    uint32_t _ultimaLeitura = 0;
    // Reta de calibracao: volts = _adc0 + (adc - _adcRef0) * (_adc1-_adc0)/(_adcRef1-_adcRef0)
    float    _adcRef0 = 0.0f, _adc0 = 0.0f;
    float    _adcRef1 = 0.0f, _adc1 = 0.0f;
};

#endif // BATTERY_MONITOR_H
