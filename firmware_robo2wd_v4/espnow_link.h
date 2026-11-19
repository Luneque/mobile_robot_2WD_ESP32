/**
 * espnow_link.h — Enlace ESP-NOW do robô (V4)
 *
 * Ponte com o gateway ESP32 (gateway_espnow/): o robô envia telemetria em
 * JSON e recebe comandos de texto (start/stop/pause/reset/dump), funcionando
 * como redundância ou alternativa ao Wi-Fi/WebSocket.
 *
 * Compatível com ESP-IDF v4 (Arduino core 2.x) e v5 (core 3.x), onde a
 * assinatura do callback de recepção mudou.
 *
 * Uso:
 *   EspNowLink link;
 *   link.begin();                       // no setup()
 *   link.sendTelemetry(json);           // no loop, na taxa configurada
 *   if (link.hasCommand()) cmd = link.getLastCommand();
 */

#ifndef ESPNOW_LINK_H
#define ESPNOW_LINK_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "config.h"

#define ESPNOW_MAX_PAYLOAD   250

class EspNowLink {
public:
    bool begin() {
        _instance = this;   // callbacks estáticos precisam achar a instância

        // ESP-NOW exige o Wi-Fi inicializado. O robô já opera em WIFI_STA para
        // o dashboard; aqui só garantimos o modo.
        if (WiFi.getMode() == WIFI_OFF) {
            WiFi.mode(WIFI_STA);
        }

        if (esp_now_init() != ESP_OK) {
            Serial.println(F("[ESPNOW] Falha ao inicializar ESP-NOW."));
            _initOk = false;
            return false;
        }

        esp_now_register_recv_cb(onRecv);
        esp_now_register_send_cb(onSent);

        _initOk = true;
        return true;
    }

    // NÃO se pode chamar esp_wifi_set_channel() depois da conexão Wi-Fi: isso
    // derruba o enlace com o AP e o sistema para (o dashboard fica
    // irresponsivo e o setup não termina). O canal do enlace é o canal atual
    // da interface, e o peer é registrado com canal 0.
    void fixChannel() {
        if (!_initOk) return;
        uint8_t ch = 0;
        wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
        if (esp_wifi_get_channel(&ch, &sec) == ESP_OK) {
            Serial.print(F("[ESPNOW] Canal atual da interface: "));
            Serial.println(ch);
        }
        Serial.println(F("[ESPNOW] Peer usara o canal atual (sem trocar de canal)."));
    }

    // Registra o peer depois que o canal já está fixado. Registra com canal 0
    // ("usa o canal atual da interface"), que é o que funciona quando o robô está
    // conectado a um AP: o canal real é o do AP e não pode ser alterado sem
    // derrubar a conexão Wi-Fi.
    void addPeerAfterChannel() {
        if (!_initOk) return;

        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, _gatewayMac, 6);
        peer.channel = 0;   // 0 = canal atual da interface
        peer.encrypt = false;
        peer.ifidx   = WIFI_IF_STA;

        esp_err_t r = esp_now_add_peer(&peer);
        if (r == ESP_ERR_ESPNOW_EXIST) {
            Serial.println(F("[ESPNOW] Peer ja registrado."));
            return;
        }
        Serial.print(F("[ESPNOW] Peer (pos-canal): "));
        Serial.println((r == ESP_OK) ? F("registrado") : F("falha"));
    }

    // Envia telemetria (JSON) ao gateway. Retorna true se aceito pelo driver.
    bool sendTelemetry(const char* json) {
        if (!_initOk || !json) return false;
        size_t n = strlen(json);
        if (n > ESPNOW_MAX_PAYLOAD) n = ESPNOW_MAX_PAYLOAD;
        esp_err_t r = esp_now_send(_gatewayMac, (const uint8_t*)json, (uint8_t)(n + 1));
        if (r == ESP_OK) _lastSendOk = true;
        return r == ESP_OK;
    }

    // Comando recebido do gateway (start/stop/pause/reset/dump).
    bool hasCommand() const { return _newCommand; }
    const char* getLastCommand() {
        _newCommand = false;
        return _lastCmd;
    }

    bool isInitialized() const { return _initOk; }
    bool lastSendOk()     const { return _lastSendOk; }

private:
    uint8_t  _gatewayMac[6] = ESPNOW_GATEWAY_MAC;
    bool     _initOk = false;
    volatile bool _newCommand = false;
    char     _lastCmd[64] = {0};
    bool     _lastSendOk = false;

    // Callback de recepção: o gateway envia payload de texto.
    #if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
    static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    #else
    static void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
    #endif
        EspNowLink* self = _instance;
        if (!self || !data) return;

        int n = (len < (int)sizeof(self->_lastCmd) - 1) ? len : (int)sizeof(self->_lastCmd) - 1;
        if (n <= 0) return;

        // O gateway pode preencher o payload com zeros à direita: copia só até o
        // primeiro terminador.
        int copy = 0;
        while (copy < n && data[copy] != '\0') copy++;
        memcpy(self->_lastCmd, data, copy);
        self->_lastCmd[copy] = '\0';

        // Trim de espaços e quebras de linha à direita
        int end = (int)strlen(self->_lastCmd) - 1;
        while (end >= 0 && (self->_lastCmd[end] == ' ' || self->_lastCmd[end] == '\r' ||
                            self->_lastCmd[end] == '\n')) {
            self->_lastCmd[end] = '\0';
            end--;
        }

        self->_newCommand = true;
        Serial.print(F("[ESPNOW] Comando recebido: "));
        Serial.println(self->_lastCmd);
    }

    // Callback de confirmação de envio. A assinatura mudou no ESP-IDF v5
    // (core 3.x): de (mac, status) para (tx_info, status).
    #if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
    static void onSent(const wifi_tx_info_t* tx, esp_now_send_status_t status) {
    #else
    static void onSent(const uint8_t* mac, esp_now_send_status_t status) {
    #endif
        EspNowLink* self = _instance;
        if (!self) return;
        self->_lastSendOk = (status == ESP_NOW_SEND_SUCCESS);
        if (status != ESP_NOW_SEND_SUCCESS) {
            Serial.println(F("[ESPNOW] Falha no envio da telemetria."));
        }
    }

    static EspNowLink* _instance;
};

// Instância única usada pelos callbacks estáticos do ESP-NOW.
inline EspNowLink* EspNowLink::_instance = nullptr;

#endif // ESPNOW_LINK_H
