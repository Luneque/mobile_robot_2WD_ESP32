// =========================================================================
// GATEWAY ESP-NOW — ESP32-S3 Super Mini
// Ponte bidirecional: ESP-NOW (robô) <-> Serial USB (PC)
// =========================================================================
// INSTRUÇÕES DE USO:
// 1. Grave este firmware no ESP32-S3 Super Mini via Arduino IDE.
//    Placa: "ESP32S3 Dev Module" (ou "LOLIN S3 Mini" se disponível).
//    Selecione a porta USB correta.
// 2. Após a gravação, abra o Monitor Serial (115200 baud) e anote o MAC
//    impresso na inicialização. Ex: "MAC: AA:BB:CC:DD:EE:FF"
// 3. Cole o MAC no firmware V4 do robô em config.h:
//    #define ESPNOW_GATEWAY_MAC {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}
// 4. Conecte o ESP32-S3 ao PC via USB. Rode o Python logger apontando
//    para a porta COM deste gateway.
// =========================================================================
//
// PROTOCOLO SERIAL (PC <-> Gateway):
//   PC -> Gateway: "CMD:<payload>\n"   (ex: "CMD:start\n")
//   Gateway -> PC: "TELEM:{json}\n"   (telemetria recebida do robô)
//   Gateway -> PC: "MAC: XX:XX:...\n" (endereço MAC no boot)
//   Gateway -> PC: "DBG:<msg>\n"      (mensagens de debug)
//
// CANAL ESP-NOW: 1 (deve ser igual no robô)
// =========================================================================

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// -------------------------------------------------------------------------
// Configurações
// -------------------------------------------------------------------------
#define SERIAL_BAUD       115200
#define ESPNOW_CHANNEL    1          // Canal Wi-Fi para ESP-NOW (1–13)
#define MAX_PAYLOAD       250        // Limite ESP-NOW em bytes

// MAC broadcast — envia para todos os dispositivos ESP-NOW na rede
static uint8_t broadcastMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Buffer para leitura serial
static String serialBuffer = "";

// -------------------------------------------------------------------------
// Callback: dados recebidos via ESP-NOW (do robô -> gateway)
// Compatível com ESP-IDF v4 (Arduino ESP32 core 2.x) e v5 (core 3.x)
// -------------------------------------------------------------------------
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
void OnDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
#else
void OnDataRecv(const uint8_t* mac_addr, const uint8_t* data, int len) {
#endif
    // Converte o payload de bytes para string (JSON)
    // O robô envia texto JSON terminado em '\0' ou preenchido com zeros
    char buf[MAX_PAYLOAD + 1];
    int copyLen = (len < MAX_PAYLOAD) ? len : MAX_PAYLOAD;
    memcpy(buf, data, copyLen);
    buf[copyLen] = '\0';

    // Remove zeros e espaços à direita (padding do struct)
    int end = copyLen - 1;
    while (end >= 0 && (buf[end] == '\0' || buf[end] == ' ')) {
        end--;
    }
    buf[end + 1] = '\0';

    // Encaminha ao PC via Serial no formato "TELEM:{json}"
    Serial.printf("TELEM:%s\n", buf);
}

// -------------------------------------------------------------------------
// Callback: confirmação de envio ESP-NOW (gateway -> robô)
// -------------------------------------------------------------------------
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("DBG:CMD enviado com sucesso");
    } else {
        Serial.println("DBG:ERRO ao enviar CMD via ESP-NOW");
    }
}

// -------------------------------------------------------------------------
// setup()
// -------------------------------------------------------------------------
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);

    Serial.println("DBG:Iniciando Gateway ESP-NOW...");

    // Inicializa Wi-Fi em modo estação (STA), sem conectar a nenhuma rede
    // ESP-NOW opera no canal Wi-Fi mesmo sem conexão
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();  // Garante que não está tentando conectar a nada

    // Força o canal Wi-Fi para o canal 1 (deve coincidir com o robô)
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Imprime o próprio MAC (usuário deve anotar e configurar no robô)
    Serial.printf("MAC: %s\n", WiFi.macAddress().c_str());
    Serial.printf("DBG:Canal ESP-NOW: %d\n", ESPNOW_CHANNEL);

    // Inicializa ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("DBG:ERRO ao inicializar ESP-NOW! Reiniciando...");
        delay(2000);
        ESP.restart();
    }

    // Registra callbacks
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);

    // Registra peer broadcast (FF:FF:FF:FF:FF:FF)
    // Isso permite enviar comandos para qualquer dispositivo ESP-NOW no canal
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastMAC, 6);
    peerInfo.channel  = ESPNOW_CHANNEL;
    peerInfo.encrypt  = false;
    peerInfo.ifidx    = WIFI_IF_STA;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("DBG:ERRO ao registrar peer broadcast");
    } else {
        Serial.println("DBG:Peer broadcast registrado");
    }

    Serial.println("DBG:Gateway pronto. Aguardando dados...");
}

// -------------------------------------------------------------------------
// loop()
// -------------------------------------------------------------------------
void loop() {
    // Lê linha da Serial (do PC para o gateway)
    // Formato esperado: "CMD:<payload>\n"
    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == '\n') {
            // Linha completa recebida
            serialBuffer.trim();

            if (serialBuffer.startsWith("CMD:")) {
                // Extrai o payload (tudo após "CMD:")
                String payload = serialBuffer.substring(4);
                payload.trim();

                if (payload.length() > 0 && payload.length() <= MAX_PAYLOAD) {
                    // Envia via ESP-NOW broadcast ao robô
                    esp_err_t result = esp_now_send(
                        broadcastMAC,
                        (const uint8_t*)payload.c_str(),
                        payload.length() + 1  // +1 para incluir '\0'
                    );

                    if (result != ESP_OK) {
                        Serial.printf("DBG:Erro esp_now_send: %d\n", result);
                    }
                } else if (payload.length() > MAX_PAYLOAD) {
                    Serial.println("DBG:ERRO: payload muito longo (>250 bytes)");
                }
            }
            // Ignora linhas que não começam com "CMD:"

            serialBuffer = "";  // Limpa buffer para próxima linha
        } else if (c == '\r') {
            // Ignora carriage return
        } else {
            // Acumula caractere no buffer
            if (serialBuffer.length() < MAX_PAYLOAD + 10) {
                serialBuffer += c;
            }
        }
    }

    // Pequeno yield para não bloquear o watchdog
    yield();
}
