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
 *   2. Grave este sketch no ESP32. Ele conecta no Wi-Fi e arma o ESC.
 *   3. Leia os resultados remotamente, de duas formas:
 *      a) WebSocket (porta 81): cada ponto é enviado como JSON; o CSV
 *         completo pode ser pedido com o comando "csv".
 *      b) Serial (115200): 'r' executa a varredura, 'c' imprime o CSV.
 *      c) Buffer em RAM: os pontos ficam retidos e podem ser lidos depois,
 *         mesmo que o cabo seja desconectado no meio da varredura.
 *
 * Conexoes (mesmas do firmware V4):
 *   ESC CH1 (Motor 1 - Esquerdo): GPIO 19
 *   ESC CH2 (Motor 2 - Direito):  GPIO 18
 *   Encoder 1 (Esq) A/B: GPIO 16 / 17
 *   Encoder 2 (Dir) A/B: GPIO 33 / 32
 *
 * Taxa da Serial: 115200 baud
 */

#include <Arduino.h>
#include "driver/pulse_cnt.h"
#include "driver/mcpwm_prelude.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

#include "secrets.h"   // WIFI_SSID, WIFI_PASSWORD

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

#ifndef WIFI_MDNS_NAME
#define WIFI_MDNS_NAME "robo"
#endif
#ifndef WIFI_HTTP_PORT
#define WIFI_HTTP_PORT 80
#endif
#ifndef WIFI_WS_PORT
#define WIFI_WS_PORT   81
#endif
#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 10000
#endif

// ==================== BUFFER DE RESULTADOS (RAM) ====================
#define MAX_PONTOS 16

struct PontoCurva {
    uint32_t pwm;
    float    rpmEsq;
    float    rpmDir;
};

static PontoCurva _pontos[MAX_PONTOS];
static uint8_t    _nPontos = 0;
static bool       _varreduraAndando = false;

static void guardarPonto(uint32_t pwm, float rpmEsq, float rpmDir) {
    if (_nPontos >= MAX_PONTOS) return;
    _pontos[_nPontos].pwm     = pwm;
    _pontos[_nPontos].rpmEsq  = rpmEsq;
    _pontos[_nPontos].rpmDir  = rpmDir;
    _nPontos++;
}

static void limparPontos() {
    _nPontos = 0;
}

static void imprimirCsv(HardwareSerial& porta) {
    porta.println("PWM_us,RPM_esq,RPM_dir");
    for (uint8_t i = 0; i < _nPontos; i++) {
        porta.print(_pontos[i].pwm);
        porta.print(',');
        porta.print(_pontos[i].rpmEsq, 1);
        porta.print(',');
        porta.println(_pontos[i].rpmDir, 1);
    }
}

static String csvComoString() {
    String s = "PWM_us,RPM_esq,RPM_dir\n";
    for (uint8_t i = 0; i < _nPontos; i++) {
        s += String(_pontos[i].pwm) + ",";
        s += String(_pontos[i].rpmEsq, 1) + ",";
        s += String(_pontos[i].rpmDir, 1) + "\n";
    }
    return s;
}

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

// ==================== WI-FI / WEBSOCKET ====================
static WebServer        _http(WIFI_HTTP_PORT);
static WebSocketsServer _ws(WIFI_WS_PORT);
static bool _wifiOk = false;

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1.0"/>
<title>Diag Motor — Curva PWM x RPM</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:#0d1117;color:#c9d1d9;font-family:'Segoe UI',sans-serif;padding:16px}
  h1{font-size:1.2rem;color:#58a6ff;margin-bottom:12px}
  .bar{display:flex;align-items:center;gap:14px;flex-wrap:wrap;margin-bottom:14px;
       background:#161b22;border-radius:8px;padding:10px 14px;border:1px solid #30363d}
  .btn{padding:7px 18px;border-radius:6px;border:none;cursor:pointer;font-size:.85rem;font-weight:600}
  #btnSweep{background:#238636;color:#fff}
  #btnCsv{background:#1f6feb;color:#fff}
  #btnClear{background:#21262d;color:#8b949e;border:1px solid #30363d}
  #dot{width:12px;height:12px;border-radius:50%;background:#f85149}
  #dot.on{background:#3fb950}
  table{border-collapse:collapse;width:100%;margin-top:14px;font-size:.9rem}
  th,td{border:1px solid #30363d;padding:6px 10px;text-align:right}
  th{background:#161b22;color:#8b949e}
  td:first-child,th:first-child{text-align:left}
</style>
</head>
<body>
<h1>Diag Motor — Curva PWM x RPM</h1>
<div class="bar">
  <div id="dot"></div>
  <span id="connTxt" style="font-size:.85rem;color:#8b949e">Desconectado</span>
  <button class="btn" id="btnSweep" onclick="sendCmd('sweep')">▶ Varredura</button>
  <button class="btn" id="btnCsv"   onclick="sendCmd('csv')">⬇ CSV</button>
  <button class="btn" id="btnClear" onclick="sendCmd('clear')">↺ Limpar</button>
</div>
<table>
  <thead><tr><th>PWM (µs)</th><th>RPM Esq</th><th>RPM Dir</th></tr></thead>
  <tbody id="tbody"></tbody>
</table>
<script>
let ws;
function connect(){
  ws = new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen  = ()=>{ document.getElementById('dot').className='on';
                     document.getElementById('connTxt').textContent='Conectado'; };
  ws.onclose = ()=>{ document.getElementById('dot').className='';
                     document.getElementById('connTxt').textContent='Desconectado';
                     setTimeout(connect,3000); };
  ws.onerror = ()=>{ ws.close(); };
  ws.onmessage = e=>{ try{ handle(JSON.parse(e.data)); }catch(_){} };
}
function sendCmd(cmd){ if(ws && ws.readyState===1) ws.send(cmd); }
function handle(d){
  if(d.tipo === 'ponto'){
    const tb = document.getElementById('tbody');
    const tr = document.createElement('tr');
    tr.innerHTML = '<td>'+d.pwm+'</td><td>'+d.re.toFixed(1)+'</td><td>'+d.rd.toFixed(1)+'</td>';
    tb.appendChild(tr);
  } else if(d.tipo === 'fim'){
    const tb = document.getElementById('tbody');
    const tr = document.createElement('tr');
    tr.innerHTML = '<td colspan="3" style="text-align:center;color:#3fb950">Varredura concluída</td>';
    tb.appendChild(tr);
  } else if(d.tipo === 'status'){
    document.getElementById('connTxt').textContent = d.msg;
  }
}
connect();
</script>
</body>
</html>
)rawliteral";

static void enviarPontoWs(uint32_t pwm, float rpmEsq, float rpmDir) {
    JsonDocument doc;
    doc["tipo"] = "ponto";
    doc["pwm"]  = pwm;
    doc["re"]   = roundf(rpmEsq * 10.0f) / 10.0f;
    doc["rd"]   = roundf(rpmDir * 10.0f) / 10.0f;
    String json;
    serializeJson(doc, json);
    _ws.broadcastTXT(json);
}

static void enviarCsvWs() {
    JsonDocument doc;
    doc["tipo"]  = "csv";
    doc["dados"] = csvComoString();
    String json;
    serializeJson(doc, json);
    _ws.broadcastTXT(json);
}

static void enviarStatusWs(const char* msg) {
    JsonDocument doc;
    doc["tipo"] = "status";
    doc["msg"]  = msg;
    String json;
    serializeJson(doc, json);
    _ws.broadcastTXT(json);
}

static void onWsEvent(uint8_t, WStype_t type, uint8_t* payload, size_t length) {
    if (type != WStype_TEXT) return;
    String msg;
    msg.reserve(length);
    for (size_t i = 0; i < length; i++) msg += (char)payload[i];
    msg.trim();

    if (msg == "sweep" || msg == "r") {
        if (!_varreduraAndando) runSweepAsyncStart();
    } else if (msg == "csv" || msg == "c") {
        enviarCsvWs();
    } else if (msg == "clear" || msg == "z") {
        limparPontos();
        enviarStatusWs("Buffer limpo.");
    }
}

static void setupWifi() {
    Serial.print("[WiFi] Conectando a '");
    Serial.print(WIFI_SSID);
    Serial.print("' ");

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - t0 > WIFI_CONNECT_TIMEOUT_MS) {
            Serial.println("\n[WiFi] Timeout! Seguindo so com Serial.");
            _wifiOk = false;
            return;
        }
        delay(250);
        Serial.print('.');
    }

    Serial.print("\n[WiFi] Conectado! IP: ");
    Serial.println(WiFi.localIP());

    if (MDNS.begin(WIFI_MDNS_NAME)) {
        Serial.print("[mDNS] Hostname: ");
        Serial.print(WIFI_MDNS_NAME);
        Serial.println(".local");
    }

    _http.on("/", []() {
        _http.send_P(200, "text/html", INDEX_HTML);
    });
    _http.begin();

    _ws.begin();
    _ws.onEvent(onWsEvent);
    Serial.print("[WS] WebSocket na porta ");
    Serial.println(WIFI_WS_PORT);

    _wifiOk = true;
}

// ==================== VARREDURA PWM x RPM (não-bloqueante) ====================
// A varredura é feita em passos no loop() para não bloquear o WebSocket.
enum SweepState {
    SWEEP_IDLE,
    SWEEP_SETTLE,
    SWEEP_MEASURE,
    SWEEP_REST
};

static const uint32_t _pwms[] = {1500, 1550, 1600, 1650, 1700, 1750, 1800, 1850};
static const int _nPwms = sizeof(_pwms) / sizeof(_pwms[0]);

static SweepState _swState = SWEEP_IDLE;
static int       _swIdx = 0;
static uint32_t  _swStartTime = 0;
static int32_t   _swTicks0a = 0, _swTicks0b = 0;

void runSweepAsyncStart() {
    limparPontos();
    _swIdx = 0;
    _swState = SWEEP_SETTLE;
    _varreduraAndando = true;
    Serial.println("\n=============================================");
    Serial.println("   CURVA PWM x RPM  (bancada, robo suspenso)");
    Serial.println("=============================================");
    Serial.println("PWM_us,RPM_esq,RPM_dir");
    if (_wifiOk) enviarStatusWs("Varredura iniciada...");
}

static void sweepStep() {
    switch (_swState) {
        case SWEEP_IDLE:
            break;

        case SWEEP_SETTLE:
            resetCounters();
            setPWM(_pwms[_swIdx]);
            _swStartTime = millis();
            _swState = SWEEP_MEASURE;
            break;

        case SWEEP_MEASURE:
            if (millis() - _swStartTime < SETTLE_MS) return;
            getTicks(_swTicks0a, _swTicks0b);
            _swStartTime = millis();
            _swState = SWEEP_REST;
            break;

        case SWEEP_REST: {
            if (millis() - _swStartTime < MEASURE_MS) return;
            int32_t t1a = 0, t1b = 0;
            getTicks(t1a, t1b);
            float dt = MEASURE_MS / 1000.0f;
            float rpm1 = ((float)(t1a - _swTicks0a) / PPR_EFETIVO) * (60.0f / dt);
            float rpm2 = ((float)(t1b - _swTicks0b) / PPR_EFETIVO) * (60.0f / dt);

            Serial.printf("%lu,%.1f,%.1f\n", _pwms[_swIdx], rpm1, rpm2);
            guardarPonto(_pwms[_swIdx], rpm1, rpm2);
            if (_wifiOk) enviarPontoWs(_pwms[_swIdx], rpm1, rpm2);

            stopMotors();
            _swIdx++;
            if (_swIdx >= _nPwms) {
                _swState = SWEEP_IDLE;
                _varreduraAndando = false;
                Serial.println("=============================================");
                Serial.println("Varredura concluida. Envie 'r' para repetir.");
                Serial.println("=============================================\n");
                if (_wifiOk) enviarStatusWs("Varredura concluida.");
            } else {
                _swState = SWEEP_SETTLE;
            }
            break;
        }
    }
}

// ==================== SETUP / LOOP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=============================================");
    Serial.println("   DIAGNOSTICO DE CURVA PWM x RPM");
    Serial.println("=============================================");
    Serial.println("O robo deve estar SUSPENSO (rodas livres).");
    Serial.println("Comandos Serial: 'r' varre | 'c' CSV | 'z' limpa");
    Serial.println("Remoto: WebSocket porta 81 (sweep / csv / clear)");
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

    setupWifi();

    Serial.printf("Armando o ESC por %d ms em neutro...\n", ARM_MS);
    setPWM(ESC_NEUTRO_US);
    delay(ARM_MS);
    Serial.println("[OK] ESC armado.\n");
}

void loop() {
    if (_wifiOk) {
        _ws.loop();
        _http.handleClient();
    }

    sweepStep();

    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'r' || c == 'R') {
            if (!_varreduraAndando) runSweepAsyncStart();
        } else if (c == 'c' || c == 'C') {
            Serial.println("--- CSV (buffer RAM) ---");
            imprimirCsv(Serial);
            Serial.println("------------------------");
        } else if (c == 'z' || c == 'Z') {
            limparPontos();
            Serial.println(">>> Buffer limpo.");
        }
    }
}
