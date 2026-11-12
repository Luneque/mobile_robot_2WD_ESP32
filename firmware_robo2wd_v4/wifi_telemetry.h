/**
 * wifi_telemetry.h — Driver de telemetria Wi-Fi para firmware V4 do robô 2WD
 *
 * Dependências (instalar via Arduino Library Manager):
 *   - "WebSockets" by Markus Sattler (arduinoWebSockets) — WebSocketsServer.h
 *   - "ArduinoJson" by Benoit Blanchon v7              — ArduinoJson.h
 *
 * Built-in do ESP32 Arduino core (não precisam instalar):
 *   - WiFi.h, WebServer.h, ESPmDNS.h
 *
 * Uso típico:
 *   WifiTelemetry telemetry;
 *   void setup() { telemetry.begin(); }
 *   void loop()  { telemetry.handle(); telemetry.sendTelemetry(pkt); }
 */

#ifndef WIFI_TELEMETRY_H
#define WIFI_TELEMETRY_H

// ─────────────────────────────────────────────────────────────────────────────
// Includes
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

#include "secrets.h"   // WIFI_SSID, WIFI_PASSWORD

// ─────────────────────────────────────────────────────────────────────────────
// Configurações com valores padrão (podem ser sobreescritas via config.h)
// ─────────────────────────────────────────────────────────────────────────────
#ifndef WIFI_MDNS_NAME
#define WIFI_MDNS_NAME          "robo"   // acessível como robo.local
#endif
#ifndef WIFI_HTTP_PORT
#define WIFI_HTTP_PORT          80
#endif
#ifndef WIFI_WS_PORT
#define WIFI_WS_PORT            81
#endif
#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 10000   // 10 s de timeout na conexão
#endif
#ifndef TELEM_INTERVAL_MS
#define TELEM_INTERVAL_MS       200     // 5 Hz de telemetria
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Struct de Telemetria
// ─────────────────────────────────────────────────────────────────────────────
struct TelemetryPacket {
    uint32_t    ts;          // millis() desde o boot
    const char* estado;      // "AGUARDO","RETA","PIVO","PAUSA","CONCLUIDO","EMERGENCIA","FALHA"
    int         lado;        // segmento atual (1–4)
    int         totalLados;  // sempre 4
    float       yaw;         // graus (com sinal)
    float       rpm1;        // RPM motor esquerdo
    float       rpm2;        // RPM motor direito
    int32_t     ticks1;      // ticks encoder esquerdo (valor absoluto)
    int32_t     ticks2;      // ticks encoder direito  (valor absoluto)
    int32_t     ticksAlvo;   // ticks alvo do segmento
    uint32_t    pwm1;        // PWM motor 1 em microsegundos
    uint32_t    pwm2;        // PWM motor 2 em microsegundos
    float       rpmAlvo;     // RPM alvo do perfil trapezoidal
};

// ─────────────────────────────────────────────────────────────────────────────
// Dashboard HTML embutido em PROGMEM
// ─────────────────────────────────────────────────────────────────────────────
static const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1.0"/>
<title>Robô 2WD — Telemetria V4</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:#0d1117;color:#c9d1d9;font-family:'Segoe UI',sans-serif;padding:16px}
  h1{font-size:1.3rem;color:#58a6ff;margin-bottom:12px;letter-spacing:.05em}
  /* ---- Status bar ---- */
  #statusBar{display:flex;align-items:center;gap:16px;flex-wrap:wrap;margin-bottom:14px;
             background:#161b22;border-radius:8px;padding:10px 14px;border:1px solid #30363d}
  #dot{width:12px;height:12px;border-radius:50%;background:#f85149;flex-shrink:0;
       box-shadow:0 0 6px #f85149}
  #dot.on{background:#3fb950;box-shadow:0 0 8px #3fb950}
  #connTxt{font-size:.85rem;color:#8b949e}
  /* ---- Badge de estado ---- */
  #badge{padding:4px 12px;border-radius:20px;font-size:.9rem;font-weight:600;
         background:#21262d;color:#8b949e;border:1px solid #30363d;transition:background .3s}
  #badge.RETA      {background:#1f4e9e;color:#a5d3ff;border-color:#388bfd}
  #badge.PIVO      {background:#5c3a00;color:#ffa657;border-color:#d29922}
  #badge.AGUARDO   {background:#21262d;color:#8b949e;border-color:#30363d}
  #badge.CONCLUIDO {background:#0f3d20;color:#3fb950;border-color:#238636}
  #badge.EMERGENCIA{background:#3d0f0f;color:#f85149;border-color:#da3633}
  #badge.FALHA     {background:#3d0f0f;color:#f85149;border-color:#da3633}
  #badge.PAUSA     {background:#2d1f00;color:#e3b341;border-color:#9e6a03}
  /* ---- Segmento ---- */
  #segTxt{font-size:.85rem;color:#8b949e}
  /* ---- Botões ---- */
  .btn{padding:7px 18px;border-radius:6px;border:none;cursor:pointer;font-size:.85rem;
       font-weight:600;transition:opacity .2s}
  .btn:hover{opacity:.85}
  #btnStart{background:#238636;color:#fff}
  #btnCsv{background:#1f6feb;color:#fff}
  /* ---- Grid de gráficos ---- */
  #charts{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:14px;margin-top:14px}
  .card{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:12px}
  .card h2{font-size:.8rem;color:#8b949e;margin-bottom:8px;text-transform:uppercase;letter-spacing:.06em}
  canvas{width:100%!important;max-height:200px}
</style>
</head>
<body>
<h1>🤖 Robô 2WD — Telemetria V4</h1>

<div id="statusBar">
  <div id="dot"></div>
  <span id="connTxt">Desconectado</span>
  <span id="badge" class="AGUARDO">AGUARDO</span>
  <span id="segTxt">Lado — de 4</span>
  <button class="btn" id="btnStart" onclick="sendCmd('start')">▶ START</button>
  <button class="btn" id="btnCsv"   onclick="downloadCsv()">⬇ Download CSV</button>
</div>

<div id="charts">
  <div class="card"><h2>RPM (Esq / Dir)</h2><canvas id="cRpm"></canvas></div>
  <div class="card"><h2>Yaw (°)</h2>        <canvas id="cYaw"></canvas></div>
  <div class="card"><h2>PWM µs (M1 / M2)</h2><canvas id="cPwm"></canvas></div>
</div>

<script>
// ---- Dados históricos para CSV ----
const csvRows = [];

// ---- Janela deslizante (15 s a 5 Hz = 75 amostras) ----
const WIN = 75;

function mkDs(label, color, dash){
  return {label, data:[], borderColor:color, backgroundColor:'transparent',
          borderWidth:1.5, pointRadius:0, borderDash:dash||[]};
}
function mkChart(id, ds, yLabel){
  return new Chart(document.getElementById(id),{
    type:'line',
    data:{labels:[], datasets:ds},
    options:{animation:false, responsive:true, maintainAspectRatio:false,
      plugins:{legend:{labels:{color:'#8b949e',boxWidth:12,font:{size:11}}}},
      scales:{
        x:{display:false},
        y:{ticks:{color:'#8b949e',font:{size:10}},grid:{color:'#21262d'},
           title:{display:!!yLabel,text:yLabel,color:'#8b949e',font:{size:10}}}
      }}
  });
}

const charts = {
  rpm: mkChart('cRpm',[mkDs('Esq','#58a6ff'), mkDs('Dir','#f78166')]),
  yaw: mkChart('cYaw',[mkDs('Yaw','#3fb950')]),
  pwm: mkChart('cPwm',[mkDs('M1','#d2a8ff'), mkDs('M2','#ffa657')])
};

function pushPoint(chart, label, ...vals){
  chart.data.labels.push(label);
  vals.forEach((v,i)=>chart.data.datasets[i].data.push(v));
  if(chart.data.labels.length > WIN){
    chart.data.labels.shift();
    chart.data.datasets.forEach(ds=>ds.data.shift());
  }
  chart.update('none');
}

// ---- WebSocket ----
let ws;
function connect(){
  ws = new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen  = ()=>{ setConn(true); };
  ws.onclose = ()=>{ setConn(false); setTimeout(connect,3000); };
  ws.onerror = ()=>{ ws.close(); };
  ws.onmessage = e=>{ try{ handle(JSON.parse(e.data)); }catch(_){} };
}
function setConn(on){
  document.getElementById('dot').className     = on?'on':'';
  document.getElementById('connTxt').textContent = on?'Conectado':'Desconectado';
}
function sendCmd(cmd){ if(ws && ws.readyState===1) ws.send(cmd); }

// Badge colorido por estado
const STATE_CLS = ['RETA','PIVO','AGUARDO','CONCLUIDO','EMERGENCIA','FALHA','PAUSA'];
function handle(d){
  // Atualizar badge
  const badge = document.getElementById('badge');
  badge.textContent = d.st;
  badge.className   = STATE_CLS.includes(d.st) ? d.st : 'AGUARDO';
  // Atualizar segmento
  document.getElementById('segTxt').textContent = 'Lado '+d.sd+' de '+d.of;
  // Gráficos
  const t = (d.ts/1000).toFixed(1)+'s';
  pushPoint(charts.rpm, t, d.r1, d.r2);
  pushPoint(charts.yaw, t, d.yaw);
  pushPoint(charts.pwm, t, d.w1, d.w2);
  // CSV buffer
  csvRows.push([d.ts,d.st,d.sd,d.of,d.yaw,d.r1,d.r2,d.t1,d.t2,d.tg,d.w1,d.w2,d.rt]);
}

// Download CSV
function downloadCsv(){
  if(!csvRows.length){ alert('Sem dados ainda.'); return; }
  const hdr = 'ts,estado,lado,totalLados,yaw,rpm1,rpm2,ticks1,ticks2,ticksAlvo,pwm1,pwm2,rpmAlvo\n';
  const body = csvRows.map(r=>r.join(',')).join('\n');
  const blob = new Blob([hdr+body],{type:'text/csv'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'telemetria_'+Date.now()+'.csv';
  a.click();
}

connect();
</script>
</body>
</html>
)rawliteral";

// ─────────────────────────────────────────────────────────────────────────────
// Classe WifiTelemetry
// ─────────────────────────────────────────────────────────────────────────────
class WifiTelemetry {
public:
    // -------------------------------------------------------------------------
    // begin() — conecta WiFi, inicia mDNS, servidor HTTP e WebSocket
    // Retorna true se tudo ok, false se WiFi não conectou (não trava o setup)
    // -------------------------------------------------------------------------
    bool begin() {
        Serial.print(F("[WiFi] Conectando a '"));
        Serial.print(WIFI_SSID);
        Serial.print(F("' "));

        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED) {
            if (millis() - t0 > WIFI_CONNECT_TIMEOUT_MS) {
                Serial.println(F("\n[WiFi] Timeout! Continuando sem Wi-Fi."));
                return false;  // tolerante a falha — não trava
            }
            delay(250);
            Serial.print('.');
        }

        Serial.print(F("\n[WiFi] Conectado! IP: "));
        Serial.println(WiFi.localIP());

        // mDNS — acessível como robo.local
        if (MDNS.begin(WIFI_MDNS_NAME)) {
            Serial.print(F("[mDNS] Hostname: "));
            Serial.print(WIFI_MDNS_NAME);
            Serial.println(F(".local"));
        } else {
            Serial.println(F("[mDNS] Falha ao iniciar mDNS."));
        }

        // Registra handler HTTP para "/"
        _http.on("/", [this]() {
            _http.send_P(200, "text/html", DASHBOARD_HTML);
        });
        _http.begin();
        Serial.print(F("[HTTP] Servidor na porta "));
        Serial.println(WIFI_HTTP_PORT);

        // Inicia WebSocket na porta 81
        _ws.begin();
        _ws.onEvent([this](uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
            _onWsEvent(num, type, payload, length);
        });
        Serial.print(F("[WS] WebSocket na porta "));
        Serial.println(WIFI_WS_PORT);

        return true;
    }

    // -------------------------------------------------------------------------
    // handle() — chamar no loop(): processa eventos WS e requisições HTTP
    // -------------------------------------------------------------------------
    void handle() {
        _ws.loop();
        _http.handleClient();
    }

    // -------------------------------------------------------------------------
    // sendTelemetry() — serializa TelemetryPacket em JSON e transmite via WS
    // -------------------------------------------------------------------------
    void sendTelemetry(const TelemetryPacket& pkt) {
        if (!hasClients()) return;

        // Chaves curtas para economizar banda
        JsonDocument doc;
        doc["ts"]  = pkt.ts;
        doc["st"]  = pkt.estado;
        doc["sd"]  = pkt.lado;
        doc["of"]  = pkt.totalLados;
        doc["yaw"] = serialized(String(pkt.yaw,  2));
        doc["r1"]  = serialized(String(pkt.rpm1, 1));
        doc["r2"]  = serialized(String(pkt.rpm2, 1));
        doc["t1"]  = pkt.ticks1;
        doc["t2"]  = pkt.ticks2;
        doc["tg"]  = pkt.ticksAlvo;
        doc["w1"]  = pkt.pwm1;
        doc["w2"]  = pkt.pwm2;
        doc["rt"]  = serialized(String(pkt.rpmAlvo, 1));

        String json;
        serializeJson(doc, json);
        _ws.broadcastTXT(json);
    }

    // -------------------------------------------------------------------------
    // hasNewCommand() — retorna true se chegou novo comando via WS (consome flag)
    // -------------------------------------------------------------------------
    bool hasNewCommand() {
        if (_newCommand) {
            _newCommand = false;
            return true;
        }
        return false;
    }

    // -------------------------------------------------------------------------
    // getLastCommand() — retorna último comando: 's'=start, 'x'=stop, etc.
    // -------------------------------------------------------------------------
    char getLastCommand() {
        return _lastCmd;
    }

    // -------------------------------------------------------------------------
    // isConnected() — WiFi conectado?
    // -------------------------------------------------------------------------
    bool isConnected() const {
        return WiFi.status() == WL_CONNECTED;
    }

    // -------------------------------------------------------------------------
    // hasClients() — tem pelo menos um cliente WebSocket conectado?
    // -------------------------------------------------------------------------
    bool hasClients() const {
        return _clientCount > 0;
    }

    // -------------------------------------------------------------------------
    // getIP() — IP do robô na rede
    // -------------------------------------------------------------------------
    String getIP() const {
        return WiFi.localIP().toString();
    }

private:
    WebServer         _http{WIFI_HTTP_PORT};
    WebSocketsServer  _ws{WIFI_WS_PORT};

    volatile bool _newCommand = false;  // flag para novo comando recebido
    char          _lastCmd    = '\0';   // último comando
    uint8_t       _clientCount = 0;     // número de clientes WS ativos

    // -------------------------------------------------------------------------
    // _onWsEvent() — callback interno de eventos WebSocket
    // -------------------------------------------------------------------------
    void _onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
        switch (type) {
            case WStype_CONNECTED:
                _clientCount++;
                Serial.print(F("[WS] Cliente conectado #"));
                Serial.println(num);
                break;

            case WStype_DISCONNECTED:
                if (_clientCount > 0) _clientCount--;
                Serial.print(F("[WS] Cliente desconectado #"));
                Serial.println(num);
                break;

            case WStype_TEXT: {
                // Converte payload para String para comparação
                String msg;
                msg.reserve(length);
                for (size_t i = 0; i < length; i++) msg += (char)payload[i];
                msg.trim();

                Serial.print(F("[WS] Comando recebido: "));
                Serial.println(msg);

                if (msg == "start" || msg == "s") {
                    _newCommand = true;
                    _lastCmd    = 's';
                } else if (msg == "stop") {
                    _lastCmd = 'x';
                } else if (msg == "pause" || msg == "p") {
                    _newCommand = true;
                    _lastCmd    = 'p';
                } else if (msg == "reset" || msg == "r") {
                    _newCommand = true;
                    _lastCmd    = 'r';
                }
                break;
            }

            default:
                break;
        }
    }
};

#endif // WIFI_TELEMETRY_H
