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
#include "chartjs_asset.h"   // Chart.js 4.5.1 gz (servido em /chartjs.min.js)

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

// ─────────────────────────────────────────────────────────────────────────────
// Struct de Telemetria
// ─────────────────────────────────────────────────────────────────────────────
struct TelemetryPacket {
    uint32_t    ts;          // millis() desde o boot
    const char* estado;      // "AGUARDO","RETA","PIVO","PAUSA","CONCLUIDO","EMERGENCIA","FALHA"
    int         lado;        // segmento atual (1–4)
    int         totalLados;  // sempre 4
    float       yaw;         // graus (com sinal, relativo ao início do segmento)
    float       yawAbs;      // graus (absoluto desde o boot; usado no mapa 2D)
    float       yawOdom;     // graus (rumo por odometria dos encoders, ticks com sinal)
    float       diverg;      // graus (yawAbs - yawOdom): divergencia IMU x odometria
    float       gyroBias;    // graus/s (bias residual do giroscopio, medido no boot)
    int32_t     odomSoma;    // soma de (dT1 - dT2) integrada (diagnostico)
    uint32_t    odomCiclos;  // quantos ciclos de controle integraram a odometria
    float       inercia;     // graus (giro da IMU depois do corte, na pausa)
    float       inerciaOdom; // graus (giro da odometria depois do corte)
    float       yawReal;     // graus (yawAbs + inercia: o rumo real)
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
<script src="/chartjs.min.js"></script>
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
  /* ---- Mapa de trajetória ---- */
  .map-card{max-width:440px;margin:0 auto 14px}
  #cMap{width:min(100%,400px)!important;max-height:none;aspect-ratio:1;background:#0d1117;border-radius:6px;display:block;margin:0 auto}
  .map-legend{display:flex;align-items:center;gap:12px;margin-top:8px;font-size:.72rem;color:#8b949e;flex-wrap:wrap}
  .map-legend i{display:inline-block;vertical-align:middle;margin-right:2px}
  .map-legend i.bl{width:14px;height:3px;border-radius:2px;background:#388bfd}
  .map-legend i.gr{width:14px;height:3px;border-radius:2px;background:#3fb950}
  .map-legend i.or{width:8px;height:8px;border-radius:50%;background:#f0883e}
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
  <button class="btn" id="btnReta"  onclick="sendCmd('t')" title="Uma reta de 500 mm, para no fim (diagnóstico de escala)">📏 Reta 500</button>
  <button class="btn" id="btnPivo"  onclick="sendCmd('g')" title="Um pivô de 90° no lugar, para no fim (diagnóstico de ângulo)">🔄 Pivô 90</button>
  <button class="btn" id="btnCsv"   onclick="downloadCsv()">⬇ Download CSV</button>
  <button class="btn" id="btnDump"  onclick="sendCmd('dump')">🗒 Dump RAM</button>
</div>

<div id="charts">
  <div class="card map-card">
    <h2>Mapa — Trajetória Real × Ideal</h2>
    <canvas id="cMap" width="400" height="400"></canvas>
    <div class="map-legend">
      <span><i class="bl"></i> Ideal (500mm)</span>
      <span><i class="gr"></i> Real</span>
      <span><i class="or"></i> Posição</span>
      <button class="btn" onclick="resetMap()" style="margin-left:auto;background:#21262d;color:#8b949e;font-size:.72rem;padding:3px 10px;border:1px solid #30363d">↺ Reset</button>
    </div>
    <div id="diag" style="margin-top:8px;font:11px 'Segoe UI',monospace;color:#8b949e">
      aguardando telemetria…
    </div>
  </div>
  <div class="card"><h2>RPM (Esq / Dir)</h2><canvas id="cRpm"></canvas></div>
  <div class="card"><h2>Yaw (°)</h2>        <canvas id="cYaw"></canvas></div>
  <div class="card"><h2>PWM µs (M1 / M2)</h2><canvas id="cPwm"></canvas></div>
</div>

<script>
// ---- Dados históricos para CSV ----
const csvRows = [];

// ---- Janela deslizante (15 s a 5 Hz = 75 amostras) ----
const WIN = 75;

// ---- Odometria / Mapa 2D ----
const MM_PER_TICK = 0.04799656; // 66mm diametro / 4320 ticks
let posX = 500, posY = 0; // Ponto de partida real: Início (500, 0)
let prevTicks = 0, prevSegSide = -1, prevSegState = '';
let trajectory = [{x: 500, y: 0}];
let curHeadingDeg = 0;      // Rumo GLOBAL real (0 = +Y, 90 = +X), integrado do yaw
let yawAbs = 0;             // Rumo global (graus, horário = positivo), integrado do yaw
let lastSt = '', lastSide = -1;

function resetMap() {
  posX = 500; posY = 0;
  // Mantém o rumo global atual: o yaw_abs do firmware é acumulado desde o
  // boot e não zera a cada trajeto, então zerar aqui dessintonizaria o mapa.
  curHeadingDeg = yawParaRumo(yawAbs);
  prevTicks = 0; prevSegSide = -1; prevSegState = '';
  trajectory = [{x: 500, y: 0}];
  drawMap();
}

// Converte o yaw absoluto do firmware para o rumo global do mapa.
// O BMI160 tem o eixo Z para cima: yaw positivo = rotação ANTI-HORÁRIA
// (regra da mão direita). O mapa usa ângulos HORÁRIO-positivos
// (0 = +Y, 90 = +X), então o rumo é o negativo do yaw.
function yawParaRumo(graus) {
  return ((-graus) % 360 + 360) % 360;
}

function updateOdometry(d) {
  const curAvgTicks = (d.t1 + d.t2) / 2;

  // Detecta transição de segmento (mudou estado ou lado)
  const mudouSeg = (d.st !== lastSt || d.sd !== lastSide);
  lastSt = d.st; lastSide = d.sd;

  if (mudouSeg) {
    // Ao iniciar a 1ª reta (lado 1) depois de várias voltas, reinicia o rastro
    if (d.st === 'RETA' && d.sd === 1 && trajectory.length > 500) {
      posX = 500; posY = 0;
      trajectory = [{x: 500, y: 0}];
    }
    prevTicks = curAvgTicks;
  }

  // Integra o rumo global a partir do yaw absoluto da IMU.
  // Na reta, o robô anda no rumo corrente; o yaw medido já inclui o desvio real.
  if (d.st === 'RETA') {
    yawAbs = d.yawAbs !== undefined ? d.yawAbs : yawAbs;
    curHeadingDeg = yawParaRumo(yawAbs);

    const deltaTicks = curAvgTicks - prevTicks;
    if (deltaTicks > 0) {
      const deltaDist = deltaTicks * MM_PER_TICK;
      const rad = (curHeadingDeg * Math.PI) / 180.0;
      posX += deltaDist * Math.sin(rad);
      posY += deltaDist * Math.cos(rad);
      trajectory.push({x: posX, y: posY});
      prevTicks = curAvgTicks;
    }
  } else if (d.st === 'PIVO') {
    // No pivô, o yaw absoluto cresce (giro horário); o rumo acompanha.
    yawAbs = d.yawAbs !== undefined ? d.yawAbs : yawAbs;
    curHeadingDeg = yawParaRumo(yawAbs);
    prevTicks = curAvgTicks;
  }
}

function drawMap() {
  const canvas = document.getElementById('cMap');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  
  ctx.clearRect(0, 0, w, h);
  
  // Grade de fundo suave (100mm por quadrícula)
  ctx.save();
  ctx.strokeStyle = '#161b22';
  ctx.lineWidth = 1;
  const margin = 40;
  // Janela de visão em milímetros: o percurso é o quadrado anti-horário
  // (500,0)->(500,500)->(0,500)->(0,0), com margem para os rótulos.
  const viewW = 700, viewH = 700;
  const scale = Math.min((w - 2 * margin) / viewW, (h - 2 * margin) / viewH);
  const cx = 250, cy = 250;   // centro da visão, em mm
  
  // Transformação: X cresce para a direita, Y cresce para cima
  // (o canvas tem Y para baixo, daí o sinal negativo)
  function toScreen(x, y) {
    return { x: w / 2 + (x - cx) * scale, y: h / 2 - (y - cy) * scale };
  }
  
  // Linhas de grade
  ctx.strokeStyle = '#21262d';
  for (let gx = -100; gx <= 600; gx += 100) {
    const p1 = toScreen(gx, -100), p2 = toScreen(gx, 600);
    ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();
  }
  for (let gy = -100; gy <= 600; gy += 100) {
    const p1 = toScreen(-100, gy), p2 = toScreen(600, gy);
    ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();
  }
  
  // 1. Quadrado Ideal (500x500 mm) em Azul tracejado.
  // Sequência definida do trajeto (giro à esquerda / anti-horário):
  // Início (500,0) -> L1 (500,500) -> L2 (0,500) -> L3 (0,0) -> Início
  const pInicio = toScreen(500, 0);
  const pL1     = toScreen(500, 500);
  const pL2     = toScreen(0, 500);
  const pL3     = toScreen(0, 0);
  
  ctx.strokeStyle = '#388bfd';
  ctx.lineWidth = 2;
  ctx.setLineDash([6, 4]);
  ctx.beginPath();
  ctx.moveTo(pInicio.x, pInicio.y);
  ctx.lineTo(pL1.x, pL1.y);
  ctx.lineTo(pL2.x, pL2.y);
  ctx.lineTo(pL3.x, pL3.y);
  ctx.closePath();
  ctx.stroke();
  ctx.setLineDash([]);
  
  // Rótulo dos Vértices ideais
  ctx.fillStyle = '#58a6ff';
  ctx.font = '10px "Segoe UI", sans-serif';
  ctx.fillText('Início (500,0)', pInicio.x - 30, pInicio.y + 16);
  ctx.fillText('L1 (500,500)', pL1.x - 15, pL1.y - 8);
  ctx.fillText('L2 (0,500)', pL2.x - 20, pL2.y - 8);
  ctx.fillText('L3 (0,0)', pL3.x - 20, pL3.y + 16);
  
  // 2. Trajetória Real Percorrida (Rastro Verde)
  if (trajectory.length > 1) {
    ctx.strokeStyle = '#3fb950';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    const sp0 = toScreen(trajectory[0].x, trajectory[0].y);
    ctx.moveTo(sp0.x, sp0.y);
    for (let i = 1; i < trajectory.length; i++) {
      const sp = toScreen(trajectory[i].x, trajectory[i].y);
      ctx.lineTo(sp.x, sp.y);
    }
    ctx.stroke();
  }
  
  // 3. Posição Atual do Robô (Ponto Laranja com vetor de orientação)
  const curPos = toScreen(posX, posY);
  
  // Triângulo ou Círculo indicador do Robô
  ctx.fillStyle = '#f0883e';
  ctx.beginPath();
  ctx.arc(curPos.x, curPos.y, 6, 0, Math.PI * 2);
  ctx.fill();
  
  // Vetor de direção (orientação do robô)
  const headingRad = (curHeadingDeg * Math.PI) / 180.0;
  const arrowLen = 18;
  const arrowX = curPos.x + arrowLen * Math.sin(headingRad);
  const arrowY = curPos.y - arrowLen * Math.cos(headingRad);
  ctx.strokeStyle = '#ffa657';
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(curPos.x, curPos.y);
  ctx.lineTo(arrowX, arrowY);
  ctx.stroke();
  
  // Coordenadas em texto
  ctx.fillStyle = '#c9d1d9';
  ctx.font = '11px "Segoe UI", monospace';
  ctx.fillText(`X: ${posX.toFixed(0)} mm | Y: ${posY.toFixed(0)} mm`, 10, 20);
  ctx.fillText(`θ: ${(curHeadingDeg % 360.0).toFixed(1)}°`, 10, 36);
  
  ctx.restore();
}

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
  // Atualiza mapa 2D
  updateOdometry(d);
  drawMap();
  // Diagnostico: confere o rumo da IMU contra o da odometria e o bias do giroscopio
  atualizarDiag(d);
  // CSV buffer
  csvRows.push([d.ts,d.st,d.sd,d.of,d.yaw,d.yawAbs,d.yawOdom,d.diverg,d.gyroBias,
                d.r1,d.r2,d.t1,d.t2,d.tg,d.w1,d.w2,d.rt]);
}

// Painel de diagnostico: divergencia IMU x odometria + bias de calibragem.
// Se o giroscopio estiver errado, |diverg| cresce; se o boot foi contaminado,
// gyroBias aparece alto.
function atualizarDiag(d){
  const el = document.getElementById('diag');
  if(!el) return;
  const div = (d.diverg!==undefined) ? d.diverg : (d.yawAbs - d.yawOdom);
  const odm = (d.yawOdom!==undefined) ? d.yawOdom : NaN;
  const bias = (d.gyroBias!==undefined) ? d.gyroBias : NaN;
  const ine = (d.inercia!==undefined) ? d.inercia : 0;
  const real = (d.yawReal!==undefined) ? d.yawReal : (d.yawAbs + ine);
  const okDiv = Math.abs(div) < 5.0;
  const okBias = Math.abs(bias) < 0.5;
  const cor = (okDiv && okBias) ? '#3fb950' : (okDiv ? '#d29922' : '#f85149');
  el.innerHTML =
    'rumo corte <b style="color:#58a6ff">' + ((d.yawAbs||0).toFixed(1)) + '°</b> · ' +
    'inércia <b style="color:#f38021">' + (ine>=0?'+':'') + ine.toFixed(1) + '°</b> · ' +
    'rumo real <b style="color:#3fb950">' + (real>=0?'+':'') + real.toFixed(1) + '°</b> · ' +
    'rumo odometria <b style="color:#a371f7">' + (isNaN(odm)?'—':odm.toFixed(1)) + '°</b> · ' +
    'divergência <b style="color:' + cor + '">' + (isNaN(div)?'—':(div>=0?'+':'') + div.toFixed(1)) + '°</b>' +
    (okDiv ? '' : ' ⚠') +
    ' &nbsp;|&nbsp; bias giro <b style="color:' + (okBias?'#3fb950':'#f85149') + '">' +
    (isNaN(bias)?'—':(bias>=0?'+':'') + bias.toFixed(2)) + ' °/s</b>' +
    (okBias ? '' : ' ⚠ boot contaminado?');
}

// Download CSV
function downloadCsv(){
  if(!csvRows.length){ alert('Sem dados ainda.'); return; }
  const hdr = 'ts,estado,lado,totalLados,yaw,yawAbs,yawOdom,diverg,gyroBias,rpm1,rpm2,ticks1,ticks2,ticksAlvo,pwm1,pwm2,rpmAlvo\n';
  const body = csvRows.map(r=>r.join(',')).join('\n');
  const blob = new Blob([hdr+body],{type:'text/csv'});
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'telemetria_'+Date.now()+'.csv';
  a.click();
}

connect();
drawMap();
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
        // Chart.js servido localmente (gzip). O dashboard nao depende de CDN.
        _http.on("/chartjs.min.js", [this]() {
            _http.sendHeader("Content-Encoding", "gzip");
            _http.send_P(200, "application/javascript",
                         (PGM_P)CHARTJS_GZ, CHARTJS_GZ_LEN);
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
        doc["yaw"] = roundf(pkt.yaw * 100.0f) / 100.0f;
        doc["yawAbs"] = roundf(pkt.yawAbs * 100.0f) / 100.0f;
        doc["yawOdom"] = roundf(pkt.yawOdom * 100.0f) / 100.0f;
        doc["diverg"] = roundf(pkt.diverg * 100.0f) / 100.0f;
        doc["gyroBias"] = roundf(pkt.gyroBias * 1000.0f) / 1000.0f;
        doc["odomSoma"] = pkt.odomSoma;
        doc["odomCiclos"] = pkt.odomCiclos;
        doc["inercia"] = roundf(pkt.inercia * 100.0f) / 100.0f;
        doc["inerciaOdom"] = roundf(pkt.inerciaOdom * 100.0f) / 100.0f;
        doc["yawReal"] = roundf(pkt.yawReal * 100.0f) / 100.0f;
        doc["r1"]  = roundf(pkt.rpm1 * 10.0f) / 10.0f;
        doc["r2"]  = roundf(pkt.rpm2 * 10.0f) / 10.0f;
        doc["t1"]  = pkt.ticks1;
        doc["t2"]  = pkt.ticks2;
        doc["tg"]  = pkt.ticksAlvo;
        doc["w1"]  = pkt.pwm1;
        doc["w2"]  = pkt.pwm2;
        doc["rt"]  = roundf(pkt.rpmAlvo * 10.0f) / 10.0f;

        String json;
        serializeJson(doc, json);
        _ws.broadcastTXT(json);
    }

    // -------------------------------------------------------------------------
    // sendRaw() — envia texto puro via WebSocket (para o dump do gravador)
    // -------------------------------------------------------------------------
    void sendRaw(const char* texto) {
        if (hasClients()) _ws.broadcastTXT(texto);
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
                } else if (msg == "dump" || msg == "d") {
                    _newCommand = true;   // dispara o tratamento no loop principal
                    _lastCmd    = 'd';    // dump do gravador de telemetria (RAM)
                } else if (msg == "reta" || msg == "t") {
                    _newCommand = true;   // reta unica de 500 mm (diagnostico)
                    _lastCmd    = 't';
                } else if (msg == "pivo" || msg == "g") {
                    _newCommand = true;   // pivo unico de 90 graus (diagnostico)
                    _lastCmd    = 'g';
                }
                break;
            }

            default:
                break;
        }
    }
};

#endif // WIFI_TELEMETRY_H
