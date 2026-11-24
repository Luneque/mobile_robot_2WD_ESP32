// =========================================================================
// Chart.js 4.5.1 (UMD, minificado) — servido localmente pelo robo.
// Licenca MIT: https://github.com/chartjs/Chart.js/blob/master/LICENSE.md
//
// O dashboard nao depende mais de CDN: o browser baixa /chartjs.min.js do
// proprio ESP32. O conteudo e gzip (70.813 bytes, ~69,2 KiB, em vez de
// 203.6 KB) e o servidor responde com Content-Encoding: gzip.
//
// Os dados ficam em chartjs_asset.cpp (compilado uma unica vez; editar este
// header ou o dashboard nao recompila o array de 70.813 bytes).
//
// GERADO por ferramentas/gerar_chartjs_asset.py — nao edite a mao.
// =========================================================================
#pragma once
#include <Arduino.h>

#define CHARTJS_GZ_LEN 70813U

// Array de bytes em PROGMEM, definido em chartjs_asset.cpp
extern const uint8_t CHARTJS_GZ[];
