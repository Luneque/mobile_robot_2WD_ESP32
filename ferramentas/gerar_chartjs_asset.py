#!/usr/bin/env python3
"""Gera chartjs_asset.h / chartjs_asset.cpp a partir do Chart.js UMD minificado.

Uso:
    python ferramentas/gerar_chartjs_asset.py [caminho/chart.umd.min.js]

Sem argumento, baixa a versao definida em VERSAO do jsDelivr.

Por que este script existe: o array de 70 KB em PROGMEM fica em um .cpp
proprio, entao editar o dashboard (wifi_telemetry.h) nao recompila o asset.
Regenerar o asset e a unica ocasiao em que esse .cpp e tocado.
"""
import gzip
import os
import sys
import urllib.request

VERSAO = "4.5.1"
URL = f"https://cdn.jsdelivr.net/npm/chart.js@{VERSAO}/dist/chart.umd.min.js"

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DESTINO = os.path.join(BASE, "firmware_robo2wd_v4")


def obter_fonte(caminho=None):
    if caminho:
        with open(caminho, "rb") as f:
            return f.read()
    print(f"baixando {URL} ...")
    with urllib.request.urlopen(URL, timeout=60) as r:
        return r.read()


def main():
    caminho = sys.argv[1] if len(sys.argv) > 1 else None
    js = obter_fonte(caminho)
    gz = gzip.compress(js, 9, mtime=0)

    h = f"""// =========================================================================
// Chart.js {VERSAO} (UMD, minificado) — servido localmente pelo robo.
// Licenca MIT: https://github.com/chartjs/Chart.js/blob/master/LICENSE.md
//
// O dashboard nao depende mais de CDN: o browser baixa /chartjs.min.js do
// proprio ESP32. O conteudo e gzip ({len(gz)/1024:.1f} KB em vez de
// {len(js)/1024:.1f} KB) e o servidor responde com Content-Encoding: gzip.
//
// Os dados ficam em chartjs_asset.cpp (compilado uma unica vez; editar este
// header ou o dashboard nao recompila o array de {len(gz)/1024:.1f} KB).
//
// GERADO por ferramentas/gerar_chartjs_asset.py — nao edite a mao.
// =========================================================================
#pragma once
#include <Arduino.h>

#define CHARTJS_GZ_LEN {len(gz)}U

// Array de bytes em PROGMEM, definido em chartjs_asset.cpp
extern const uint8_t CHARTJS_GZ[];
"""
    cpp = ["// GERADO por ferramentas/gerar_chartjs_asset.py — nao edite a mao.",
           f"// Chart.js {VERSAO} UMD min, gzip. Ver chartjs_asset.h.",
           '#include "chartjs_asset.h"',
           "",
           "const uint8_t CHARTJS_GZ[] PROGMEM = {"]
    for i in range(0, len(gz), 20):
        cpp.append("  " + ",".join(f"0x{b:02x}" for b in gz[i:i + 20]) + ",")
    cpp.append("};")

    with open(os.path.join(DESTINO, "chartjs_asset.h"), "w", encoding="utf-8", newline="\n") as f:
        f.write(h)
    with open(os.path.join(DESTINO, "chartjs_asset.cpp"), "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(cpp) + "\n")

    print(f"js {len(js)} bytes -> gzip {len(gz)} bytes")
    print("chartjs_asset.h / chartjs_asset.cpp regenerados")


if __name__ == "__main__":
    main()
