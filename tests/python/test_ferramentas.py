#!/usr/bin/env python3
"""Testes das ferramentas de telemetria (sem robô, sem rede).

Cobrem o que já quebrou de verdade: o replay de dump para CSV já falhou
duas vezes quando o número de colunas mudou (12 -> 13 -> 14). Estes testes
travam o formato.

Rode com:
    python -m unittest discover -s tests/python -v
"""
import csv
import glob
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

RAIZ = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(RAIZ / "robo_monitor"))

import gateway_espnow          # noqa: E402
import robo_ws_logger          # noqa: E402

FIXTURE = RAIZ / "tests" / "fixtures" / "dump_exemplo.txt"
FIRMWARE = RAIZ / "firmware_robo2wd_v4"


class TestReplayDump(unittest.TestCase):
    """O dump do gravador (RAM) precisa virar CSV com o formato vigente."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.saida = pathlib.Path(self.tmp.name) / "replay.csv"
        robo_ws_logger.replay_dump(FIXTURE, self.saida)
        with open(self.saida, newline="", encoding="utf-8") as f:
            self.linhas = list(csv.reader(f))

    def tearDown(self):
        self.tmp.cleanup()

    def test_cabecalho_tem_16_colunas(self):
        # O dump nao tem total_lados (economia bytes na RAM); o CSV do gateway
        # tem 17 porque o JSON do ESP-NOW carrega o campo "of".
        self.assertEqual(len(self.linhas[0]), 16, "CSV com número de colunas diferente do dump")

    def test_cabecalho_inclui_yaw_abs(self):
        self.assertIn("yaw_abs_deg", self.linhas[0],
                      "coluna do rumo global sumiu — o mapa 2D depende dela")

    def test_cabecalho_inclui_diagnostico(self):
        # Diagnostico IMU x odometria: sem essas colunas não dá para conferir
        # o rumo do giroscopio contra os encoders.
        for col in ("yaw_odom_deg", "diverg_deg", "gyro_bias_dps"):
            self.assertIn(col, self.linhas[0], "coluna de diagnóstico ausente: " + col)

    def test_cabecalho_primeira_coluna(self):
        self.assertEqual(self.linhas[0][0], "timestamp_ms")

    def test_numero_de_amostras(self):
        recs = [l for l in FIXTURE.read_text(encoding="utf-8").splitlines()
                if l.startswith("REC,")]
        self.assertEqual(len(self.linhas) - 1, len(recs),
                         "perdeu ou inventou amostras no replay")

    def test_valores_preservados(self):
        recs = [l.split(",") for l in FIXTURE.read_text(encoding="utf-8").splitlines()
                if l.startswith("REC,")]
        # a linha do CSV é a REC sem o prefixo "REC"
        self.assertEqual(self.linhas[1], recs[0][1:])
        self.assertEqual(self.linhas[-1], recs[-1][1:])


class TestConsistenciaDeFormato(unittest.TestCase):
    """O dump, o logger e o gateway precisam falar o mesmo formato."""

    def test_gateway_csv_tem_17_colunas(self):
        # O CSV do gateway inclui total_lados (campo "of" do JSON), por isso
        # tem uma coluna a mais que o CSV do replay.
        self.assertEqual(gateway_espnow.CABECALHO_CSV, [
            "timestamp_ms", "estado", "lado", "total_lados", "yaw_deg",
            "yaw_abs_deg", "yaw_odom_deg", "diverg_deg", "gyro_bias_dps",
            "rpm_esq", "rpm_dir", "ticks_esq", "ticks_dir",
            "ticks_alvo", "pwm_esq_us", "pwm_dir_us", "rpm_alvo",
        ])

    def test_logger_le_a_chave_yawAbs(self):
        src = (RAIZ / "robo_monitor" / "robo_ws_logger.py").read_text(encoding="utf-8")
        self.assertIn('"yawAbs"', src, "logger não lê mais o yawAbs do JSON")

    def test_gateway_le_a_chave_yawAbs(self):
        src = (RAIZ / "robo_monitor" / "gateway_espnow.py").read_text(encoding="utf-8")
        self.assertIn('"yawAbs"', src, "gateway não lê mais o yawAbs do JSON")

    def test_firmware_emite_yawAbs(self):
        src = (FIRMWARE / "wifi_telemetry.h").read_text(encoding="utf-8")
        self.assertIn('doc["yawAbs"]', src, "firmware não emite mais yawAbs no WebSocket")
        # No .ino o JSON e montado com aspas escapadas: \"yawAbs\"
        ino = (FIRMWARE / "firmware_robo2wd_v4.ino").read_text(encoding="utf-8")
        self.assertIn('yawAbs', ino, "firmware não envia mais o rumo global no ESP-NOW")
        self.assertIn('pkt.yawAbs', ino, "pacote de telemetria sem o rumo global")

    def test_dump_usa_yaw_abs(self):
        src = (FIRMWARE / "telemetry_recorder.h").read_text(encoding="utf-8")
        self.assertIn("yaw_abs_deg", src, "cabeçalho do dump sem a coluna do rumo global")

    def test_dashboard_nao_usa_cdn(self):
        src = (FIRMWARE / "wifi_telemetry.h").read_text(encoding="utf-8")
        self.assertNotIn("https://", src, "dashboard voltou a depender de recurso externo")
        self.assertIn('/chartjs.min.js', src)


class TestDashboardJS(unittest.TestCase):
    """O JS do dashboard vive embutido num header .h. Um erro de sintaxe faz a
    pagina inteira parar de funcionar (os graficos congelam, sem nenhum aviso)
    — foi exatamente o que aconteceu. Este teste compila o script com node."""

    def _js_do_dashboard(self):
        src = (FIRMWARE / "wifi_telemetry.h").read_text(encoding="utf-8")
        m = re.search(r'R"rawliteral\((.*?)\)rawliteral"', src, re.S)
        if m is None:
            self.fail("nao achei o bloco DASHBOARD_HTML")
        html = m.group(1)
        m2 = re.search(r"<script>(.*?)</script>", html, re.S)
        if m2 is None:
            self.fail("nao achei o <script> inline do dashboard")
        return m2.group(1)

    def _achar_node(self):
        """Encontra um node que funcione. O 'node' do PATH pode ser um shim do
        nvm que nao executa nada ('No active Node.js version is configured'),
        entao tambem tentamos o node embarcado nas tools do Hermes."""
        candidatos = []
        achado = shutil.which("node")
        if achado:
            candidatos.append(achado)
        candidatos += sorted(glob.glob(os.path.expandvars(
            r"%LOCALAPPDATA%\hermes\tools\node-*\node.exe")))
        for cand in candidatos:
            try:
                r = subprocess.run([cand, "--version"],
                                   capture_output=True, text=True, timeout=20)
                if r.returncode == 0 and r.stdout.strip():
                    return cand
            except Exception:
                continue
        return None

    def test_js_do_dashboard_compila(self):
        js = self._js_do_dashboard()
        node = self._achar_node()
        if not node:
            self.skipTest("nenhum node funcional para validar a sintaxe do JS")
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False,
                                         encoding="utf-8") as f:
            f.write(js)
            caminho = f.name
        try:
            r = subprocess.run([node, "--check", caminho],
                               capture_output=True, text=True)
        finally:
            os.unlink(caminho)
        self.assertEqual(r.returncode, 0,
                         "JS do dashboard com erro de sintaxe:\n" + r.stderr)

    def test_dashboard_tem_painel_de_diagnostico(self):
        js = self._js_do_dashboard()
        for trecho in ("atualizarDiag", "inercia", "yawReal"):
            self.assertIn(trecho, js, "painel de diagnostico sem " + trecho)

    def test_mapa_usa_o_rumo_real(self):
        # O rastro do mapa tem de usar yawReal (corte + inercia). Com yawAbs o
        # traco sai torto, ~5 graus por pivo a menos.
        js = self._js_do_dashboard()
        m = re.search(r"function updateOdometry\(d\)\s*\{(.*?)\n\}", js, re.S)
        if m is None:
            self.fail("nao achei updateOdometry")
        corpo = m.group(1)
        self.assertIn("d.yawReal !== undefined) ? d.yawReal", corpo,
                      "mapa nao esta usando yawReal (rumo real, com inercia)")
        self.assertIn("yawParaRumo(rumoReal)", corpo,
                      "mapa nao esta integrando o rumo real")


if __name__ == "__main__":
    unittest.main(verbosity=2)
