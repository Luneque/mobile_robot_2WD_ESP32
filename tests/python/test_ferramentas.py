#!/usr/bin/env python3
"""Testes das ferramentas de telemetria (sem robô, sem rede).

Cobrem o que já quebrou de verdade: o replay de dump para CSV já falhou
duas vezes quando o número de colunas mudou (12 -> 13 -> 14). Estes testes
travam o formato.

Rode com:
    python -m unittest discover -s tests/python -v
"""
import csv
import pathlib
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

    def test_cabecalho_tem_13_colunas(self):
        # O dump nao tem total_lados (economia bytes na RAM); o CSV do gateway
        # tem 14 porque o JSON do ESP-NOW carrega o campo "of".
        self.assertEqual(len(self.linhas[0]), 13, "CSV com número de colunas diferente do dump")

    def test_cabecalho_inclui_yaw_abs(self):
        self.assertIn("yaw_abs_deg", self.linhas[0],
                      "coluna do rumo global sumiu — o mapa 2D depende dela")

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

    def test_gateway_csv_tem_14_colunas(self):
        # O CSV do gateway inclui total_lados (campo "of" do JSON), por isso
        # tem uma coluna a mais que o CSV do replay.
        self.assertEqual(gateway_espnow.CABECALHO_CSV, [
            "timestamp_ms", "estado", "lado", "total_lados", "yaw_deg",
            "yaw_abs_deg", "rpm_esq", "rpm_dir", "ticks_esq", "ticks_dir",
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


if __name__ == "__main__":
    unittest.main(verbosity=2)
