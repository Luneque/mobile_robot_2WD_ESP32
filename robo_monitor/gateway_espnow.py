#!/usr/bin/env python3
"""gateway_espnow.py — Ponte ESP-NOW (gateway ESP32-S3) <-> PC

Conecta à porta serial do gateway ESP32-S3 (gravado com gateway_espnow.ino) e:
  - encaminha telemetria recebida ("TELEM:{json}") para o stdout e para CSV;
  - aceita comandos do PC e os envia ao robô no formato "CMD:<payload>".

Uso:
    python gateway_espnow.py [--port COM9] [--baud 115200] [--csv saida.csv]

Comandos aceitos no stdin:
    start | stop | pause | reset | dump   (enviados ao robô via ESP-NOW)

Exemplo:
    python gateway_espnow.py --port COM9 --csv telemetria.csv

Dependências:
    pip install pyserial
"""

import argparse
import csv
import pathlib
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("[ERRO] pyserial não instalado. Rode: pip install pyserial")
    sys.exit(1)


CABECALHO_CSV = [
    "timestamp_ms", "estado", "lado", "total_lados", "yaw_deg", "yaw_abs_deg",
    "yaw_odom_deg", "diverg_deg", "gyro_bias_dps",
    "rpm_esq", "rpm_dir", "ticks_esq", "ticks_dir", "ticks_alvo",
    "pwm_esq_us", "pwm_dir_us", "rpm_alvo",
]


class GatewayPonte:
    def __init__(self, porta: str, baud: int, csv_path: pathlib.Path | None):
        self.porta = porta
        self.baud = baud
        self.csv_path = csv_path
        self._csv_file = None
        self._csv_writer = None
        self._parar = False

    def _abrir_csv(self):
        if self.csv_path is None:
            return
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(CABECALHO_CSV)
        print(f"[csv] {self.csv_path}")

    def _gravar_csv(self, dados: dict):
        if self._csv_writer is None or self._csv_file is None:
            return
        self._csv_writer.writerow([
            dados.get("ts", ""), dados.get("st", ""), dados.get("sd", ""),
            dados.get("of", ""), dados.get("yaw", ""), dados.get("yawAbs", ""),
            dados.get("yawOdom", ""), dados.get("diverg", ""), dados.get("gyroBias", ""),
            dados.get("r1", ""),
            dados.get("r2", ""), dados.get("t1", ""), dados.get("t2", ""),
            dados.get("tg", ""), dados.get("w1", ""), dados.get("w2", ""),
            dados.get("rt", ""),
        ])
        self._csv_file.flush()

    def _enviar_comando(self, ser: serial.Serial, cmd: str):
        linha = f"CMD:{cmd}\n".encode()
        ser.write(linha)
        ser.flush()
        print(f"[pc->robo] {cmd}")

    def _teclado(self, ser_holder: dict):
        """Lê comandos do stdin em thread separada."""
        while not self._parar:
            try:
                linha = sys.stdin.readline()
            except (KeyboardInterrupt, EOFError):
                break
            if not linha:
                break
            cmd = linha.strip()
            if not cmd:
                continue
            if cmd in ("start", "stop", "pause", "reset", "dump"):
                ser = ser_holder.get("ser")
                if ser is not None and ser.is_open:
                    self._enviar_comando(ser, cmd)
                else:
                    print("[aviso] serial não conectada; comando ignorado")
            elif cmd in ("sair", "exit", "quit"):
                print("[fim] encerrando...")
                self._parar = True
                break
            else:
                print(f"[ignorado] comando desconhecido: {cmd!r}")

    def executar(self):
        self._abrir_csv()
        print(f"[serial] conectando a {self.porta} @ {self.baud}")
        try:
            ser = serial.Serial(self.porta, self.baud, timeout=1)
        except serial.SerialException as e:
            print(f"[ERRO] não foi possível abrir {self.porta}: {e}")
            return 1

        ser_holder = {"ser": ser}
        t = threading.Thread(target=self._teclado, args=(ser_holder,), daemon=True)
        t.start()

        print("[pronto] comandos: start | stop | pause | reset | dump | sair")
        try:
            while not self._parar:
                try:
                    linha = ser.readline()
                except serial.SerialException as e:
                    print(f"[ERRO] serial: {e}")
                    break
                if not linha:
                    continue
                texto = linha.decode("utf-8", errors="replace").strip()
                if not texto:
                    continue

                if texto.startswith("TELEM:"):
                    payload = texto[len("TELEM:"):]
                    print(f"[robo] {payload}")
                    self._processar_telem(payload)
                elif texto.startswith("MAC:"):
                    print(f"[gateway] {texto}")
                elif texto.startswith("DBG:"):
                    print(f"[gateway] {texto}")
                else:
                    print(f"[serial] {texto}")
        except KeyboardInterrupt:
            print("\n[fim] interrompido")
        finally:
            self._parar = True
            ser.close()
            if self._csv_file and not self._csv_file.closed:
                self._csv_file.flush()
                self._csv_file.close()
        return 0

    def _processar_telem(self, payload: str):
        import json
        # O gateway prefixa TODO payload com "TELEM:", inclusive as linhas do
        # dump do gravador ("REC,..."). Trata os dois formatos.
        if payload.startswith("REC,"):
            print(f"  {payload}")
            return
        try:
            dados = json.loads(payload)
        except json.JSONDecodeError:
            print(f"[aviso] TELEM inválido: {payload[:80]}")
            return
        self._gravar_csv(dados)


def main():
    p = argparse.ArgumentParser(description="Ponte ESP-NOW: gateway ESP32-S3 <-> PC")
    p.add_argument("--port", default="COM9", help="Porta serial do gateway (padrão: COM9)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--csv", default="telemetria_espnow.csv", help="CSV de saída")
    args = p.parse_args()

    csv_path = pathlib.Path(args.csv) if args.csv else None
    ponte = GatewayPonte(args.port, args.baud, csv_path)
    sys.exit(ponte.executar())


if __name__ == "__main__":
    main()
