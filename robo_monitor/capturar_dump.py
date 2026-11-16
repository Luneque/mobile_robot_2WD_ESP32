#!/usr/bin/env python3
"""Captura um dump do gravador de telemetria via serial.

Uso:
    python capturar_dump.py --port COM9 --baud 115200 --saida dump.txt [--start 's']

Abre a porta, aguarda o boot, envia o comando de start (opcional), espera o
tempo informado e então envia 'd' para despejar o buffer. Grava tudo em texto.
"""

import argparse
import pathlib
import sys
import time

try:
    import serial
except ImportError:
    print("[ERRO] pyserial não instalado. Rode: pip install pyserial")
    sys.exit(1)


def main():
    p = argparse.ArgumentParser(description="Captura dump do gravador de telemetria.")
    p.add_argument("--port", default="COM9")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--saida", default="dump.txt")
    p.add_argument("--start", action="store_true", help="envia 's' para iniciar a trajetória")
    p.add_argument("--espera", type=float, default=45.0, help="segundos até o dump")
    p.add_argument("--pos-dump", type=float, default=8.0, help="segundos de leitura após o dump")
    args = p.parse_args()

    saida = pathlib.Path(args.saida)
    print(f"[captura] porta {args.port} @ {args.baud} -> {saida}")

    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        # Drena o que veio do boot
        time.sleep(0.5)
        ser.reset_input_buffer()

        if args.start:
            print("[captura] enviando 's' (start)...")
            ser.write(b"s")
            ser.flush()

        print(f"[captura] aguardando {args.espera:.0f}s de execução...")
        time.sleep(args.espera)

        print("[captura] enviando 'd' (dump)...")
        ser.write(b"d")
        ser.flush()

        print(f"[captura] lendo por {args.pos_dump:.0f}s...")
        fim = time.time() + args.pos_dump
        with open(saida, "w", encoding="utf-8") as f:
            while time.time() < fim:
                dados = ser.read(4096)
                if dados:
                    texto = dados.decode("utf-8", errors="replace")
                    f.write(texto)
                    f.flush()

    print(f"[captura] concluído: {saida}")


if __name__ == "__main__":
    main()
