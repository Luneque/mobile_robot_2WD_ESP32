#!/usr/bin/env python3
"""Captura o dump do gravador de telemetria via WebSocket (robô sem cabo).

Uso:
    python capturar_dump_wifi.py [--host robo.local] [--port 81] \
        [--start] [--espera 45] [--saida dump.txt]

Fluxo:
    1. Conecta no WebSocket do robô (reconecta se cair no boot).
    2. Envia "start" para iniciar a trajetória.
    3. Aguarda o tempo de execução, recebendo telemetria em tempo real.
    4. Envia "dump" e lê as linhas "REC,..." do buffer em RAM.
    5. Grava tudo em texto para análise posterior.

Robustez:
    - O ESP32 reinicia no boot e o WebSocket cai; este script reconecta.
    - O ping do WebSocket pode expirar durante o armamento do ESC; o
      intervalo de ping é maior e há reconexão automática.
"""

import argparse
import asyncio
import pathlib
import sys


async def conectar(host: str, port: int, timeout: float = 30.0):
    """Conecta no WebSocket, reconectando até conseguir."""
    import websockets

    url = f"ws://{host}:{port}/"
    fim = asyncio.get_event_loop().time() + timeout
    tentativa = 0
    while asyncio.get_event_loop().time() < fim:
        tentativa += 1
        try:
            ws = await websockets.connect(
                url, ping_interval=20, ping_timeout=20, close_timeout=2
            )
            print(f"[wifi] conectado (tentativa {tentativa})")
            return ws
        except (OSError, websockets.exceptions.WebSocketException) as e:
            print(f"[wifi] tentativa {tentativa} falhou: {type(e).__name__}")
            await asyncio.sleep(2)
    raise RuntimeError(f"não foi possível conectar a {url}")


async def main_async(args):
    import websockets

    ws = await conectar(args.host, args.port)

    if args.start:
        print("[wifi] enviando 'start'...")
        await ws.send("start")

    print(f"[wifi] aguardando {args.espera:.0f}s de execução...")
    fim_exec = asyncio.get_event_loop().time() + args.espera
    while asyncio.get_event_loop().time() < fim_exec:
        try:
            await asyncio.wait_for(
                ws.recv(), timeout=max(0.5, fim_exec - asyncio.get_event_loop().time())
            )
        except asyncio.TimeoutError:
            break
        except websockets.exceptions.ConnectionClosedError:
            print("[wifi] conexão caiu; reconectando...")
            ws = await conectar(args.host, args.port, timeout=20.0)

    print("[wifi] enviando 'dump'...")
    try:
        await ws.send("dump")
    except websockets.exceptions.ConnectionClosedError:
        print("[wifi] conexão caiu antes do dump; reconectando...")
        ws = await conectar(args.host, args.port, timeout=20.0)
        await ws.send("dump")

    print(f"[wifi] lendo dump por até {args.pos_dump:.0f}s...")
    fim_leitura = asyncio.get_event_loop().time() + args.pos_dump
    linhas_rec = []
    mensagens = []

    while asyncio.get_event_loop().time() < fim_leitura:
        try:
            msg = await asyncio.wait_for(
                ws.recv(), timeout=max(0.5, fim_leitura - asyncio.get_event_loop().time())
            )
        except asyncio.TimeoutError:
            break
        except websockets.exceptions.ConnectionClosedError:
            print("[wifi] conexão caiu durante o dump; reconectando...")
            try:
                ws = await conectar(args.host, args.port, timeout=10.0)
                await ws.send("dump")   # reenvia para continuar
            except Exception:
                break
            continue

        if isinstance(msg, bytes):
            continue
        for linha in msg.splitlines():
            if linha.startswith("REC,"):
                linhas_rec.append(linha)
            elif "[REC]" in linha:
                mensagens.append(linha)
        if any("Dump concluido" in m for m in mensagens):
            break

    saida = pathlib.Path(args.saida)
    with open(saida, "w", encoding="utf-8") as f:
        for m in mensagens:
            f.write(m + "\n")
        f.write("timestamp_ms,estado,lado,yaw_deg,rpm_esq,rpm_dir,ticks_esq,ticks_dir,ticks_alvo,pwm_esq_us,pwm_dir_us,rpm_alvo\n")
        for l in linhas_rec:
            f.write(l + "\n")

    print(f"[wifi] {len(linhas_rec)} amostras REC capturadas")
    print(f"[wifi] arquivo: {saida}")


def main():
    p = argparse.ArgumentParser(description="Captura dump do gravador via Wi-Fi.")
    p.add_argument("--host", default="robo.local")
    p.add_argument("--port", type=int, default=81)
    p.add_argument("--saida", default="dump_wifi.txt")
    p.add_argument("--start", action="store_true")
    p.add_argument("--espera", type=float, default=45.0)
    p.add_argument("--pos-dump", type=float, default=25.0)
    args = p.parse_args()

    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("\n[wifi] interrompido")
        sys.exit(1)
    except RuntimeError as e:
        print(f"[wifi] erro: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()

