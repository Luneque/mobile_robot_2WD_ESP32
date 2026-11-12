#!/usr/bin/env python3
"""
robo_ws_logger.py — Monitor e logger de telemetria via WebSocket
=================================================================
Conecta ao WebSocket do robô 2WD (ESP32) e exibe telemetria em tempo real
no terminal, além de salvar automaticamente em CSV.

Uso:
    python robo_ws_logger.py [--host robo.local] [--port 81]

Exemplos:
    python robo_ws_logger.py
    python robo_ws_logger.py --host 192.168.1.42
    python robo_ws_logger.py --host robo.local --port 81

Dependências:
    pip install -r requirements.txt
    (websockets>=12.0, rich>=13.0)

Campos JSON recebidos do robô:
    ts  — timestamp em ms (millis() desde o boot)
    st  — estado: "AGUARDO", "RETA", "PIVO", "PAUSA", "CONCLUIDO", "EMERGENCIA", "FALHA"
    sd  — lado atual (1–4)
    of  — total de lados (sempre 4)
    yaw — ângulo yaw em graus
    r1  — RPM motor esquerdo
    r2  — RPM motor direito
    t1  — ticks encoder esquerdo
    t2  — ticks encoder direito
    tg  — ticks alvo do segmento
    w1  — PWM motor esquerdo em µs
    w2  — PWM motor direito em µs
    rt  — RPM alvo (perfil trapezoidal)
"""

import asyncio
import argparse
import json
import csv
import datetime
import pathlib
import sys
import signal

# -------------------------------------------------------------------------
# Constantes
# -------------------------------------------------------------------------
VERSAO = "1.0.0"
PASTA_LOGS = pathlib.Path(__file__).parent / "logs"

# Mapeamento de estado para cor no rich
CORES_ESTADO = {
    "AGUARDO":    "dim white",
    "RETA":       "bold blue",
    "PIVO":       "bold yellow",
    "PAUSA":      "bold cyan",
    "CONCLUIDO":  "bold green",
    "EMERGENCIA": "bold red",
    "FALHA":      "bold red",
}


def verificar_dependencias():
    """Verifica se websockets e rich estão instalados e instrui o usuário."""
    faltando = []
    try:
        import websockets  # noqa: F401
    except ImportError:
        faltando.append("websockets>=12.0")
    try:
        import rich  # noqa: F401
    except ImportError:
        faltando.append("rich>=13.0")

    if faltando:
        print("\n[ERRO] Dependências ausentes para executar o robo_ws_logger:")
        for dep in faltando:
            print(f"  - {dep}")
        print("\nPara instalar todas as dependências, execute:")
        print("  pip install -r requirements.txt")
        print("\nOu instale individualmente com:")
        print(f"  pip install {' '.join(faltando)}\n")
        sys.exit(1)


# -------------------------------------------------------------------------
# Classe principal do logger
# -------------------------------------------------------------------------
class RoboLogger:
    """
    Gerencia a conexão WebSocket, exibição no terminal e gravação em CSV.
    """

    def __init__(self, host: str, porta: int):
        self.host = host
        self.porta = porta
        self.url = f"ws://{host}:{porta}/"

        # Contadores de sessão
        self.total_pacotes = 0
        self.erros_json = 0
        self.inicio_sessao = datetime.datetime.now()

        # Último pacote recebido (para exibição)
        self.ultimo_dado: dict = {}

        # Controle de execução
        self._rodando = True

        # CSV
        self._arquivo_csv = None
        self._escritor_csv = None
        self._caminho_csv: pathlib.Path | None = None

        # Imports dinâmicos do rich (após verificação)
        from rich.console import Console
        self.console = Console()

    def _abrir_csv(self):
        """Cria o arquivo CSV com timestamp na pasta de logs."""
        PASTA_LOGS.mkdir(parents=True, exist_ok=True)
        timestamp_str = self.inicio_sessao.strftime("%Y%m%d_%H%M%S")
        self._caminho_csv = PASTA_LOGS / f"sessao_{timestamp_str}.csv"

        self._arquivo_csv = open(self._caminho_csv, "w", newline="", encoding="utf-8")
        self._escritor_csv = csv.writer(self._arquivo_csv)

        # Cabeçalho do CSV
        self._escritor_csv.writerow([
            "timestamp_ms",
            "estado",
            "lado",
            "yaw_deg",
            "rpm_esq",
            "rpm_dir",
            "ticks_esq",
            "ticks_dir",
            "ticks_alvo",
            "pwm_esq_us",
            "pwm_dir_us",
            "rpm_alvo",
        ])

    def _fechar_csv(self):
        """Fecha o arquivo CSV com segurança."""
        if self._arquivo_csv and not self._arquivo_csv.closed:
            self._arquivo_csv.flush()
            self._arquivo_csv.close()

    def _gravar_linha_csv(self, dados: dict):
        """Grava uma linha de telemetria no CSV."""
        if self._escritor_csv is None:
            return
        self._escritor_csv.writerow([
            dados.get("ts", ""),
            dados.get("st", ""),
            dados.get("sd", ""),
            dados.get("yaw", ""),
            dados.get("r1", ""),
            dados.get("r2", ""),
            dados.get("t1", ""),
            dados.get("t2", ""),
            dados.get("tg", ""),
            dados.get("w1", ""),
            dados.get("w2", ""),
            dados.get("rt", ""),
        ])
        # Flush periódico para não perder dados em caso de crash
        if self.total_pacotes % 10 == 0:
            self._arquivo_csv.flush()

    def _construir_tabela(self):
        """Constrói a tabela Rich com os dados de telemetria mais recentes."""
        from rich.table import Table
        from rich.text import Text
        from rich import box

        d = self.ultimo_dado
        estado = d.get("st", "---")
        cor_estado = CORES_ESTADO.get(estado, "white")

        # Cabeçalho da tabela
        tabela = Table(
            title=f"[bold]Robô 2WD — Telemetria em Tempo Real[/bold]  "
                  f"[dim]({self.url})[/dim]",
            box=box.ROUNDED,
            show_header=True,
            header_style="bold magenta",
            expand=True,
        )

        tabela.add_column("Campo", style="bold", width=18)
        tabela.add_column("Esquerdo / Valor", justify="right", width=22)
        tabela.add_column("Direito", justify="right", width=22)

        # Estado e segmento
        tabela.add_row(
            "Estado",
            Text(estado, style=cor_estado),
            f"Lado {d.get('sd', '-')} / {d.get('of', '-')}",
        )

        # Timestamp
        ts_ms = d.get("ts", 0)
        ts_str = f"{ts_ms / 1000:.1f}s" if ts_ms else "---"
        tabela.add_row("Tempo boot", ts_str, "")

        # Yaw
        yaw = d.get("yaw", None)
        yaw_str = f"{yaw:+.2f}°" if yaw is not None else "---"
        tabela.add_row("Yaw", yaw_str, "")

        # RPM
        r1 = d.get("r1", None)
        r2 = d.get("r2", None)
        rt = d.get("rt", None)
        tabela.add_row(
            "RPM (alvo)",
            f"{r1:.1f}" if r1 is not None else "---",
            f"{r2:.1f}" if r2 is not None else "---",
        )
        tabela.add_row(
            "RPM alvo",
            f"{rt:.1f}" if rt is not None else "---",
            "",
        )

        # Ticks
        t1 = d.get("t1", None)
        t2 = d.get("t2", None)
        tg = d.get("tg", None)
        tabela.add_row(
            "Ticks (alvo)",
            f"{t1}" if t1 is not None else "---",
            f"{t2}" if t2 is not None else "---",
        )
        tabela.add_row(
            "Ticks alvo",
            f"{tg}" if tg is not None else "---",
            "",
        )

        # PWM
        w1 = d.get("w1", None)
        w2 = d.get("w2", None)
        tabela.add_row(
            "PWM (µs)",
            f"{w1} µs" if w1 is not None else "---",
            f"{w2} µs" if w2 is not None else "---",
        )

        # Rodapé com estatísticas da sessão
        duracao = datetime.datetime.now() - self.inicio_sessao
        tabela.caption = (
            f"Pacotes recebidos: [bold]{self.total_pacotes}[/bold]  |  "
            f"Erros JSON: [red]{self.erros_json}[/red]  |  "
            f"Duração: {str(duracao).split('.')[0]}  |  "
            f"CSV: [dim]{self._caminho_csv.name if self._caminho_csv else 'N/A'}[/dim]"
        )

        return tabela

    async def conectar_e_logar(self):
        """
        Coroutine principal: conecta ao WebSocket e processa mensagens.
        Reconecta automaticamente em caso de desconexão.
        """
        import websockets
        from rich.live import Live

        self.console.print(
            f"\n[bold cyan]Robô Monitor v{VERSAO}[/bold cyan]\n"
            f"Conectando a [bold]{self.url}[/bold]...\n"
            f"Pressione [bold]Ctrl+C[/bold] para encerrar.\n"
        )

        # Abre o arquivo CSV
        self._abrir_csv()
        self.console.print(f"[dim]CSV: {self._caminho_csv}[/dim]\n")

        tentativas = 0

        while self._rodando:
            try:
                tentativas += 1
                async with websockets.connect(
                    self.url,
                    ping_interval=10,
                    ping_timeout=5,
                ) as ws:
                    tentativas = 0  # Reset ao conectar com sucesso
                    self.console.print("[green]✓ Conectado![/green]")

                    # Exibição em tempo real com Rich Live (atualiza mesma linha sem scrollar)
                    with Live(
                        self._construir_tabela(),
                        console=self.console,
                        refresh_per_second=5,
                        screen=False,
                    ) as live:
                        async for mensagem in ws:
                            if not self._rodando:
                                break
                            self._processar_mensagem(mensagem)
                            live.update(self._construir_tabela())

            except websockets.exceptions.ConnectionClosedError as e:
                self.console.print(f"[yellow]Conexão encerrada: {e}. Reconectando...[/yellow]")
                await asyncio.sleep(2)
            except ConnectionRefusedError:
                self.console.print(
                    f"[red]Conexão recusada em {self.url}.[/red] "
                    f"Tentativa {tentativas}. Aguardando 5s..."
                )
                await asyncio.sleep(5)
            except OSError as e:
                self.console.print(
                    f"[red]Erro de rede: {e}[/red] "
                    f"Tentativa {tentativas}. Aguardando 5s..."
                )
                await asyncio.sleep(5)
            except asyncio.CancelledError:
                break

    def _processar_mensagem(self, mensagem: str):
        """
        Processa uma mensagem JSON recebida via WebSocket.
        Atualiza os dados internos e grava no CSV.
        """
        try:
            dados = json.loads(mensagem)
            self.ultimo_dado = dados
            self.total_pacotes += 1
            self._gravar_linha_csv(dados)
        except json.JSONDecodeError:
            self.erros_json += 1

    def imprimir_resumo(self):
        """Imprime o resumo da sessão ao encerrar."""
        from rich.panel import Panel

        duracao = datetime.datetime.now() - self.inicio_sessao

        self.console.print("\n")
        self.console.print(Panel(
            f"[bold]Resumo da Sessão[/bold]\n\n"
            f"Duração:          [cyan]{str(duracao).split('.')[0]}[/cyan]\n"
            f"Pacotes recebidos:[cyan]{self.total_pacotes:>8}[/cyan]\n"
            f"Erros de JSON:    [red]{self.erros_json:>8}[/red]\n"
            f"Arquivo CSV:      [dim]{self._caminho_csv}[/dim]\n",
            title="[bold green]Sessão encerrada[/bold green]",
            border_style="green",
        ))

    def parar(self):
        """Sinaliza o encerramento gracioso."""
        self._rodando = False


# -------------------------------------------------------------------------
# Ponto de entrada
# -------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Monitor e logger de telemetria do robô 2WD via WebSocket.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--host",
        default="robo.local",
        help="Endereço IP ou hostname do robô (padrão: robo.local)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=81,
        help="Porta WebSocket do robô (padrão: 81)",
    )
    args = parser.parse_args()

    # Verifica dependências antes de iniciar
    verificar_dependencias()

    logger = RoboLogger(host=args.host, porta=args.port)

    # Configura tratamento de Ctrl+C
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    def _encerrar(signum, frame):
        logger.parar()
        # Cancela todas as tasks pendentes
        for task in asyncio.all_tasks(loop):
            task.cancel()

    signal.signal(signal.SIGINT, _encerrar)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _encerrar)

    try:
        loop.run_until_complete(logger.conectar_e_logar())
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        logger._fechar_csv()
        logger.imprimir_resumo()
        loop.close()


if __name__ == "__main__":
    main()
