#!/usr/bin/env python3
"""Compila e executa os testes nativos do firmware (sem hardware).

Inclui os headers REAIS do firmware; o stub tests/native/Arduino.h supre a
unica dependencia de Arduino. Assim o teste exercita o codigo que vai para
o robo, nao uma copia da formula.

Uso:
    python tests/native/rodar.py

O compilador e escolhido nesta ordem:
    1. variavel de ambiente CXX (ex.: CXX="clang++")
    2. pacote python ziglang (pip install ziglang) -> "python -m ziglang c++"
    3. g++ ou clang++ no PATH
"""
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile

RAIZ = pathlib.Path(__file__).resolve().parents[2]
NATIVO = pathlib.Path(__file__).resolve().parent
FW = RAIZ / "firmware_robo2wd_v4"
TESTES = ["test_cinematica.cpp", "test_pivo.cpp"]


def encontrar_compilador():
    if os.environ.get("CXX"):
        return os.environ["CXX"].split()
    try:
        import ziglang  # noqa: F401
        return [sys.executable, "-m", "ziglang", "c++"]
    except ImportError:
        pass
    for candidato in ("g++", "clang++"):
        if shutil.which(candidato):
            return [candidato]
    return None


def main():
    cc = encontrar_compilador()
    if not cc:
        print("erro: nenhum compilador C++ encontrado.")
        print("instale g++, clang++ ou rode: pip install ziglang")
        return 1

    falhas = 0
    with tempfile.TemporaryDirectory() as tmp:
        for teste in TESTES:
            exe = pathlib.Path(tmp) / (pathlib.Path(teste).stem + ".exe")
            cmd = cc + [
                "-O1", "-std=c++17", "-Wall",
                "-I", str(NATIVO),
                "-I", str(FW),
                str(NATIVO / teste),
                "-o", str(exe),
            ]
            print(f"\n[compilando] {teste}")
            r = subprocess.run(cmd)
            if r.returncode != 0:
                print("FALHA na compilacao")
                falhas += 1
                continue
            r = subprocess.run([str(exe)])
            if r.returncode != 0:
                falhas += 1

    print()
    if falhas:
        print(f"FALHA: {falhas} teste(s) com problema")
        return 1
    print("Todos os testes nativos passaram")
    return 0


if __name__ == "__main__":
    sys.exit(main())
