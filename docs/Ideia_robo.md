# Robô 2WD

> **[HISTÓRICO — Ideação original, pré-V1]**
> Este documento registra a concepção inicial do projeto e o levantamento de materiais.
> O mapeamento de pinos abaixo é o da Versão 1 e **não corresponde ao firmware oficial V3**
> (encoder esquerdo em GPIO 16/17, I2C da IMU BMI160 em GPIO 21/22). Consulte
> `README.md`, `docs/FSD.md` e `docs/MANUAL_TECNICO.md` para o estado atual.

## Materiais
 - Chassi Circular (foto2.png)
 - 2 Rodas
 - 2 Motores DC com Encoder (foto4.png)
 - ESP32 WROOM32 com Placa de Expansão (foto1.png)
 - ESC para Motor Escovado 2 Canais (foto3.png)
---

## Informações Extras

- Diâmetro do Chassi: 14cm
- Distância entre as rodas: 12cm
- Diâmetro das rodas: 7cm
- Tensão da bateria/esc: 7.4v (2S)
- Redução do motor: 1:90
---

## Conexões
- ESC CH1: GPIO18
- ESC CH2: GPIO19
- Encoder 1 Canal A: GPIO23
- Encoder 1 Canal B: GPIO22
- Encoder 2 Canal A: GPIO33
- Encoder 2 Canal B: GPIO32

## Ideia Inicial
Programar a placa ESP32 para mover o robô em uma trajetória de teste simples abaixo da velocidade máxima dos motores. A leitura dos encoders deve permitir uma movimentação mais precisa do robô. Mais funcionalidades poderão ser adicionadas futuramente.