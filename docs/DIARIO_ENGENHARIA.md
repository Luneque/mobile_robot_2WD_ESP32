# Diário de Engenharia — Projeto Robô Diferencial 2WD com ESP32

---

## Registro Cronológico de Desenvolvimento e Evolução

### 1. Etapa 1: Versão 1 — Abordagem Inicial por Software (Legado: `legacy/firmware_robo2wd/`)
* **Implementação:** PWM gerado pelo periférico LEDC e leitura de encoders via interrupções por software na CPU (`attachInterrupt` em bordas de GPIO).
* **Limitações Observadas:**
  * Alta taxa de transições nos encoders Hall causava sobrecarga contínua de interrupções na CPU, gerando atrasos e perda de bordas de quadratura.
  * O ESC sofria com instabilidade de timing e a odometria acumulava erros acentuados.

---

### 2. Etapa 2: Versão 2 — Migração para Periféricos de Hardware (Legado: `legacy/firmware_robo2wd_v2/`)
* **Implementação:**
  * Substituição do LEDC pelo **MCPWM** do ESP-IDF (timer dedicado de 1 MHz, período de 20 ms / 50 Hz com resolução de $1\,\mu\text{s}$).
  * Substituição das ISRs pelo módulo **PCNT (Pulse Counter)** com decodificação 4x direta em portas lógicas e filtro anti-ruído digital de 1000 ns.
  * Inclusão de compensador de sincronismo cruzado (*cross-coupling*) em linha reta.
* **Calibração Real Homologada:**
  * O teste manual no utilitário de bancada `diag_pcnt` comprovou a redução real de **1:90** e a contagem real de **4.320 pulsos por volta completa** ($12 \text{ PPR} \times 90 \times 4\text{x}$).
* **Limitações Remanescentes:**
  * O código rodava em loops bloqueantes `while(true)` dentro do `setup()`.
  * Ausência de perfil de aceleração (degrau abrupto de velocidade que causava patinagem em velocidades mais altas).
  * Curvas de pivô sem proteção contra travamento de roda (*stall*).

---

### 3. Etapa 3: Versão 3 — Arquitetura Oficial Robusta e Não-Bloqueante (`firmware_robo2wd_v3/`)
Para elevar o projeto ao nível de maturidade industrial, a Versão 3 foi implementada com os seguintes avanços:
1. **Validação Estrita de Hardware:** Se qualquer unidade PCNT ou gerador MCPWM falhar no boot, o robô aborta a operação de forma segura em `ESTADO_FALHA_HARDWARE`, evitando *Kernel Panics*.
2. **Máquina de Estados Não-Bloqueante no `loop()`:** A trajetória do quadrado foi migrada para uma máquina de estados finita (FSM) orientada a tempo (`millis()`), com aguardo real de comando `'s'` na serial ou timeout automático de 5 segundos.
3. **Task Watchdog Timer (TWDT):** Reconfigurado via `esp_task_wdt_reconfigure()` para 3 segundos e subscrito apenas após o armamento do ESC, protegendo o robô contra *deadlocks* sem risco de reinicialização prematura no boot.
4. **Proteção Contra Roda Travada (*Stall*):** Checagem ativa de desbalanceamento ($|\,|rpm_1| - |rpm_2|\,| > 45\%$) agora presente tanto em retas quanto em giros pivô, desligando os motores imediatamente em `ESTADO_EMERGENCIA`.
5. **Perfil Trapezoidal de Velocidade e Otimização para 80 RPM:**
   * Aceleração calibrada em 50 mm ($\approx 600\text{ mm/s}^2$) e desaceleração em 60 mm.
   * O robô atinge velocidade de cruzeiro de **80 RPM ($\approx 293\text{ mm/s}$)** mantida durante 78% do segmento, reduzindo o tempo de reta para **$\approx 1,88\text{ s}$** com parada milimétrica sem deslizamento.
   * Giros de 90° otimizados para **50 RPM** ($\approx 0,52\text{ s}$).

---

### 4. Resumo da Estrutura Atual do Repositório
* **`firmware_robo2wd_v3/`:** **Versão Oficial Atual e Ativa.**
* **`legacy/`:** Contém as implementações históricas `firmware_robo2wd/` (V1) e `firmware_robo2wd_v2/` (V2).
* **`diag_pcnt/`:** Ferramenta oficial de diagnóstico e validação dos encoders via PCNT em hardware.
