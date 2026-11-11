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

### 4. Etapa 4: Integração da IMU Bosch BMI160 (10/11/2026 — commit `032dc0e`)
* **Motivação:** Movimentos em linha reta apresentavam desvio acumulativo que a sincronização por encoder não conseguia compensar completamente. As curvas de pivô dependiam exclusivamente de tempo morto estimado.
* **Remapeamento de Hardware:**
  * Encoder esquerdo migrado de GPIO 23/22 → **GPIO 16/17**, liberando o barramento I2C (GPIO 21/22) para a IMU.
* **Driver `imu_bmi160.h` criado:**
  * Comunicação I2C nativa a 400 kHz (sem biblioteca externa).
  * Inicialização do giroscópio (modo normal, ODR 100 Hz, faixa ±250°/s) e do acelerômetro (modo normal, ODR 100 Hz, faixa ±2g).
  * Calibração estática de *bias* do giroscópio (`calibrateGyro()`) com média de 150 amostras em repouso.
  * Integração de ângulo de guinada (Yaw) via `update(float dt_sec)` com `dt` em segundos.
  * Funções `resetYaw()` e `getYawAngleDeg()` para uso pela FSM.
* **Integração no firmware V3:**
  * Correção de rumo em retas via `IMU_KP_YAW_RETA` (ganho proporcional sobre o erro de Yaw).
  * Critério de parada de curva substituído de temporizador por limiar de Yaw integrado.

---

### 5. Etapa 5: Calibração de Polaridade do Eixo Z (10/11/2026 — commit `969d368`)
* **Problema:** Sem calibração de polaridade, a correção de rumo operava na direção errada — amplificando o desvio em vez de corrigi-lo.
* **Procedimento:** Utilitário `calib_imu_polaridade/calib_imu_polaridade.ino` executado com giro manual no sentido horário.
  * Resultado: Eixo Z bruto reportava valor **NEGATIVO** para giro horário (convenção oposta ao esperado).
* **Correção aplicada:** Leitura invertida `-readRawGyroZ()` no driver (tanto em `calibrateGyro()` quanto em `update()`), estabelecendo a convenção: **giro horário = Yaw positivo**.
* **Ajuste de ganho:** `IMU_KP_YAW_RETA` reduzido de `1.2f` → `0.6f` para evitar oscilação no controle de rumo (sobreposição com sincronismo de encoders).

---

### 6. Etapa 6: Correções Lógicas de `yawError` e `IMU_KP_PIVO` (11/11/2026 — commit `bc36eef`)
* **Problema raiz identificado:** A instrução `float yawAtual = abs(imu.getYawAngleDeg())` descartava o sinal do ângulo, provocando:
  1. **Retas:** A correção de rumo era aplicada na direção **errada** (sempre "corrigia" como se o robô estivesse desviando para a mesma direção, independentemente do desvio real).
  2. **Curvas:** O progresso do pivô era subestimado (ângulo absoluto ≠ ângulo relativo ao início da curva), fazendo o robô girar além ou aquém dos 90°.
* **Solução — erro relativo com sinal preservado:**
  * `imu.resetYaw()` é chamado no início de cada segmento (reta ou curva) e zera o ângulo de yaw.
  * O erro relativo é o próprio yaw medido: `float yawError = yawAtual;` (preserva o sinal, pois o reset garante a referência zero).
  * A variável intermediária `g_yawRef` foi removida por ser redundante (o `resetYaw()` já zera o ângulo).
* **`IMU_KP_PIVO` (removido na Etapa 7):** Tinha sido aplicado como `sync_corr += yawError * IMU_KP_PIVO;`, mas o termo era redundante e causava translação no pivô — ver Etapa 7.
* **Critério de parada de curva atualizado:** `yawAbs >= (g_anguloTotalGraus - 2.0f)` — tolerância de 2° previne loop infinito por latência de leitura.
* **Acelerômetro:** Configurado pelo driver (ODR 100 Hz, ±2 g), mas não lido pelo firmware — apenas o utilitário `diag_i2c_bmi160` faz a leitura. Dados disponíveis para fusão sensorial futura.
* **Compilação:** ✅ 317.328 bytes (24% flash), 23.756 bytes RAM. Gravado em COM9.

---

### 7. Etapa 7: Remoção da correção de yaw no pivô (11/11/2026)
* **Problema:** O termo `sync_corr += yawError * IMU_KP_PIVO` era aplicado em modo comum (o mesmo sinal nos dois motores). Em um pivô, isso não altera a taxa de rotação (a diferença entre as rodas permanece) e causa uma translação proporcional ao yaw — o robô derivava durante a curva.
* **Correção:** Termo removido e `IMU_KP_PIVO` eliminado de `config.h`. A IMU continua atuando no pivô pelo critério de parada (`|yawError| ≥ (ânguloAlvo − 2°)`) e pelo mapeamento do perfil trapezoidal.

---

### 8. Resumo da Estrutura Atual do Repositório
* **`firmware_robo2wd_v3/`:** **Versão Oficial Atual e Ativa** (com IMU BMI160 integrada e funcional).
* **`legacy/`:** Contém as implementações históricas `firmware_robo2wd/` (V1) e `firmware_robo2wd_v2/` (V2).
* **`diag_pcnt/`:** Ferramenta oficial de diagnóstico e validação dos encoders via PCNT em hardware.
* **`diag_i2c_bmi160/`:** Utilitário de diagnóstico do barramento I2C e leitura da IMU BMI160 (Chip ID, giroscópio e acelerômetro).
* **`calib_imu_polaridade/`:** Ferramenta de validação da polaridade do eixo Z da IMU (confirma convenção horário=positivo).
