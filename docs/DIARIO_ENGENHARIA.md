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

### 8. Etapa 8: Versão 4 — Conectividade Sem Fio, Dashboard Web e Otimização Dinâmica (12/11/2026 — commits `e6c03f0`, `9e2682d`)
* **Motivação:** A dependência do cabo serial USB limitava testes com o robô em solo livre e impedia análise gráfica contínua de variáveis de controle (RPM, Yaw, PWM e Ticks).
* **Desenvolvimento da Versão 4 (`firmware_robo2wd_v4/`):**
  * **Driver `wifi_telemetry.h`:** Integração do rádio Wi-Fi (modo Station na rede local) com servidor HTTP (porta 80), mDNS (`http://robo.local/`) e servidor WebSocket assíncrono (porta 81).
  * **Dashboard Web Embarcado:** Interface HTML/JS com dark theme em memória flash (PROGMEM), renderizando gráficos em tempo real com Chart.js (janela de 15s), botões de comando remoto (**START**) e exportação instantânea de dados em CSV.
  * **Partida Remota Segura:** O robô pode ser disparado remotamente pelo navegador via WebSocket sem cabo de dados conectado.
  * **Ponte ESP-NOW (`gateway_espnow/`):** Firmware dedicado para o microcontrolador ESP32-S3 Super Mini para permitir telemetria transparente via rádio 2.4 GHz em campo aberto sem roteador Wi-Fi.
  * **Ferramenta PC (`robo_monitor/`):** Script `robo_ws_logger.py` para monitoramento no terminal via biblioteca `rich` e salvamento automático de logs em CSV.
* **Calibração Dinâmica Pós-Telemetria em Solo:**
  * **Problema diagnosticado no CSV 1:** O robô ficava ~3,0 s parado no início de cada reta vencendo atrito estático com o feedforward original de 50 µs (`ESC_DEADBAND_US`).
  * **Ajuste aplicado:** Feedforward de zona morta elevado para `110.0 µs` (`ESC_DEADBAND_US`), rompendo o atrito estático no ciclo zero (t=0) e reduzindo o tempo de reta de ~7,6 s para ~3,8 s.
  * **Controle Angular Reforçado:** Ganho proporcional de Yaw em retas aumentado de `0.6` para `1.2` (`IMU_KP_YAW_RETA`) e sincronismo de encoders aumentado para `0.08` (`K_SYNC`), reduzindo o desvio angular para menos de 2°.
* **Resultados Homologados:** O robô executou o quadrado completo de 4 lados e 4 curvas com arrancada imediata, fechamento perfeito do percurso e telemetria 100% gravada sem perdas.

---

### 9. Etapa 9: Refinamento Físico de Polaridade, Fechamento de Pivô a 90° e Cruzeiro (12/11/2026 — commits `4b24c2e`, `77339d5`)
* **Correção Crítica da Polaridade de Yaw (`imu_bmi160.h`):**
  * Verificou-se que a inversão manual `-readRawGyroZ()` gerava sinal negativo para giro à direita (horário), gerando realimentação positiva em retas.
  * Restaurada a leitura direta `readRawGyroZ()`, restabelecendo a convenção: **giro horário = Yaw positivo (+)** e realimentação negativa estável.
* **Calibração da Tolerância de Pivô para 90,0° Real:**
  * O critério de corte foi ajustado de 88,0° para **89,0°** (`yawAbs >= (g_anguloTotalGraus - 1.0f)`).
  * Com a desaceleração inercial durante a pausa de 800 ms, o robô passou a parar cravado em **+89,1° a +90,2°** (erro médio < 0,55° por curva).
* **Calibração Realista do Cruzeiro (55 RPM):**
  * Redefinido `VELOCIDADE_CRUZEIRO_RPM` de 80 RPM para **55 RPM (~201 mm/s)**, alinhado à curva de torque/tensão da bateria 2S e caixa 1:90 em retas de 500 mm.
  * O erro de velocidade em regime foi zerado e a diferença residual entre as rodas caiu para menos de 90 ticks (< 4 mm).
* **Parada Remota via WebSocket:** Implementado tratamento para comandos de parada imediata (`stop` / `pause`).
* **Padronização JSON Numérico:** Os campos `yaw`, `r1`, `r2` e `rt` foram convertidos para tipos numéricos puros no payload JSON do WebSocket.

---

### 10. Etapa 10: Recalibração Cinemática (66 mm), Blindagem de Pivô e Mapa 2D de Trajetória (13/11/2026)
* **Constatação Física:** O par de rodas instalado no chassi possui bitola/diâmetro real de **66,0 mm** (e não 70,0 mm da especificação original).
* **Impacto Odométrico e Cinemático:**
  * Circunferência real recalculada: $C = \pi \times 66,0 = \mathbf{207,345\text{ mm}}$ (antes era $219,911\text{ mm}$).
  * Resolução linear recalculada: $\text{MM\_POR\_PULSO} = \frac{207,345}{4320} \approx \mathbf{0,047997\text{ mm/pulso}}$ (antes era $0,050905\text{ mm/pulso}$).
  * Meta de ticks para reta de 500 mm: $\frac{500}{0,047997} \approx \mathbf{10.417\text{ pulsos}}$ (antes parava em 9.822 pulsos, percorrendo apenas ~471 mm e ficando curto).
  * Meta de ticks de arco por roda no pivô de 90°: $\frac{\pi \times 120 / 4}{0,047997} \approx \mathbf{1.964\text{ pulsos}}$ (antes calculava 1.851 pulsos).
* **Mitigação do Overshoot e Guarda de Movimento no Pivô:**
  * Redução da velocidade alvo do pivô de 50 RPM para **35 RPM** (`VELOCIDADE_GIRO_RPM = 35.0f`), diminuindo a energia cinética e a inércia rotacional no corte de potência a 89,5°.
  * **Guarda Mínima de 200 ticks (`MIN_TICKS_PIVO`):** Elimina condição de corrida na qual o giroscópio ainda carregava yaw residual da pausa anterior e disparava término prematuro a 0° de rotação real. Homologação subsequente registrou **12 pivôs consecutivos sem falhas (média de ~89,9°)**.
* **Compensação de Drift M2 > M1:**
  * Ajuste de `K_SYNC` de 0.08 para **0.15** (+87%), reduzindo a assimetria sistemática do motor direito de ~102 ticks para 86 ticks e estabilizando o desvio de rumo em retas para **0,59° médio**.
* **Visualizador 2D de Trajetória no Dashboard (`wifi_telemetry.h`):**
  * Implementação de elemento Canvas HTML5 de alta fidelidade desenhando o percurso ideal do quadrado (500×500 mm) em linha tracejada azul.
  * Reconstrução contínua da trajetória (*dead reckoning*) plotada em linha verde com base no odômetro calibrado e yaw relativo.
  * Indicador de pose do robô (ponto de localização + vetor direcional) com exibição numérica das coordenadas $(X, Y)$ em mm e ângulo $\theta$ em graus, além de botão para reset da visualização.

---

### 11. Etapa 11: Correção de Rotação por Delta Real ($\Delta\text{yaw}$) e Odometria Cumulativa (13/11/2026)
* **Diagnóstico de Desvio Cumulativo:**
  * O robô acumulava desvio que se agravava a cada volta porque o critério de corte de pivô avaliava o valor absoluto do yaw em vez do delta de rotação efetivo ($\Delta\text{yaw} = \text{yawAtual} - \text{yawStartSeg}$). Qualquer resíduo de yaw (+1° ou +2°) antecipava o corte da curva, fazendo o robô girar apenas ~86° a 88° reais por canto.
  * No painel web, o mapa de trajetória forçava os ângulos nominais de 90° em cada lado (`baseDeg`), mascarando o erro real e desenhando um quadrado ilusório na tela.
* **Correções Aplicadas:**
  1. **Controle por $\Delta\text{yaw}$ no Firmware (`firmware_robo2wd_v4.ino`):** Captura `g_yawStartSeg` no início exato de cada pivô e avalia `fabsf(yawAtual - g_yawStartSeg) >= 89.5°`, garantindo o giro completo e exato de 90° físicos em todas as curvas.
  2. **Odometria Realista e Cumulativa no Dashboard (`wifi_telemetry.h`):** O rastro do mapa agora utiliza orientação global cumulativa contínua, sem forçar múltiplos fixos de 90°. Se houver qualquer imperfeição na pista, o mapa refletirá com exatidão a inclinação e a trajetória real do robô.

---

### 12. Etapa 12: Feedforward por Velocidade, Gravador em RAM e Calibração no Solo (16/11/2026)

* **Diagnóstico da Velocidade (curva PWM × RPM):**
  * O motor não atingia os 55 RPM de cruzeiro, e o feedforward era um valor fixo de 110 µs — eficaz para romper o atrito estático, mas sem relação com a velocidade alvo.
  * O utilitário `diag_motor_curve` levantou a curva real (robô suspenso). Duas medições divergiram bastante entre si (a primeira indicava ~90 RPM em 1850 µs; a segunda, ~116 RPM). A segunda foi adotada por ter sido obtida com o robô em condição estável.
  * Conclusão: o hardware não era o limite. O motor entrega folga acima do alvo; o que faltava era o controle acertar o pulso.

* **Feedforward Proporcional à Velocidade (`pid_controller.h`):**
  * O PID passou a interpolar a curva PWM × RPM e corrigir por `CURVA_FATOR_CARGA` (a curva é medida sem carga; no solo o motor perde ~24% da velocidade).
  * Evolução do fator: `0,75` → `0,80`, calibrado em solo contra o RPM real medido.
  * `PID_KI` subiu de `0,6` para `1,5` quando se constatou que o teto do PID (1500 + 110 + 1,5·200 = 1730 µs) ficava abaixo dos ~1740 µs necessários sob carga.

* **Calibração do Pivô (critério de corte):**
  * O yaw registrado na telemetria **congela no instante do corte** (não inclui frenagem), o que permitiu calibrar a antecipação por comparação direta.
  * A antecipação migrou de um literal `0,5f` para a macro `PIVO_ANTECIPACAO_GRAUS`, calibrada em `0,7°` (corte em 89,3°).
  * Resultado em solo: **90,29° ± 0,54°** (amplitude 1,54°), contra 89,7–91,0° do ajuste anterior. Viés residual de +0,29°.

* **Gravador de Telemetria em RAM (`telemetry_recorder.h`):**
  * Como o robô não pode ficar com o cabo serial conectado durante o teste no solo, a telemetria passou a ser gravada em RAM (2000 amostras a 20 Hz, 100 s) e despejada depois por Serial (`d`) ou WebSocket (`dump`).
  * Armadilhas encontradas: o buffer circular enchia de amostras repetidas de `CONCLUIDO` e descartava o teste (limitado a 1 amostra/2 s); o comando `dump` não ativava a flag `_newCommand`; o timeout automático de 5 s disparava com o Wi-Fi conectado e sobrescrevia o buffer (desativado quando há Wi-Fi).

* **Correção de Concorrência nos Comandos Remotos:**
  * A flag `_newCommand` era consumida por três blocos diferentes do `loop()`. O primeiro a rodar zerava a flag e os demais nunca viam o comando — o `start` remoto parecia ignorado.
  * Solução: tratamento centralizado em um único ponto, com `switch` para `start`/`stop`/`pause`/`reset`/`dump`.

* **Teste no Solo (resultados):**

  | Métrica | Antes | Depois |
  | :--- | :---: | :---: |
  | Pico de RPM nas retas | 70,1 | 58,4 |
  | PWM máximo | 1770 µs | 1744 µs |
  | Tempo por reta | 3,45 s | 3,75 s |
  | Yaw médio do pivô | +90,33° | +90,29° |
  | Desvio padrão do pivô | 0,75° | 0,54° |
  | Amplitude do pivô | 1,35° | 1,54° |
  | Yaw final das retas | até +1,75° | −0,70° a +0,26° |
  | Ticks por reta (alvo 10.417) | 10.443–10.488 | 10.446–10.516 |

  * Nota de método: um primeiro teste com antecipação de 0,7° foi descartado porque o robô ainda não estava posicionado no solo; a repetição confirmou os números acima.

* **Lição registrada:** a velocidade final (~58 RPM contra o alvo de 55) tem importância secundária. O que determina o fechamento da trajetória é a precisão angular dos pivôs (erro sistemático acumula; erro aleatório se dispersa) e a estabilidade de rumo nas retas.

---

### 13. Resumo da Estrutura Atual do Repositório
* **`firmware_robo2wd_v4/`:** **Versão Oficial Atual e Ativa** (Wi-Fi, WebSocket, Dashboard Web e Fusão IMU).
* **`gateway_espnow/`:** Firmware de gateway para ESP32-S3 Super Mini (Ponte ESP-NOW <-> USB).
* **`robo_monitor/`:** Ferramentas Python de recepção, visualização e armazenamento de telemetria.
* **`legacy/`:** Contém as versões históricas preservadas:
  * `firmware_robo2wd/` (Versão 1 — software ISRs);
  * `firmware_robo2wd_v2/` (Versão 2 — hardware bloqueante);
  * `firmware_robo2wd_v3/` (Versão 3 — FSM não-bloqueante + IMU com fio).
* **`diag_pcnt/`:** Utilitário oficial de diagnóstico e validação dos encoders em hardware.
* **`diag_i2c_bmi160/`:** Utilitário de diagnóstico I2C da IMU Bosch BMI160.
* **`calib_imu_polaridade/`:** Ferramenta de validação da polaridade do eixo Z da IMU.
* **`diag_motor_curve/`:** Utilitário de bancada que levanta a curva PWM × RPM dos motores (com Wi-Fi/WebSocket, buffer em RAM e página web), usado para decidir entre ajuste de PID e recalibração de ESC.
* **`robo_monitor/`:** Ferramentas Python: `robo_ws_logger.py` (telemetria + `--replay` de dump para CSV), `capturar_dump.py` (dump via serial) e `capturar_dump_wifi.py` (dump via WebSocket, com reconexão).

## 📄 Atualizações Recentes (16/11/2026)

- **Feedforward por velocidade:** curva PWM×RPM medida em bancada, corrigida por `CURVA_FATOR_CARGA = 0.80`; `PID_KI = 1.5`. Pico em solo: 58,4 RPM.
- **Pivô calibrado:** `PIVO_ANTECIPACAO_GRAUS = 0.7°` (corte 89,3°), resultado 90,29° ± 0,54°.
- **Gravador em RAM:** 2000 amostras a 20 Hz com dump por Serial ou WebSocket — permite testar no solo sem cabo.
- **Correção de concorrência:** comandos remotos centralizados (a flag `_newCommand` era consumida por blocos concorrentes).
- **`diag_motor_curve` com Wi-Fi** e ferramentas de captura de dump em `robo_monitor/`.

## 📄 Atualizações Recentes (13/11/2026)

- **Firmware:** odometria recalibrada para roda de 66 mm (10.417 ticks por reta); corte de pivô por delta real de yaw (`g_yawStartSeg`) com guarda de 200 ticks e tolerância de 0,5° (89,5°); K_SYNC=0.15, IMU_KP_YAW_RETA=2.0; PWM clamp ±50 µs; giro pivô 35 RPM.
- **Dashboard:** quadrado ideal usando coordenadas reais (500 mm); rastro real em verde com odometria cumulativa; indicador de pose (X, Y, θ) numérico; seta de direção corrigida.
- **Documentação:** README atualizado com resumo das mudanças.
