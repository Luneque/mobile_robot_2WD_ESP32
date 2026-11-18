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

### 13. Etapa 13: ESP-NOW no Robô e Consolidação da Ponte (16/11/2026)

* **Problema:** a FSD declarava o ESP-NOW como parte da versão oficial e o gateway existia, mas o firmware do robô **não tinha** nenhum código ESP-NOW (0 referências a `esp_now`). A especificação prometia uma funcionalidade inexistente.
* **Implementação (`espnow_link.h`):**
  * Enlace bidirecional com o gateway ESP32-S3: o robô envia a mesma telemetria em JSON (5 Hz, via `snprintf` em buffer de 200 bytes) e recebe comandos de texto.
  * Compatível com ESP-IDF v4 e v5 — a assinatura do callback de envio mudou no core 3.x (`(mac, status)` → `(tx_info, status)`); há guarda por `ESP_IDF_VERSION_MAJOR`.
  * Canal configurável (`ESPNOW_CHANNEL`), peer registrado e fallback em broadcast.
* **Integração:** os comandos `start`/`stop`/`pause`/`reset`/`dump` passaram a ser aceitos por **WebSocket e ESP-NOW**, com dispatch centralizado (um único ponto de consumo de cada flag). O dump do gravador é devolvido pelo canal que recebeu o pedido.
* **Ferramenta `robo_monitor/gateway_espnow.py`:** ponte serial do gateway, aceita os cinco comandos pelo teclado, grava a telemetria em CSV e trata `TELEM:`/`MAC:`/`DBG:`.
* **Custo:** +9,5 KB de flash (1.051.735 bytes, 80%); RAM inalterada em 116.808 bytes (35%).
* **Pendência de validação:** o enlace ainda não foi testado de ponta a ponta com o hardware do gateway (há apenas um ESP32 disponível; o ESP32-S3 não estava conectado).

* **Resultado da validação ponta a ponta (16/11/2026, gateway ESP32 clássico em COM18):**

  | Direção | Resultado |
  | :--- | :--- |
  | Robô → gateway (telemetria) | 1.472 pacotes em 40 s (5 Hz) |
  | PC → robô (`CMD:start`) | Confirmado; robô executou `RETA → PIVO → RETA` |
  | Dump do gravador → PC | 443 amostras, de `RETA` a `CONCLUIDO` |

  * Números do dump via ESP-NOW idênticos aos do Wi-Fi: pico 58,8 RPM, pivôs 89,6°–90,7° (média 90,12°, desvio 0,40°), yaw das retas −1,00° a +0,37°, ticks 10.449–10.505.
  * **Lição de depuração:** o gateway prefixa *todo* payload com `TELEM:`, inclusive o dump — as linhas chegam como `TELEM:REC,...`. Um filtro por `startswith("REC,")` retorna zero e faz parecer que o dump falhou. A ponte `gateway_espnow.py` agora distingue JSON de telemetria de linha `REC,`.
  * **Ajustes da validação:** o gateway passa a imprimir o MAC real via `esp_wifi_get_mac()` depois do modo Wi-Fi (antes saía `00:00:00:00:00:00`); o lote do dump por ESP-NOW caiu para 1 pacote/100 ms (rajadas descartam pacotes).

---

### 14. Etapa 14: Telemetria Adaptativa (16/11/2026)

* **Problema:** a telemetria era fixa em 5 Hz (`TELEM_INTERVAL_MS = 200`), insuficiente para análise dinâmica — um pico de RPM numa reta de 3,8 s tinha ~19 amostras. O gravador em RAM (20 Hz) cobria a análise profunda, mas o dashboard e o logger em tempo real continuavam grosseiros.
* **Risco identificado antes de mexer:** o envio WebSocket é bloqueante e o loop de controle roda a 50 Hz (20 ms). Subir a taxa custa ~1,5 ms por envio; a 20 Hz seriam ~3% do tempo.
* **Solução — taxa adaptativa:** em vez de taxa fixa, o firmware escolhe pelo estado:
  * `RETA`/`PIVO`: `TELEM_INTERVAL_MOV_MS = 50` (20 Hz);
  * `AGUARDO`/`CONCLUIDO`: `TELEM_INTERVAL_IDLE_MS = 500` (2 Hz).
  * Aplicado ao WebSocket **e** ao ESP-NOW.
* **Medição em bancada:** 20,1 Hz em movimento, 2,2 Hz em repouso.
* **Validação do impacto no controle (5 Hz → 20 Hz):**

  | Métrica | 5 Hz | 20 Hz |
  | :--- | :---: | :---: |
  | Pico de RPM | 58,8 | 57,9 |
  | Tempo por reta | 3,70 s | 3,80 s |
  | Pivô médio | +90,12° | +90,22° |
  | Desvio dos pivôs | 0,40° | 0,21° |
  | Amplitude dos pivôs | 1,09° | 0,58° |

  * O controle **não degradou**: a dispersão angular caiu pela metade. A diferença de ~1 RPM e 0,1 s está dentro da variação entre execuções (bateria, piso, posicionamento).
* **Lição de método:** o dump analisado vem do gravador em RAM (20 Hz), que já era 20 Hz antes — essa comparação **não** mede o efeito da telemetria WebSocket. O que ela prova é a ausência de degradação do controle. A medição direta da taxa (20,1/2,2 Hz) é que valida a telemetria.
* **Risco residual registrado:** o custo por envio multiplica com o número de clientes conectados; com 1 cliente fica em ~3% do tempo.

---

### 15. Etapa 15: Mapa 2D com Orientação Real (17/11/2026)

* **Problema (item 5 do TODO):** o mapa desenhava o rastro com `baseDeg` nominal por lado (0/270/180/90), então o erro angular real ficava mascarado — o traçado não refletia o desvio medido.
* **Solução:** integração do **rumo global real** (`g_yawAbs`) no firmware:
  * A IMU zera o yaw a cada segmento (`resetYaw()`), então o rumo global é acumulado pelo delta entre amostras (`g_yawAbs += yaw - g_yawPrevAmostra`), com o delta zerado a cada segmento.
  * Enviado como `yawAbs` no WebSocket, no ESP-NOW e no gravador (nova coluna `yaw_abs_deg`).
  * O dashboard passou a usar esse rumo no lugar do `baseDeg`.
* **Referencial ancorado no `start`:** o `yaw_abs` é acumulado desde o boot e, após ~140 s parado, já havia derivado ~2,8°. Isso sozinho gerava **35 mm** de erro lateral em L1. O `start` (e o `reset`) agora fazem `g_yawAbs = 0`, reduzindo esse erro para **13 mm**.
* **Bug de sinal que a versão nominal escondia:** o quadrado ideal desenhado era **horário**, mas o trajeto real é **anti-horário**. O BMI160 tem o eixo Z para cima, logo yaw positivo = anti-horário (regra da mão direita); o mapa usa ângulos horário-positivos (0 = +Y, 90 = +X). O rumo renderizado passou a ser `-yawAbs`. Com o `baseDeg` nominal, o desenho e a referência errados se cancelavam — por isso o bug passou despercebido.
* **Validação no solo (robô no solo, autorização explícita do usuário):**

  | Métrica | Valor |
  | :--- | :--- |
  | `yaw_abs` | 0 → 89,6 → 178,7 → 268,0 → 357,4 |
  | Erro por lado | 13 / 20 / 18 / 14 mm |
  | Fechamento | 14 mm |
  | Pico de RPM | 54,2–55,1 (média 54,5) |
  | Pivôs | 89,78–90,98° (desvio 0,46°) |

  * No browser, o rastro verde ocupa X −6..508 / Y 3..512 e o quadrado ideal X 86..314 / Y 86..314 (pixels) — sobreposição direta.
* **Custos:** o campo novo (`int16_t`, graus × 10) levou a `RecSample` de 32 para 36 bytes por alinhamento; o buffer do gravador foi ajustado de 2000 para **1984 amostras** (99,2 s) para caber na DRAM. Compilação: 1.052.431 bytes (80%) flash, 124.240 (37%) RAM.
* **Lição de método (duas):**
  1. **Teste com o robô suspenso não valida odometria.** Em duas rodadas o yaw ficou zerado porque o robô estava suspenso para gravação — as rodas giram, mas o corpo não. A partir de agora, toda validação de orientação exige confirmação explícita de que o robô está no solo.
  2. **Não inferir a direção do movimento a partir de um comentário de código.** A correção do quadrado ideal foi feita a partir de `g_sentidoGiro = 1 // Horário`, que não correspondia ao mundo real. A sequência de pontos já havia sido definida pelo usuário e estava correta no código original.

---

### 16. Etapa 16: Chart.js Local (17/11/2026)

* **Problema (item 4 do TODO):** o dashboard carregava o Chart.js de CDN (`cdn.jsdelivr.net`). Sem upstream no AP, os gráficos não carregavam — só o mapa 2D (Canvas puro) funcionava.
* **Solução:** o Chart.js 4.5.1 (UMD minificado) passou a ser servido pelo próprio ESP32.
  * Novo `chartjs_asset.h` (gerado): o pacote em PROGMEM como array de bytes.
  * Nova rota HTTP `/chartjs.min.js` com `Content-Type: application/javascript`.
  * O `<script src>` do dashboard aponta para `/chartjs.min.js`.
* **Decisão — gzip:** o pacote minificado tem **208 KB**. A partição de app é de 1,25 MB e o sketch já ocupava 1.052 KB (80%), então sobravam ~258 KB. Servir sem compressão deixaria só ~50 KB de folga. Com **gzip** o pacote cai para **70,8 KB**, sobrando ~188 KB. A rota responde `Content-Encoding: gzip` e o browser descomprime.
  * Alternativa recusada: SPIFFS/LittleFS — exigiria formatar e manter um filesystem, mais peças móveis para o mesmo resultado.
* **Custo de build:** o array de 70,8 KB ficou em `chartjs_asset.cpp` (o header tem só a declaração). Assim editar o dashboard não recompila o asset: **editar `wifi_telemetry.h` caiu de 4m26s para 1m04s**. Regenerar o asset é a única ocasião em que o `.cpp` é tocado. O tempo de execução não é afetado.
* **Licença:** Chart.js é MIT; o aviso de licença está no header gerado.
* **Reprodutibilidade:** `ferramentas/gerar_chartjs_asset.py` baixa (ou lê) o UMD min, aplica gzip e regenera `.h` + `.cpp`.

---

### 17. Etapa 17: Testes Automatizados e CI (17/11/2026)

* **Problema (item 6 do TODO):** nenhuma validação automática. Toda mudança dependia de teste manual em solo — e o projeto já foi vítima disso: o replay quebrou duas vezes quando o número de colunas do dump mudou (12→13→14), e o teste da antecipação de 0,7° foi descartado porque o robô estava suspenso.
* **Escopo escolhido (primeira leva):** o que é puro e tem alto impacto. Ficaram de fora, por enquanto, o critério de corte do pivô (mexe no laço de controle), o perfil trapezoidal e o `yawParaRumo()` do dashboard (JS).
* **O obstáculo:** a lógica vivia embutida em `.ino`/`.h` que incluem `Arduino.h`. Solução em duas partes:
  * `firmware_robo2wd_v4/robo_matematica.h` — `ticksParaMm()` e `ticksParaArcoPivo()` extraídas do `iniciarMovimentoReta/Curva`. **Fórmulas idênticas**; o binário resultante tem o mesmo tamanho (1.123.859 bytes).
  * `tests/native/Arduino.h` — stub mínimo. Na prática os headers só usam `PI` e `abs()`; se precisarem de mais, o stub cresce.
* **Testes criados:**
  * `tests/native/test_cinematica.cpp` — **12 testes** sobre os headers **reais** (nada de fórmula copiada): `MM_POR_PULSO`, 500 mm → 10417 ticks, pivô de 90° → 1963 ticks, saturação da curva, pontos medidos em bancada e o feedforward de 55 RPM sob carga (208,6 µs).
  * `tests/python/test_ferramentas.py` — **11 testes**: replay do dump (13 colunas, valores preservados, contagem) e consistência de formato entre dump, logger, gateway e dashboard.
  * `tests/native/rodar.py` — compila e executa; escolhe `CXX`, `ziglang` (pip) ou `g++`/`clang++`.
* **CI:** `.github/workflows/ci.yml`, dois jobs:
  * `testes` — Python + nativo.
  * `build` — gera `secrets.h` a partir do exemplo e compila o firmware do robô **e** o gateway (`arduino/compile-sketches`).
* **Validação local:** 12/12 nativos e 11/11 Python passando; firmware 1.123.859 bytes (85%), gateway 885.292 bytes (67%); YAML validado.
* **Observação de formato descoberta:** o CSV do gateway tem **14** colunas (inclui `total_lados`, campo `of` do JSON) e o do replay tem **13** (o dump não grava `total_lados` para economizar RAM). Não é bug — fica documentado nos testes.
* **Próximo passo sugerido:** extrair `atingiuMetaPivo()` para testar o corte do pivô. É o teste mais valioso (a antecipação de 0,7° e a guarda de 200 ticks foram calibradas por tentativa) e o mais sensível.

---

### 18. Etapa 18: Validação do Refactor de Cinemática e a Calibração do Giroscópio (17/11/2026)

* **Objetivo:** confirmar em solo que a extração de `ticksParaMm()`/`ticksParaArcoPivo()` para `robo_matematica.h` (Etapa 17) não mudou o comportamento.
* **Resultado direto no robô:** o dump grava `ticks_alvo`, que é a saída das funções extraídas. Nos dois testes, `RETA = 10417` e `PIVO = 1963` — **idêntico**. As distâncias reais das retas (503 mm) e os ângulos de pivô (~90°) também coincidiram.
* **Primeira rodada (boot contaminado):** fechamento de **337 mm**, com deriva de **~13° por reta**. O indício: a assimetria de ticks **inverteu de sinal** (de −55 para +146) e o PWM médio caiu de 1710 para 1686 µs com o RPM subindo de 54,5 para 62 — coerente com a bateria nova, mas não explicava a inversão.
* **Causa-raiz:** o robô havia reiniciado e a bateria fora trocada. A calibração do giroscópio (`imu.calibrateGyro(150)`) roda **no boot** — com o corpo em movimento, o bias resultante faz o controlador "acreditar" que está girando. O refactor foi descartado como causa.
* **Segunda rodada (boot com o robô parado), mesmo firmware:**

  | Métrica | Bateria anterior | Boot contaminado | **Recalibrado** |
  | :--- | :---: | :---: | :---: |
  | Deriva por reta | −0,4 a −1,4° | −12 a −14° | **+0,15 a −0,88°** |
  | Erro por lado | 13/20/18/14 mm | 70/203/302/337 mm | **5/9/10/5 mm** |
  | Fechamento | 14 mm | 337 mm | **5 mm** |
  | Pico de RPM | 54,5 | 62,1 | 60,4 |
  | Pivôs | 90,20° ± 0,46 | 90,05° ± 0,63 | 90,40° ± 0,50 |

* **Conclusão:** o refactor é behavior-identical e o **melhor resultado do projeto foi obtido com ele** (5 mm de fechamento, contra 14 mm do teste anterior).
* **Requisito operacional novo:** o robô deve ser ligado com o **corpo parado** e não pode ser tocado nos primeiros segundos (boot + calibração + Wi-Fi + armamento do ESC). Isso passa a ser pré-requisito de qualquer teste em solo.
* **Efeito colateral observado:** a bateria nova eleva o RPM de cruzeiro de ~54,5 para ~60,4. O `VELOCIDADE_CRUZEIRO_RPM` continua 55 e o feedforward compensa — a trajetória segue precisa, mas vale monitorar.

---

### 19. Etapa 19: Teste do Critério de Corte do Pivô (17/11/2026)

* **Objetivo:** travar os três mecanismos do critério de parada do pivô, que até então só existiam inline no laço de controle — calibrados por tentativa e erro e invisíveis a regressão.
* **Extração:** `atingiuMetaReta()` e `atingiuMetaPivo()` em `robo_matematica.h`, com as expressões idênticas. As constantes `MIN_TICKS_PIVO` (200) e `PIVO_FALLBACK_FATOR` (1,15) saíram do corpo da função para `config.h`, onde ficam visíveis e testáveis. O `.ino` passou a usar uma única expressão ternária.
* **Cobertura — 17 testes:**
  * **Guarda de 200 ticks**: 150 e 199 ticks não param, mesmo com yaw de 89,9° e 95,0°; 200 ticks com 89,3° para.
  * **Antecipação de 0,7°**: corta em 89,30°; 89,29° não corta.
  * **Fallback 1,15× (2257 ticks)**: para com yaw de 80°; 2256 não para. É a rede de segurança contra giro infinito se a IMU falhar.
  * **Sem IMU**: para por ticks (1963 sim, 1962 não).
  * **Reta**: 10417 sim, 10416 não.
  * **Fixtures reais** dos quatro pivôs medidos no solo (1969/89,79°, 1973/90,41°, 1988/91,17°, 1983/90,24°) — todos disparam, e verifica-se que **não disparariam antes** do ponto observado. É o teste de caracterização: amarra o comportamento medido em vez de um valor inventado.
* **Ferramenta:** `tests/native/rodar.py` passou a compilar e rodar cada teste como executável separado (dois arquivos com `main` não linkam juntos).
* **Validação:** 29 testes nativos (12 de cinemática + 17 de pivô) e 11 Python, todos passando. Firmware compila com 1.123.839 bytes (85%) — 20 bytes a menos que antes da extração.
* **O que o teste NÃO cobre:** se 0,7° é a antecipação *ideal*. Ele garante que o critério faz o que está escrito; a calibração continua sendo validada em solo.

---

### 20. Etapa 20: Validação em Solo do Refactor do Critério de Corte (17/11/2026)

* **Objetivo:** confirmar que a extração de `atingiuMetaReta()`/`atingiuMetaPivo()` para `robo_matematica.h` não alterou o comportamento do pivô.
* **Método:** três execuções do quadrado no solo, com boot parado, comparando pivôs, fechamento e deriva.
* **Resultado:**

  | Execução | Fechamento | Erros/lado | Pivô médio | Desvio | Ticks no corte | RPM |
  | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
  | 1ª (bateria nova) | 5 mm | 5/9/10/5 | +90,40° | 0,50 | 1978 | 60,4 |
  | 2ª (pós-refactor) | 16 mm | 12/21/17/16 | +89,98° | 0,39 | 1951 | 59,6 |
  | 3ª | 9 mm | 7/12/12/9 | +90,31° | **0,20** | 1984 | 60,5 |

* **Conclusão:** o **pivô é estável** — média de 90,23° com amplitude de apenas **0,43°** nas três execuções, e a 3ª apresentou o menor desvio. O fechamento varia de 5 a 16 mm conforme a **deriva nas retas** (−0,15 a −0,88°), que é o fator dominante e não tem relação com o critério de corte. A 2ª execução era a ponta alta da dispersão natural.
* **Granularidade:** o corte é verificado a 50 Hz com o robô girando a ~56°/s, logo a resolução é de ~1,1° por iteração — a diferença observada (0,43°) está dentro de um quantum do laço.
* **Fechamento do item 5 do TODO:** 29 testes nativos + 11 Python, CI configurada e firmware validado em solo.

---

### 21. Etapa 21: A/B da Correção de Yaw por IMU (18/11/2026)

* **Motivação:** o usuário observou que a trajetória voltou a apresentar desvio nas retas e direção final diferente da inicial, e levantou a hipótese de que o ângulo medido pela IMU estaria atrapalhando.
* **Método:** A/B em solo, mesmas condições (boot parado, mesmo piso). Variante temporária com `IMU_KP_YAW_RETA = 0.0f` (gravada e depois restaurada para 2.0). Métrica: soma dos módulos dos erros por lado (melhor indicador de precisão, pois o fechamento pode ser cancelamento de erros em sentidos opostos).

  | Execução | Soma \|erros/lado\| | Fechamento | Direção final | Deriva por reta |
  | :--- | :---: | :---: | :---: | :--- |
  | A — kp = 2,0 | **22 mm** | 11 mm | **−0,83°** | +0,44 / −0,18 / −1,02 / −1,11 |
  | B1 — kp = 0,0 | 81 mm | 8 mm | −1,70° | −1,46 / −0,77 / +0,02 / +0,12 |
  | B2 — kp = 0,0 | **114 mm** | **40 mm** | **−6,10°** | −0,99 / −1,20 / −2,09 / −1,42 |

* **Conclusão:** a correção de yaw por IMU é **essencial**, não prejudicial. Sem ela o desvio **acumula ao longo do quadrado** (na B2 a direção final ficou 6,1° fora) e a soma dos erros foi 4–5× maior. O ganho original (2,0) foi mantido.
* **Hipótese descartada:** o sensor não é a causa do desvio residual.
* **Causa real do desvio residual:** **assimetria mecânica entre os motores** — M1 consistentemente mais lenta (−18 a −62 ticks em ~10.450, ou 0,2–0,9% de diferença de velocidade), o que gera uma curva lenta e constante. O clampe da correção é ±50 µs e o uso observado é ~22 µs, ou seja, **há margem para aumentar o ganho**.
* **Contexto histórico:** a deriva por reta variou entre **0,24° e 7,95°** em 13 execuções desde 12/11. As execuções recentes estão entre as melhores — não houve degradação, há variação.
* **Próxima alavanca:** aumentar `IMU_KP_YAW_RETA` (2,0 → 3,0–4,0) e/ou o clampe (±50 → ±80 µs).

---

### 22. Etapa 22: Tentativa de Aumentar o Ganho de Yaw (18/11/2026)

* **Objetivo:** ver se mais autoridade na correção de yaw melhora a precisão. O clampe (±50 µs) estava sendo usado em ~22 µs, havendo margem.
* **Variante:** `IMU_KP_YAW_RETA` 2,0 → 3,5 e clampe 50 → 80 µs (a macro `SYNC_CLAMP_US` foi criada nessa ocasião).

  | Métrica | kp = 2,0 | kp = 3,5 |
  | :--- | :---: | :---: |
  | Desvio de direção por lado | +0,32 / +0,08 / +0,55 / +1,21° | +1,11 / +0,61 / −0,83 / +0,73° |
  | \|desvio\| médio | **0,54°** | 0,82° |
  | Comprimento dos lados | 503/500/503/499 (amp. 4 mm) | 503/494/497/511 (amp. 17 mm) |
  | \|deriva\| média | 0,69° | **0,48°** |
  | Oscilação do yaw (σ) | 0,57 | **0,36** |
  | Direção final | −0,83° | **+0,40°** |
  | Soma \|erros/lado\| | **22 mm** | 39 mm |

* **Resultado: negativo.** O ganho maior deixou o controle mais rígido (menos oscilação, deriva mais uniforme, melhor direção final), mas **piorou a forma do quadrado** — maior desvio de direção por lado e lados menos uniformes. Com n=1, a diferença é pequena, mas não há evidência de ganho.
* **Decisão:** mantido `IMU_KP_YAW_RETA = 2.0` e clampe 50 µs. A macro `SYNC_CLAMP_US` permanece (higiene, comportamento idêntico — binário com o mesmo tamanho).
* **Conclusão mais ampla:** entre remover a correção (piora muito: 22 → 81–114 mm) e aumentá-la (não melora), o valor vigente está num ponto razoável. **O limite atual é mecânico**, não de ganho de controle.

---

### 23. Resumo da Estrutura Atual do Repositório
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
