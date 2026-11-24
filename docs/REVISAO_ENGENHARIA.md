# REVISAO_DE_ENGENHARIA.md — Revisão Geral do Projeto

Revisão de código, consistência documental e planejamento de melhorias.
Registro para acompanhar a evolução das correções ao longo do tempo.

- **Projeto:** Teste_car_0 — robô diferencial 2WD com ESP32-D0WD-V3
- **Firmware oficial:** `firmware_robo2wd_v4/` (v4.0.0)
- **Revisão nº 1:** 19/11/2026
- **Base:** commit `84ee7ac` (branch `main`, sincronizado com `origin/main`)
- **Método:** leitura integral do firmware, das ferramentas PC, dos testes e da documentação; conferência cruzada de macros × docs × CSV; três inspeções independentes em paralelo (núcleo, periféricos/dashboard, doc × código). Os itens de alta severidade foram re-verificados manualmente antes de entrar neste documento.
- **Estado do hardware na revisão:** robô em operação no solo; quadrado com fechamento de 7,8–8,7 mm e pivôs 88,2–90,7°.

Legenda de severidade:

| Marca | Significado |
| :--- | :--- |
| **[ALTA]** | Funcionalidade declarada não funciona, ou risco de reset/perda de dados |
| **[MÉDIA]** | Divergência real entre código e documentação, ou redundância que já divergiu |
| **[BAIXA]** | Código morto, comentário enganoso, higiene |

---

## 1. Sumário Executivo

A revisão encontrou **4 defeitos de alta severidade** (três deles em caminhos de
teste sem cabo, que é justamente o recurso que permite testar o robô no solo),
**5 divergências de média severidade entre código e documentação**, um bloco de
estado duplicado 6× que já divergiu, e um conjunto de código morto.

O ponto mais importante: **o dump de telemetria — o mecanismo que torna possível
testar no solo sem cabo — está quebrado em dois dos três caminhos** (WebSocket e
ESP-NOW truncam a última coluna; o dump por serial pode reiniciar o robô no meio).
O caminho que validou todos os testes até hoje (WebSocket com ~450 amostras)
funciona por estar abaixo do limite que dispara o problema.

O projeto atende às especificações iniciais: a trajetória quadrada fecha com
precisão de ~8 mm e o pivô está calibrado em ~89,7°. Nenhum dos defeitos abaixo
afeta o comportamento dinâmico do robô — afetam telemetria, comandos e
manutenibilidade.

---

## 2. Bugs Reais e Confirmados

### 2.1 [RESOLVIDO em 23/11/2026] STOP via WebSocket é silenciosamente descartado

- **Local:** `firmware_robo2wd_v4/wifi_telemetry.h:667-668`
- **Causa:** o ramo `msg == "stop"` grava `_lastCmd = 'x'` mas **não** seta
  `_newCommand = true` (todos os outros ramos setam). O dispatch central só lê
  comando se `hasNewCommand()` (`firmware_robo2wd_v4.ino:746`).
- **Efeito:** a parada remota pelo canal principal (Wi-Fi) não existe. O
  ESP-NOW funciona, porque `EspNowLink` seta a flag para qualquer payload.
- **Agravante:** o dashboard não tem botão de STOP (só START, Reta, Pivô, CSV,
  Dump) — `wifi_telemetry.h:140-144`.
- **Correção:** adicionar `_newCommand = true;` no ramo `"stop"` (1 linha) e,
  opcionalmente, um botão de parada no dashboard.

### 2.2 [RESOLVIDO em 23/11/2026] Dump remoto (WebSocket/ESP-NOW) trunca a última coluna

- **Local:** `firmware_robo2wd_v4.ino:51` (`static char _dumpLinha[96];`)
- **Causa:** a linha do gravador tem 16 colunas e precisa de mais que 96 bytes.
  Medido com os mesmos especificadores do `dumpSample()`:

  ```
  REC,303937,CONCLUIDO,5,-0.05,337.30,320.90,16.40,-0.010,60.2,59.8,10485,10470,10417,1740,1735,55.0
  ```

  = **98 bytes** (o `rpmAlvo` sai "5" em vez de "55.0"). Pior caso ~141 bytes.
- **Efeito:** o dump via WS/ESP-NOW sai cortado e o replay
  (`robo_monitor/robo_ws_logger.py:400`, que exige 17 partes) descarta a linha.
  O dump por serial não tem esse problema (usa `Serial.print`).
- **Correção:** ampliar para 160 bytes e checar o retorno do `snprintf` (o
  `return outLen - 1` atual mascara o truncamento).

### 2.3 [RESOLVIDO em 23/11/2026] Dump por serial pode reiniciar o robô no meio

- **Local:** `firmware_robo2wd_v4/telemetry_recorder.h:164-197` +
  `firmware_robo2wd_v4.ino:701`
- **Causa:** `recorder.dump()` despeja até 1700 linhas (~150 KB) a 115200 baud
  ≈ **13 s**, bloqueando o `loop()`. O `esp_task_wdt_reset()` só é chamado em
  `firmware_robo2wd_v4.ino:686`, e o TWDT de 5 s está inscrito na `loopTask`
  (`:619`, `config.h:135`).
- **Efeito:** com o buffer cheio, o dump derruba o robô no meio do despejo. O
  teste registrado (452 amostras ≈ 3,3 s) passou por estar abaixo do limite —
  por isso o problema nunca apareceu.
- **Correção:** dividir em lotes com `esp_task_wdt_reset()` entre eles (o
  padrão já existe no dump WS/ESP-NOW).

### 2.4 [RESOLVIDO em 23/11/2026] Bloco de reancoragem duplicado 6× e já divergente

- **Local:** `firmware_robo2wd_v4.ino:781-791`, `:799-808`, `:816-826`,
  `:849-859`, `:897-907`, `:915-925`
- **Causa:** a sequência `g_yawAbs=0`, `g_yawOdom=0`, `g_inerciaTotal=0`,
  `g_odomSoma=0`, `g_odomCiclos=0`, `g_ladoAtual=1`, modos, `subEstado`,
  `g_emLinhaReta`, `estadoAtual` está copiada em seis pontos.
- **Efeito:** o restart por **serial** no `ESTADO_CONCLUIDO` (`:1016-1020`)
  **não** reancora nada. O segundo quadrado consecutivo fica com o rumo do
  anterior e o mapa sai errado, sem aviso.
- **Correção:** extrair `reancorarTrajetoria()` e chamar nos seis pontos.

### 2.5 [RESOLVIDO em 23/11/2026] ESP-NOW envia 251 bytes quando o payload tem 250

- **Local:** `firmware_robo2wd_v4/espnow_link.h:95-96`
- **Causa:** `n` é limitado a `ESPNOW_MAX_PAYLOAD` (250) e depois envia
  `n + 1`. Acima do limite de 250 do ESP-NOW.
- **Efeito:** `esp_now_send` retorna erro nesse caso.
- **Correção:** enviar `n` bytes (ou limitar `n` a 249).

### 2.6 [RESOLVIDO em 23/11/2026] `SYNC_CLAMP_US` ignorado no pivô

- **Local:** `firmware_robo2wd_v4.ino:325-326` (reta) vs `:355-356` (pivô)
- **Causa:** a reta usa a macro `SYNC_CLAMP_US` (50 µs); o pivô usa o literal
  `35.0f`. A macro foi criada justamente para eliminar o literal.
- **Correção:** usar `SYNC_CLAMP_US` nos dois modos (ou criar
  `SYNC_CLAMP_PIVO_US` e documentar a diferença).

### 2.7 [RESOLVIDO em 23/11/2026] Watchdog de enlace ESP-NOW prometido e não implementado

- **Local:** `firmware_robo2wd_v4.ino:39` (`g_lastEspnowRx`) +
  `config.h:163` (`ESPNOW_LINK_TIMEOUT_MS`)
- **Causa:** `g_lastEspnowRx` é escrito em `:651` e `:764` e nunca lido; a
  macro não tem uso. O comentário em `:39` anuncia um watchdog que não existe.
- **Correção:** implementar o timeout (usar `g_lastEspnowRx`) ou remover a
  macro e a variável.

### 2.8 [RESOLVIDO em 23/11/2026] `rpmAlvo` não é o perfil trapezoidal

- **Local:** `firmware_robo2wd_v4.ino:439` e `:518-520`
- **Causa:** a telemetria promete "RPM alvo do perfil trapezoidal"
  (`wifi_telemetry.h:74`) mas envia sempre `VELOCIDADE_CRUZEIRO_RPM` /
  `VELOCIDADE_GIRO_RPM`, não o valor instantâneo (20→55) calculado em
  `calcularRPMTrapezoidal*`.
- **Correção:** enviar o valor trapezoidal real.

### 2.9 [RESOLVIDO em 23/11/2026] Comentários de convenção de giro estão errados

- **Local:** `firmware_robo2wd_v4.ino:92` ("+1 = Direita (Horário)"),
  `:213` e `imu_bmi160.h:72` ("giro a DIREITA (Horário) gera valor POSITIVO")
- **Causa:** o BMI160 tem o eixo Z para cima, logo yaw positivo = rotação
  **anti-horária** (regra da mão direita). O mapa usa `-yaw`
  (`wifi_telemetry.h:192-198`) e `robo_matematica.h:26-27` diz isso. O valor
  está correto; o rótulo é que mente.
- **Efeito:** qualquer ajuste de sinal feito seguindo esses comentários vai
  errar o sentido.
- **Correção:** alinhar os três comentários à convenção real.

### 2.10 Resolução dos itens 2.1–2.9 (23/11/2026)

Todos os nove itens foram corrigidos e verificados. Onde havia causa dupla, a
causa real era pior do que o título do item:

- **2.2:** a causa dominante **não** era só o buffer de 96 bytes — o `snprintf`
  do `dumpSample()` tinha **17 conversões para 16 argumentos** (um `%.2f` a
  mais), ou seja, comportamento indefinido e uma coluna espúria no CSV remoto.
  Corrigido para 16 × 16; `_dumpLinha` passou de 96 para `DUMP_LINE_MAX` (160).
- **2.3:** `recorder.dump()` agora despeja em lotes e chama uma função de
  cessão (`setYieldFn`) que alimenta o Task Watchdog — antes eram ~13 s
  bloqueando o `loop()`, acima do WDT de 5 s.
- **2.4:** criada `reancorarTrajetoria()`, único ponto de reancoragem. O restart
  por **serial** no `ESTADO_CONCLUIDO`, que não reancorava nada, passou a usá-la.
- **2.6:** o pivô ganhou `SYNC_CLAMP_PIVO_US` (35 µs), valor calibrado em solo,
  em vez de herdar o clamp de 50 µs da reta — mantendo o comportamento validado.
- **2.7:** `ESPNOW_LINK_TIMEOUT_MS` e `g_lastEspnowRx` eram código morto que
  prometia um watchdog; foram removidos.

**Validação com o robô no solo:**

| Verificação | Resultado |
| :--- | :--- |
| Dump via WebSocket (328 linhas) | todas com 17 campos, última coluna íntegra |
| `rt` na reta (antes: fixo em 55) | **20,0 → 55,0 RPM** na aceleração e **55 → 23,6** na desaceleração |
| `rt` no pivô / parado | 25,0 RPM / 0,0 RPM |
| Botão STOP do painel | badge `RETA → CONCLUIDO`, PWM em 1500 µs |

**Novos testes:** `tests/native/test_recorder.cpp` (formato do dump, cabeçalho de
16 campos, truncamento, última coluna) e `TestFormatStrings` no
`test_ferramentas.py` (audita **todas** as chamadas `snprintf`/`printf` do
firmware contra o número de argumentos — 15 chamadas, 0 divergências). Foi esse
teste que revelou o `snprintf` 17×16.

---


## 3. Divergências entre Código e Documentação

**Verificação em 24/11/2026 (commit `51f21f9`).** Todos os itens desta seção foram
re-verificados contra o código e a documentação vigentes:

- **3.1** o Manual passa a descrever apenas `s`/`S`, `g`/`G`, Enter e `d` na serial, e deixa
  explícito que `t`/`p`/`x`/`r` existem só por WebSocket/ESP-NOW.
- **3.2** resolvido antes pelo item 2.1: o dashboard tem o botão **⏹ STOP** desde 23/11/2026.
- **3.3** o Manual passa a apontar `gateway_espnow/gateway_espnow.ino` (não existe macro de
  canal no `config.h` do robô).
- **3.4** o Roteiro do ESP-NOW passa a dizer que a ponte **imprime** as linhas `REC,` e que
  o CSV gravado contém apenas a telemetria ao vivo.
- **3.5** a ambiguidade foi resolvida com a tabela de formatos (Manual, seção 5.3) e com a
  menção explícita de que as colunas extras são do CSV do dashboard.
- **3.6** contagens corrigidas: **16 + 17 + 9** verificações nativas e **18** Python,
  conferidas executando `tests/native/rodar.py` e `python -m unittest`.
- **3.7** comentários corrigidos (`firmware_robo2wd_v4.ino`, JS do dashboard): a taxa real é
  20 Hz em movimento / 2 Hz parado e a janela é ~3,75 s.
- **3.8** a diferença entre os formatos (16 / 17 / 19 colunas) está documentada formalmente na
  seção 5.3 do Manual, com o dump do gravador como registro de referência.
- **3.9** literais corrigidos: fator de carga 0,75 → 0,80; limiar do pivô ~83,2° → ~84,3°;
  canal ESP-NOW 1 → 11; `CHARTJS_GZ_LEN` agora citado como 70.813 bytes; docstring do dump já
  lista 16 campos; comentários de `yawAbs`/`yawReal` já corretos; aceleração ~600 → ~310 mm/s²;
  porta do gateway `COM9` → `COM18` no Manual.

### 3.1 [RESOLVIDO em 24/11/2026] Comandos seriais documentados não funcionam

- **Doc:** `docs/MANUAL_TECNICO.md:133` promete `s/t/g/d/p/x/r`.
- **Código:** o `loop()` só trata `d` (`firmware_robo2wd_v4.ino:697-703`); em
  `AGUARDO`/`CONCLUIDO` aceita `s/S/g/G` e **`g` inicia o quadrado**, não o
  pivô único (`:888-892`, `:1008-1012`). `t`, `p`, `x` e `r` são ignorados.
- **Correção:** alinhar o Manual ao comportamento real — ou implementar
  `t/p/x/r` na serial (é o mesmo dispatch, custa pouco).

### 3.2 [RESOLVIDO em 24/11/2026] "Parada remota pelo painel" prometida não existe

- **Doc:** `docs/MANUAL_TECNICO.md:128`.
- **Código:** ver item 2.1; o dashboard não tem botão de STOP.

### 3.3 [RESOLVIDO em 24/11/2026] Instrução de canal ESP-NOW aponta para macro inexistente

- **Doc:** `docs/MANUAL_TECNICO.md:158` manda ajustar `ESPNOW_CHANNEL` no
  `config.h` do robô.
- **Código:** a macro foi removida do robô (`config.h:152-163` só define
  `ESPNOW_GATEWAY_MAC` e `ESPNOW_LINK_TIMEOUT_MS`). Quem fixa canal é
  `gateway_espnow/gateway_espnow.ino:44`.
- **Correção:** trocar a instrução para o `gateway_espnow.ino`.

### 3.4 [RESOLVIDO em 24/11/2026] Roteiro do ESP-NOW promete CSV que não é gravado

- **Doc:** `docs/ROTEIRO_ESPNOW.md:142-143` ("o CSV gravado contém o teste
  completo").
- **Código:** `robo_monitor/gateway_espnow.py:157-159` imprime as linhas
  `REC,` e retorna, sem gravar.
- **Correção:** gravar as linhas `REC,` no CSV (com o cabeçalho do dump) ou
  corrigir o texto.

### 3.5 [RESOLVIDO em 24/11/2026] "CSV ganha `yawReal` e `inercia`" ambíguo

- **Doc:** `docs/FSD.md:171`, `docs/MANUAL_TECNICO.md:192`.
- **Realidade:** isso vale para o CSV do **dashboard** (`wifi_telemetry.h:459`).
  O dump do gravador (`telemetry_recorder.h:46-48`), o logger WS
  (`robo_ws_logger.py:135-152`) e o gateway (`gateway_espnow.py:35-40`) **não**
  têm esses campos — e são eles que ficam com o registro do teste no solo.

### 3.6 [RESOLVIDO em 24/11/2026] Contagem de testes desatualizada no TODO

- **Doc:** `TODO.md:74` e `:155` dizem "29 testes nativos"; `:70` diz "11
  testes" Python.
- **Realidade:** **16 + 17 = 33** testes nativos e **15** Python (verificados
  com `tests/native/rodar.py` e `python -m unittest`).

### 3.7 [RESOLVIDO em 24/11/2026] Telemetria "5 Hz" em três lugares

- **Local:** `firmware_robo2wd_v4.ino:19`, `:689` e o JS do dashboard
  (`wifi_telemetry.h:170`, `WIN=75` → "15 s a 5 Hz").
- **Realidade:** 20 Hz em movimento e 2 Hz em repouso (`config.h:148-149`).
  A janela do dashboard hoje é ~3,75 s, não 15 s.

### 3.8 [RESOLVIDO em 24/11/2026 — diferença documentada] Três formatos de CSV divergentes

- **16 colunas:** dump (`telemetry_recorder.h:46-48`), replay
  (`robo_ws_logger.py:377-382`), logger WS (`:135-152`),
  `capturar_dump_wifi.py:113`, fixture `tests/fixtures/dump_exemplo.txt:1`.
- **17 colunas:** gateway (`gateway_espnow.py:35-40`, com `total_lados`).
- **19 colunas:** dashboard (`wifi_telemetry.h:459`, com `totalLados`,
  `yawReal`, `inercia`).
- **Agravante:** o teste `tests/python/test_ferramentas.py:83-113` **fixa** a
  divergência em vez de apontá-la.
- **Correção:** padronizar (ou documentar formalmente a diferença e fazer o
  teste comparar de fato).

### 3.9 [RESOLVIDO em 24/11/2026] Outras divergências pontuais

| Item | Doc | Realidade |
| :--- | :--- | :--- |
| `CURVA_FATOR_CARGA` | `pid_controller.h:19` cita 0,75 | `config.h:52` = 0,80 |
| Limiar do pivô | `tests/native/test_pivo.cpp:30-33,60-63` dizem ~83,2° | 84,3° (5,7°) |
| `CHARTJS_GZ_LEN` | `chartjs_asset.h:6-7` diz 69,2 KB | 70.813 bytes |
| Canal ESP-NOW | `gateway_espnow.ino:27` diz "CANAL ESP-NOW: 1" | `:44` = 11 |
| Docstring do dump | `telemetry_recorder.h:20` lista 13 campos | 16 campos |
| `yawAbs` | `wifi_telemetry.h:58` "absoluto desde o boot" | zera no `start` |
| `yawReal` | `wifi_telemetry.h:66` "yawAbs + inercia" | `yawAbs + g_inerciaTotal` |
| Aceleração | `docs/FSD.md:114`, `MANUAL_TECNICO.md:102` "≈600 mm/s²" | ~167–460 mm/s² |
| Porta do gateway | Manual `COM18`/`COM9`, Roteiro `COM10`, default `COM9` | validado em `COM18` |
| TODO × git | `TODO.md:3-4,24` citam `3ca004f` | HEAD `84ee7ac` |

---

## 4. Código Morto e Redundante

### 4.1 Macros e variáveis sem uso

Verificado por varredura em todo o firmware (`firmware_robo2wd_v4/`):

| Identificador | Local | Observação |
| :--- | :--- | :--- |
| `PIN_BATTERY_ADC` | `config.h:26` | monitor de bateria não implementado |
| `ESC_PWM_FREQ_HZ` | `config.h:44` | MCPWM hardcodeia `period_ticks=20000` |
| `PPR_MOTOR`, `REDUCAO_MOTOR`, `QUADRATURA_MULT` | `config.h:31-33` | `PPR_EFETIVO` é literal |
| `ESPNOW_LINK_TIMEOUT_MS` | `config.h:163` | ver item 2.7 |
| `g_distanciaTotalMm` | `.ino:90` | só escrita (`:176`) |
| `g_gyroPeakDps` | `.ino:113` | só escrita e impressa |
| `g_lastEspnowRx` | `.ino:39` | ver item 2.7 |
| `REC_EST_PAUSA` | `telemetry_recorder.h:79,91` | nunca retornado |
| `recorder.restart/overflow/setRecording` | `telemetry_recorder.h:156,224,225` | nunca chamados |
| `getDistanceM1_mm/M2_mm` | `encoder_reader_pcnt.h:168-169` | só no legado V2 |
| `isInitialized()` (encoder/motor/espnow) | três headers | só o da IMU é usado |
| `getYawRateDps()` | `imu_bmi160.h:108` | `getRawYawRateDps()` é a usada |
| `setTunings`, `getLastOutput` | `pid_controller.h:50,91` | nunca chamados |
| `EspNowLink::isInitialized/lastSendOk` | `espnow_link.h:108-109` | nunca chamados |
| `WifiTelemetry::isConnected` | `wifi_telemetry.h:611` | o loop usa `WiFi.status()` |
| `BMI160_DATA_REG` | `imu_bmi160.h:17` | nunca usado |
| `prevSegSide`, `prevSegState` | `wifi_telemetry.h:176,187` (JS) | só atribuídos |
| `#badge.PAUSA` / `STATE_CLS` com `PAUSA` | `wifi_telemetry.h:108,403` | estado nunca enviado |

### 4.2 Estados e ramos inalcançáveis

- `ESTADO_ARMANDO_ESC` (`.ino:70`) não tem `case` no `switch(estadoAtual)` do
  `loop()` — se algo o alcançar, o loop não faz nada.
- `sentido < 0` em `iniciarMovimentoCurva` (`.ino:195-214`): só é chamada com
  `sentido = 1` (`:936`); todo o ramo "ESQUERDA" é inalcançável.
- `REC_EST_PAUSA` e o badge "PAUSA" do dashboard nunca são acionados.

### 4.3 Redundâncias

- **Mapeamento estado→string 3×:** `.ino:394-415` (WebSocket),
  `:463-468` (ESP-NOW), `:491-501` (gravador) — com diferenças (o ESP-NOW não
  zera o RPM).
- **Reancoragem 6×:** ver item 2.4.
- **Leitura de sensores repetida:** `abs(getTicksM1/M2())` em 4-6 pontos por
  ciclo (`:258-260`, `:389-390`, `:459-460`, `:515-516`) e
  `g_yawAbs - g_yawOdom` recalculado 3×.
- **Constante duplicada:** `MM_PER_TICK = 0.04799656` no JS
  (`wifi_telemetry.h:174`) duplica `MM_POR_PULSO` do `config.h`.

---

## 5. Riscos de Robustez (além dos itens 2.1–2.3)

| Local | Risco |
| :--- | :--- |
| `espnow_link.h:120-148` | `Serial.print` dentro dos callbacks ESP-NOW (contexto da tarefa Wi-Fi) concorre com o `loop()` no `Serial` |
| `espnow_link.h:114-115` | `_lastCmd[64]` não é volátil/atômico; a leitura de 64 B não é atômica |
| `imu_bmi160.h:112-139` | `readReg`/`writeReg` ignoram `Wire.endTransmission()`; `readRawGyroZ()` devolve `0.0f` em falha de I2C (indistinguível de "parado") |
| `imu_bmi160.h:81-94` | `update(dt_sec)` não valida `dt_sec <= 0` |
| `motor_control_mcpwm.h:29-93`, `encoder_reader_pcnt.h:31-115` | falha parcial no `begin()` não destrói timer/operadores/unidades já alocadas |
| `encoder_reader_pcnt.h:130-148` | `updateSpeed()` só roda em `SUB_EM_MOVIMENTO`; em `AGUARDO` o RPM fica congelado |
| `.ino:240` | `abs(d1) < 10000` é um guard heurístico: se o PCNT saturar, a odometria deixa de ser integrada silenciosamente |
| `.ino:45-49`, `:706-738` | `_dumpIdx`/`_dumpTotal`/`_dumpUltimoEnvio` são compartilhados entre o dump WS e o ESP-NOW |
| `wifi_telemetry.h:422-424,231` | `csvRows` e `trajectory` crescem sem teto no browser |
| `wifi_telemetry.h:463` | `URL.createObjectURL()` nunca é revogado |

---

## 6. O que NÃO é problema (alegações descartadas)

Três suspeitas levantadas durante a inspeção foram verificadas e **não** se
confirmam — registradas aqui para não serem "redescobertas":

1. **`snprintf` do `dumpSample()` com 17 especificadores e 16 argumentos**
   (`telemetry_recorder.h`): **verdadeiro — e foi corrigido**. A suspeita levantada
   na inspeção era correta e foi confirmada pelo compilador do teste nativo
   (`-Wformat-insufficient-args`). Havia um `%.2f` a mais na linha do dump:
   comportamento indefinido no `snprintf` e uma coluna espúria no CSV remoto.
   Corrigido para 16 conversões × 16 argumentos; agora coberto por
   `test_conversoes_batem_com_argumentos` e por `tests/native/test_recorder.cpp`.
   (Registro honesto: esta alegação havia sido descartada numa verificação
   anterior por contagem manual equivocada.)
2. **`abs()` em float** (`imu_bmi160.h:89`, `pid_controller.h:57`, `.ino:285`):
   **falso**. No core 3.3.12 resolve para `std::abs(float)`, sem truncamento.
   O stub de `tests/native/Arduino.h` está correto.
3. **`g_yawAbs` "horário"**: o valor está correto; só o rótulo do comentário
   mente (item 2.9).

---

## 7. Plano de Melhorias

### Limites do hardware (restrições para qualquer proposta)

| Recurso | Uso | Livre |
| :--- | :--- | :--- |
| Flash | 1.130.039 / 1.310.720 bytes (86%) | ~180 KB |
| DRAM | 120.864 / 327.680 bytes (36%) | ~207 KB (68 KB são o gravador) |
| CPU | 240 MHz, dual core | Wi-Fi + ESP-NOW no mesmo rádio |
| Gravador | 1700 amostras × 40 bytes = 68 KB (85 s) | já estourou o DRAM uma vez |

### P1 — Corrigir bugs (baixo risco, maior impacto por linha)

| # | Ação | Local |
| :--- | :--- | :--- |
| 1.1 | Setar `_newCommand = true` no ramo `"stop"` do WebSocket | `wifi_telemetry.h:667` |
| 2.2 | Ampliar `_dumpLinha` para 160 bytes e checar o retorno do `snprintf` | `.ino:51` |
| 2.3 | Dump serial em lotes com `esp_task_wdt_reset()` entre eles | `telemetry_recorder.h:164` |
| 2.4 | Extrair `reancorarTrajetoria()` e chamar nos 6 pontos + restart serial | `.ino:781-925`, `:1016` |
| 2.5 | ESP-NOW enviar `n` em vez de `n+1` | `espnow_link.h:96` |
| 2.6 | Usar `SYNC_CLAMP_US` no pivô | `.ino:355-356` |

### P2 — Consistência e manutenção

| # | Ação |
| :--- | :--- |
| 3.1 | Alinhar o Manual aos comandos seriais reais (ou implementar `t/p/x/r`) |
| 3.3 | Corrigir a instrução de `ESPNOW_CHANNEL` no Manual |
| 3.4 | Gravar as linhas `REC,` no CSV do gateway (ou corrigir o Roteiro) |
| 3.6 | Atualizar a contagem de testes no TODO (33 nativos / 15 Python) |
| 3.7 | Corrigir os textos de "5 Hz" e a janela do dashboard |
| 3.8 | Padronizar o CSV (ou documentar e fazer o teste comparar de fato) |
| 3.9 | Corrigir as divergências pontuais da seção 3.9 |
| 4.1 | Eliminar o código morto (libera flash e confusão) |
| 4.2 | Resolver os estados/ramos inalcançáveis |
| 2.7 | Implementar o watchdog de enlace ESP-NOW ou remover a macro/variável |
| 2.8 | Enviar o `rpmAlvo` real do perfil trapezoidal |
| 2.9 | Corrigir os comentários de convenção de giro |

### P3 — Horizonte (exige decisão)

| # | Ação | Observação |
| :--- | :--- | :--- |
| — | Gravar `yawReal`/`inercia` no gravador e nos CSVs | +12 bytes/amostra (40→52); o buffer vai a ~88 KB. Ainda cabe, mas convém reduzir `REC_MAX_SAMPLES` para ~1300 para manter 85 s |
| — | `reta`/`pivo` no gateway ESP-NOW | hoje a ponte só oferece start/stop/pause/reset/dump |
| — | Botão de STOP no dashboard | depende de 2.1 |
| — | Monitor de bateria | **ATENDIDO (19/11/2026)** — divisor R1=100k/R2=47k + 100 nF montado no GPIO 34; driver `battery_monitor.h`, limiares 7,0/6,6 V, corte em `ESTADO_EMERGENCIA`, telemetria `vbat` e calibração persistida na NVS |
| — | Escala de distância (TODO item 22) | ticks dizem 503–504 mm; medido 480 mm. Se resolver, revisar `PPR_EFETIVO`/diâmetro e o mapa de uma vez |
| — | Limitar `csvRows`/`trajectory` no dashboard | crescimento sem teto no browser |
| — | Validar `Wire.endTransmission()` na IMU | hoje falha de I2C é indistinguível de "parado" |

---

## 8. Acompanhamento

Este documento é o registro base da Revisão nº 1. A cada rodada de
correções, atualizar a coluna **Estado** abaixo e referenciar o commit.

| Item | Descrição | Severidade | Estado |
| :--- | :--- | :---: | :--- |
| 2.1 | STOP via WebSocket descartado | ALTA | **resolvido** |
| 2.2 | `_dumpLinha[96]` trunca o dump remoto | ALTA | **resolvido** (e causa real era o `snprintf` 17×16) |
| 2.3 | Dump serial estoura o WDT de 5 s | ALTA | **resolvido** |
| 2.4 | Reancoragem duplicada 6× e divergente | ALTA | **resolvido** |
| 2.5 | ESP-NOW envia 251 bytes | MÉDIA | **resolvido** |
| 2.6 | `SYNC_CLAMP_US` ignorado no pivô | MÉDIA | **resolvido** (`SYNC_CLAMP_PIVO_US`) |
| 2.7 | Watchdog de enlace ESP-NOW não implementado | MÉDIA | **resolvido** (macro/variável mortos removidos) |
| 2.8 | `rpmAlvo` não é o trapezoidal | MÉDIA | **resolvido** |
| 2.9 | Comentários de convenção de giro errados | MÉDIA | **resolvido** |
| 3.1 | Comandos seriais do Manual não funcionam | MÉDIA | pendente |
| 3.2 | "Parada remota pelo painel" não existe | MÉDIA | pendente |
| 3.3 | Instrução de `ESPNOW_CHANNEL` aponta macro removida | MÉDIA | pendente |
| 3.4 | Roteiro ESP-NOW promete CSV não gravado | MÉDIA | pendente |
| 3.5 | "CSV ganha `yawReal`/`inercia`" ambíguo | BAIXA | pendente |
| 3.6 | Contagem de testes do TODO desatualizada | MÉDIA | pendente |
| 3.7 | Telemetria "5 Hz" em três lugares | BAIXA | pendente |
| 3.8 | Três formatos de CSV divergentes | MÉDIA | pendente |
| 3.9 | Divergências pontuais (tabela) | BAIXA | pendente |
| 4.1 | Código morto (tabela) | BAIXA | pendente |
| 4.2 | Estados/ramos inalcançáveis | BAIXA | pendente |
| 4.3 | Redundâncias (estado→string, sensores) | BAIXA | pendente |
| 5 | Riscos de robustez (tabela) | BAIXA | pendente |

---

## Referências

- `docs/DIARIO_ENGENHARIA.md` — Etapas 23–25 (diagnóstico de rumo e inércia).
- `TODO.md` — itens 22 (escala de distância) e 23 (divergência IMU × odometria).
- `docs/FSD.md` — tabela de parâmetros homologados.
- `tests/python/test_ferramentas.py` — testes de formato (a ampliar, item 3.8).
