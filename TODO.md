# TODO — Projeto Robô 2WD (Teste_car_0)

Revisão completa do estado do projeto em **19/11/2026**, após o commit `3ca004f`
(rumo real com inércia, pivô recalibrado, diagnóstico de rumo, mapa com rumo real).

Legenda de prioridade: **[P1]** bloqueia ou degrada a operação · **[P2]** qualidade/robustez · **[P3]** higiene/evolução.

---

## 1. Estado Atual (resumo)

- **Firmware oficial:** `firmware_robo2wd_v4/` (v4.0.0). Compila: **1.130.039 bytes (86%)** flash, **120.864 bytes (36%)** RAM.
- **Último teste em solo válido** (robô posicionado corretamente, 19/11/2026):

  | Métrica | Valor | Alvo |
  | :--- | :---: | :---: |
  | Fechamento do quadrado | **7,8–8,7 mm** | 0 |
  | Lados (pelos ticks) | 503–504 mm | 500 |
  | Pivôs (rumo real) | 88,2–90,7° (média ≈ 89,7°) | 90,0° |
  | Erro de direção final | −1,3° a −2,4° | 0° |
  | Divergência IMU × odometria | ~5° por pivô (acumulada ~20°) | — |

- **Working tree:** limpo.
- **Branch:** `main` sincronizado com `origin/main` até `864c6d9`.
- **Testes:** 16 + 17 + 9 testes nativos (`test_cinematica.cpp`, `test_pivo.cpp`, `test_recorder.cpp`) e 18 Python, CI verde.

---

## 2. Problemas em Aberto (por prioridade)

### [P1] Funcionalidades declaradas mas não implementadas

1. ~~**ESP-NOW: lado do robô não existe.**~~ **ATENDIDO E VALIDADO (16/11/2026).**
   - Implementado em `espnow_link.h`: telemetria JSON ao gateway ESP32 (5 Hz) e recepção de `start`/`stop`/`pause`/`reset`/`dump`.
   - Comandos aceitos por WebSocket **e** ESP-NOW, com dispatch centralizado; o dump é devolvido pelo canal que recebeu o pedido.
   - Adicionada a ponte `robo_monitor/gateway_espnow.py` (serial do gateway, grava CSV).
   - **Validado de ponta a ponta** com gateway ESP32 clássico (COM18): robô→gateway 1472 pacotes/40 s; `CMD:start` confirmado e robô executou; dump devolvido com 443 amostras (RETA→CONCLUIDO). Números do dump idênticos aos do Wi-Fi (pico 58,8 RPM, pivôs 89,6–90,7°, desvio 0,40°).
   - **Lição:** o gateway prefixa *todo* payload com `TELEM:`, inclusive o dump — as linhas chegam como `TELEM:REC,...`. Filtros por `startswith("REC,")` retornam 0. A ponte agora distingue os dois formatos.
   - **Restrição:** gateway e robô precisam de canal coincidente. O canal do robô é o do AP (medido: 11); o gateway fixa o canal (`ESPNOW_CHANNEL`). O robô registra o peer com canal 0.
   - **Armadilha:** `esp_wifi_set_channel()` após o connect Wi-Fi derruba o enlace e trava o `setup()` — não usar.

2. **Monitor de bateria não implementado.**
   - `PIN_BATTERY_ADC = -1` (desabilitado) e nenhuma leitura de tensão no firmware.
   - Sem proteção contra LiPo 2S profunda: risco real de dano à bateria e de o robô parar no meio do percurso.
   - **Divisor de tensão (validado em 19/11/2026):** R1 = 100 kΩ (bateria → GPIO) e R2 = 47 kΩ (GPIO → GND), com **C = 100 nF em paralelo com R2** (filtro). O capacitor é o que mantém a leitura estável (ADC entre 2991 e 3024 em 40 leituras). Atenuação de 11 dB no ADC.
  - **Ganho real medido:** ~**0,00273 V/ADC** — 8,22 V nos terminais deram ADC 3013.
  - **O ADC do ESP32 com 11 dB não é linear:** a curva nominal (`adc/4095 × 2,5 V`) superestima por um fator de ~96. Por isso a calibração com multímetro é obrigatória.
  - **Recalibrar:** meça a bateria no multímetro e envie `vbat <tensão>` pelo dashboard; o valor fica gravado na NVS e sobrevive ao reboot.

3. **Sem proteção de baixa tensão / sem telemetria de corrente.**
   - Relacionado ao item anterior. O ESC é escovado (bidirecional duplo) de 2-3S; operar abaixo de ~6,4 V danifica a célula.

### [P2] Robustez e qualidade

4. ~~**Dashboard depende de internet (Chart.js via CDN).**~~ **ATENDIDO (17/11/2026).**
   - O Chart.js 4.5.1 (UMD min) passou a ser servido pelo próprio ESP32 em `/chartjs.min.js`; o dashboard não depende mais de CDN nem de upstream do AP.
   - Ativo: `chartjs_asset.h` + `chartjs_asset.cpp` (gerados por `ferramentas/gerar_chartjs_asset.py`), com o pacote **gzip** (70,8 KB em vez de 208 KB) em PROGMEM; a rota responde `Content-Type: application/javascript` + `Content-Encoding: gzip`.
   - O array vive em `.cpp` justamente para não recompilar quando o dashboard muda: editar `wifi_telemetry.h` caiu de 4m26s para 1m04s.
   - Compressão foi necessária porque o app está em 80% da flash (1,25 MB de partição): sem gzip o pacote custaria 208 KB e deixaria só ~50 KB de folga.

5. ~~**Mapa 2D usa ângulos nominais por lado.**~~ **ATENDIDO (17/11/2026).**
   - Removido o `baseDeg` nominal por lado. O mapa agora integra o **rumo global real** (`g_yawAbs`), acumulado do yaw da IMU e ancorado no `start`.
   - O firmware envia `yawAbs` no WebSocket, no ESP-NOW e no gravador (nova coluna `yaw_abs_deg` no CSV).
   - **O erro angular real passou a aparecer** — que era exatamente o objetivo.
   - Validação no solo: trajetória anti-horária fechando em **14 mm**; erro por lado 13/20/18/14 mm; pico de 54,5 RPM; pivôs 89,78–90,98°.
   - **Correção de rumo que a versão nominal escondia:** o quadrado ideal desenhado era horário, mas o trajeto real é anti-horário. O BMI160 tem Z para cima (yaw positivo = anti-horário, regra da mão direita) e o mapa usa convenção horária, então o rumo renderizado é `-yawAbs`.
   - **Deriva em repouso:** o `yaw_abs` acumulado desde o boot chegava a ~2,8° no momento do `start` (140 s parado), o que sozinho gerava ~35 mm de erro lateral em L1. O referencial agora é ancorado no `start` (`g_yawAbs = 0`), reduzindo esse erro para 13 mm.
   - **Atualização (19/11/2026):** o rastro passou a usar o **rumo real** (`yawReal` = corte + inércia de **todos** os pivôs). Com `yawAbs` (rumo congelado no corte) o traçado saía torto ~5° por pivô. O `yawReal` também foi corrigido: antes somava só a inércia do último segmento, deixando o rumo final ~14° errado.

6. ~~**Sem testes automatizados.**~~ **PARCIALMENTE ATENDIDO (17/11/2026).**
   - Criado o diretório `tests/` com testes que rodam **sem hardware**:
     - `tests/native/test_cinematica.cpp` — 12 testes sobre os headers **reais** do firmware (a única dependência de Arduino é suprida pelo stub `tests/native/Arduino.h`): `MM_POR_PULSO`, ticks por reta de 500 mm (10417), ticks do pivô de 90° (1963), interpolação da curva PWM×RPM e o feedforward de 55 RPM sob carga. Roda com `python tests/native/rodar.py` (escolhe `CXX`, `ziglang` ou `g++`/`clang++`).
     - `tests/python/test_ferramentas.py` — 11 testes: replay do dump para CSV (colunas, valores, contagem) e consistência de formato entre dump, logger, gateway e dashboard. É a classe de bug que já quebrou de verdade (o replay falhou quando as colunas mudaram 12→13→14).
   - **CI:** `.github/workflows/ci.yml` com dois jobs — `testes` (Python + nativo) e `build` (compila o firmware do robô e o gateway via `arduino/compile-sketches`).
   - Para viabilizar os testes nativos, `ticksParaMm()` e `ticksParaArcoPivo()` foram extraídas para `firmware_robo2wd_v4/robo_matematica.h` — **as fórmulas são idênticas** e o binário resultante tem o mesmo tamanho (1.123.859 bytes).
   - **O que falta para fechar o item:** testes do critério de corte do pivô (extrair `atingiuMetaPivo()`), do perfil trapezoidal e do `yawParaRumo()` do dashboard (JS). Os dois últimos exigem mais refactor; o do pivô é o mais valioso e o mais sensível (mexe no laço de controle).
   - **Atualização (17/11/2026):** `tests/native/test_pivo.cpp` criado — **17 testes** do critério de parada (guarda de 200 ticks, antecipação, fallback 1,15×, modo sem IMU e reta), incluindo **fixtures reais** dos quatro pivôs medidos no solo. `atingiuMetaReta()`/`atingiuMetaPivo()` extraídas para `robo_matematica.h`, com as expressões idênticas. Total: **29 testes nativos**.
   - **Atualização (19/11/2026):** `tests/python/test_ferramentas.py` — **15 testes**, incluindo `test_js_do_dashboard_compila` (compila o `<script>` do dashboard com `node --check`; um `)` a mais já quebrou o JS inteiro **em silêncio**) e `test_mapa_usa_o_rumo_real`.

7. ~~**Telemetria de 5 Hz é pouco para análise dinâmica.**~~ **ATENDIDO (16/11/2026).**
   - Implementada **telemetria adaptativa**: 20 Hz durante `RETA`/`PIVO`, 2 Hz em `AGUARDO`/`CONCLUIDO` (WebSocket e ESP-NOW).
   - Medido em bancada: **20,1 Hz em movimento, 2,2 Hz em repouso** (alvos 20/2 Hz).
   - Validação do impacto no controle: pivôs com desvio 0,40° → **0,21°** e amplitude 1,09° → **0,58°**; RPM 58,8 → 57,9; tempo de reta 3,70 → 3,80 s. Controle não degradou.
   - O gravador em RAM (20 Hz) já cobria a análise profunda; o ganho está no dashboard/logger em tempo real (~76 amostras por reta, contra ~19 antes).
   - Risco residual: o envio WebSocket é bloqueante; se o número de clientes crescer, o custo por envio multiplica. Com 1 cliente, ~3% do tempo.

### [P3] Higiene de código e documentação

8. ~~**Comentários desatualizados em `config.h`.**~~ **ATENDIDO (18/11/2026).**
   - `VELOCIDADE_CRUZEIRO_RPM`: "~40 RPM real" → medido ~60 RPM pico.
   - `RAMPA_ACEL_DIST_MM`: "20 a 80 RPM" → 20 a 55 RPM.
   - `CURVA_FATOR_CARGA`: comentário reescrito com a medição de 17/11 (pico ~60 RPM).
   - `K_SYNC`: dizia "assimetria M2>M1"; a direção observada varia entre execuções — texto tornado neutro.
   - `PID_KI_M1/M2`: comentário histórico "0.6 -> 1.5" reduzido à justificativa vigente.

9. ~~**Macro `IMU_HABILITADA` é código morto.**~~ **ATENDIDO (18/11/2026)** — removida. O código usa `imu.isInitialized()` diretamente; a macro nunca foi referenciada.
   - **Bônus:** `ESC_DEADBAND_US` e `ESPNOW_CHANNEL` (do robô) também estavam mortos e foram removidos. Quem fixa canal é o gateway; o robô usa o canal atual da interface.

10. ~~**`PID_KI` com comentário histórico no valor.**~~ **ATENDIDO (18/11/2026)** — ver item 8.

11. ~~**FSD ainda cita "~40 RPM real em solo".**~~ **ATENDIDO (18/11/2026)** — item 4 da seção 2.2 e a linha da tabela do `ESC_DEADBAND_US` (macro removida) corrigidos.
    - `docs/FSD.md:27`: texto de 13/11 não atualizado com o resultado de 16/11 (~58 RPM).

12. ~~**Manual Técnico não documenta o gravador em RAM nem as ferramentas de dump.**~~ **ATENDIDO (19/11/2026)** — seção 5.3 (gravação em solo sem cabo), 5.4 (ESP-NOW) e a lista de comandos seriais (`s`/`t`/`g`/`d`/`p`/`x`/`r`) documentadas; os valores de 35 RPM, 0,7° e 2000 amostras foram corrigidos para **25 RPM, 5,7° e 1700 amostras**.

13. **Diário com referências antigas a 89,5°.**
    - Etapas 9–11 citam "corte em 89,5°"; as etapas 20–24 citam 89,3° com `PIVO_ANTECIPACAO_GRAUS = 0,7°`; **o valor vigente é corte em ~84° com antecipação de 5,7°** (Etapa 25).
    - Como é um registro cronológico, é intencional — mas o TODO e o FSD passam a apontar o valor atual.

14. **`diag_motor_curve/secrets.h` existe no disco e está ignorado pelo `.gitignore`.**
    - Correto do ponto de vista de segurança (`**/secrets.h`), mas o sketch não tinha um `secrets.h.example` próprio (o firmware V4 tem). **ATENDIDO (18/11/2026)** — `diag_motor_curve/secrets.h.example` criado.

15. ~~**Artefato órfão `diag_motor_curve/gráfico.png`.**~~ **SEM OBJETO (18/11/2026)** — o arquivo não existe mais no disco (nunca foi rastreado). Nada a decidir.

16. **`legacy/` com V1/V2/V3.**
    - Documentado como preservação histórica. Conforme o pretendido; apenas registrar que não deve ser compilado por engano.

---

## 3. Pontos de Atenção Técnicos (não são bugs, mas merecem registro)

17. **Duas medições da curva PWM×RPM divergiram muito.**
    - Primeira: ~90 RPM em 1850 µs. Segunda (adotada): ~116 RPM.
    - A diferença provavelmente vem de condição de bancada/bateria. Se o hardware for mexido (roda, bateria, motor), a curva precisa ser revalidada — o feedforward depende dela.

18. **Fator de carga é constante, não adaptativo.**
    - `CURVA_FATOR_CARGA = 0.80` foi calibrado para o peso e o piso atuais.
    - Bateria fraca, piso diferente ou rodas gastas descalibram o feedforward. O PID absorve parte do erro, mas o ponto de operação se desloca.

19. **Zona morta da curva tem dois pontos colapsados.**
    - `CURVA_RPM = {0.0, 1.9, 35.0, ...}` com `CURVA_PWM = {1600, 1600, 1650, ...}`.
    - Para alvos abaixo de 1,9 RPM a interpolação retorna sempre 1600 µs. Hoje o mínimo é 20 RPM, então não afeta — mas é frágil se `VELOCIDADE_MIN_RPM` mudar.

20. **`snprintf` usado no `dumpSample()` do gravador.**
    - Depende de `stdio` embutido; funciona no ESP32, mas é mais pesado que `dtostrf`/`utoa`. Sem impacto mensurável hoje (10 amostras a cada 50 ms).

21. **O buffer do gravador é volátil.**
    - Se o robô reiniciar ou desligar antes do dump, os dados se perdem. Aceitável para esta etapa, mas vale registrar.

22. **Escala de distância em aberto.**
    - Os ticks dizem **503–504 mm** por lado de 500 mm (+0,7%); no chão o usuário mediu **480 mm** (−4%). Com a circunferência efetiva medida (205 mm) e `PPR_EFETIVO = 4320`, o esperado seria ~500 mm.
    - Hipóteses: deslizamento na partida/chegada, ou `PPR_EFETIVO` real ≈ 4490.
    - **Não afeta** a navegação nem o fechamento (7,8–8,7 mm), mas a escala do mapa e o alvo nominal de 500 mm ficam errados em ~4%.
    - **Como fechar:** rodar o comando `📏 Reta 500` e medir no chão do ponto de partida à marca de parada, comparando com os ticks reportados.

23. **Divergência IMU × odometria de ~5° por pivô.**
    - Não afeta o controle (que usa a IMU), mas o painel `diag` e o `rumo odometria` do mapa carregam essa diferença. O alerta foi ajustado para o acumulado (> 30°).
    - Provável causa: os encoders medem o giro **depois** do corte (a inércia não aparece nos ticks) e a escala de distância (item 22).

---

## 4. Sugestão de Ordem de Trabalho

1. ~~**[P1]** Decidir o destino do ESP-NOW~~ — **atendido**: implementado o lado do robô (`espnow_link.h`) + ponte `gateway_espnow.py`. **Validado com um ESP32 clássico** (ESP-WROOM-32, `COM18`); o ESP32-S3 era o plano original, mas a placa apresentou defeito. Resta, se desejado, trocar broadcast por MAC dedicado.
2. **[P1]** Monitor de bateria (hardware + `PIN_BATTERY_ADC` + alerta/telemetria).
3. ~~**[P2]** Chart.js local (baixar o arquivo para PROGMEM ou servir do SPIFFS).~~ **ATENDIDO (17/11/2026)** — servido de `/chartjs.min.js` pelo próprio ESP32 (gzip em PROGMEM).
4. ~~**[P2]** Mapa 2D com orientação real medida (remover `baseDeg` nominal).~~ **ATENDIDO (17/11/2026)** — ver item 5 dos problemas de fundo.
5. ~~**[P2]** Testes automatizados: ao menos um teste de regressão da interpolação da curva e do critério de corte do pivô, com CI.~~ **ATENDIDO (17/11/2026)** — CI criada; 29 testes nativos (incluindo o critério de corte do pivô) e 11 Python passando.
6. ~~**[P3]** Higiene: comentários do `config.h`, remover `IMU_HABILITADA`, atualizar FSD:27 e a seção 5 do Manual, `secrets.h.example` para o `diag_motor_curve`.~~ **ATENDIDO (18/11/2026)** — ver itens 8–15 da seção [P3].
7. ~~**[P3]** Decidir o destino de `diag_motor_curve/gráfico.png`.~~ **SEM OBJETO (18/11/2026)** — o arquivo não existe mais no disco.

---

## 5. Verificações Realizadas nesta Revisão

- **Código:** ausência de `esp_now` no firmware do robô confirmada por grep; `IMU_HABILITADA` sem referências; `PIN_BATTERY_ADC = -1` sem uso.
- **Documentação:** valores confrontados entre `config.h`, FSD, Manual, Diário e README (itens 8–13).
- **Git:** `git status` limpo exceto o PNG; `git ls-files` confirma que apenas `secrets.h.example` está rastreado (sem vazamento de credenciais).
- **Compilação:** 1.130.039 bytes (86%) flash, 120.864 bytes (36%) RAM.
- **Funcional:** replay do dump testado (452 amostras → CSV com cabeçalho correto).
- **Solo (19/11/2026):** quadrado com fechamento de 7,8–8,7 mm; pivôs 88,2–90,7°; mapa fechando em ~7 mm.

---

## 6. Revisão de Engenharia (19/11/2026)

Auditoria completa de código × documentação registrada em
[`docs/REVISAO_ENGENHARIA.md`](docs/REVISAO_ENGENHARIA.md), com plano de
melhorias em três prioridades e tabela de acompanhamento.

**4 defeitos de alta severidade** (todos em telemetria/comandos; nenhum afeta a
dinâmica do robô):

1. **STOP via WebSocket é descartado** — `wifi_telemetry.h:667` grava o comando
   sem setar `_newCommand`; o dispatch nunca o processa. A parada remota pelo
   canal principal não existe (só o ESP-NOW para).
2. **Dump remoto trunca a última coluna** — `_dumpLinha[96]` é pequeno demais
   para a linha de 16 colunas (medido: 98 bytes no caso real, ~141 no pior
   caso). O dump via WebSocket/ESP-NOW sai cortado e o replay descarta a linha.
3. **Dump por serial pode reiniciar o robô** — `recorder.dump()` despeja até
   1700 linhas (~13 s) bloqueando o `loop()`, acima do WDT de 5 s. O teste
   registrado (452 amostras) passou por estar abaixo do limite.
4. **Reancoragem duplicada 6× e já divergente** — o restart por serial não
   reancora o rumo; o segundo quadrado consecutivo herda o rumo do anterior.

**Divergências de média severidade:** comandos seriais do Manual que o código
não aceita (e `g` inicia o quadrado em vez do pivô); instrução de
`ESPNOW_CHANNEL` apontando macro removida; Roteiro ESP-NOW prometendo CSV que a
ponte não grava; contagem de testes do TODO (real: 33 nativos / 15 Python);
três formatos de CSV divergentes (16/17/19 colunas).

**Código morto:** 18 macros/funções sem uso, 3 estados inalcançáveis e
redundâncias (mapeamento estado→string 3×, leituras de sensor 4-6× por ciclo).

**Descartado após verificação:** a suspeita de `snprintf` com 17 especificadores
e 16 argumentos é **falsa** (são 16/16); `abs()` em float está correto no core
3.3.12.

---

---

