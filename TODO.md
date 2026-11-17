# TODO — Projeto Robô 2WD (Teste_car_0)

Revisão completa do estado do projeto em **16/11/2026**, após o commit `fa01d2a`
(feedforward por velocidade, gravador em RAM, calibração no solo).

Legenda de prioridade: **[P1]** bloqueia ou degrada a operação · **[P2]** qualidade/robustez · **[P3]** higiene/evolução.

---

## 1. Estado Atual (resumo)

- **Firmware oficial:** `firmware_robo2wd_v4/` (v4.0.0). Compila: 1.042.211 bytes (79%) flash, 116.648 bytes (35%) RAM.
- **Último teste em solo válido** (robô posicionado corretamente):

  | Métrica | Valor | Alvo |
  | :--- | :---: | :---: |
  | Pico de RPM nas retas | 58,4 | 55 |
  | PWM máximo | 1744 µs | — |
  | Tempo por reta | 3,75 s | — |
  | Yaw médio do pivô | 90,29° | 90,0° |
  | Desvio padrão do pivô | 0,54° | — |
  | Yaw final das retas | −0,70° a +0,26° | 0° |
  | Ticks por reta | 10.446–10.516 | 10.417 |

- **Working tree:** limpo, exceto `diag_motor_curve/gráfico.png` (artefato não rastreado).
- **Branch:** `main` sincronizado com `origin/main` até `fa01d2a`.

---

## 2. Problemas em Aberto (por prioridade)

### [P1] Funcionalidades declaradas mas não implementadas

1. ~~**ESP-NOW: lado do robô não existe.**~~ **ATENDIDO E VALIDADO (16/11/2026).**
   - Implementado em `espnow_link.h`: telemetria JSON ao gateway ESP32-S3 (5 Hz) e recepção de `start`/`stop`/`pause`/`reset`/`dump`.
   - Comandos aceitos por WebSocket **e** ESP-NOW, com dispatch centralizado; o dump é devolvido pelo canal que recebeu o pedido.
   - Adicionada a ponte `robo_monitor/gateway_espnow.py` (serial do gateway, grava CSV).
   - **Validado de ponta a ponta** com gateway ESP32 clássico (COM18): robô→gateway 1472 pacotes/40 s; `CMD:start` confirmado e robô executou; dump devolvido com 443 amostras (RETA→CONCLUIDO). Números do dump idênticos aos do Wi-Fi (pico 58,8 RPM, pivôs 89,6–90,7°, desvio 0,40°).
   - **Lição:** o gateway prefixa *todo* payload com `TELEM:`, inclusive o dump — as linhas chegam como `TELEM:REC,...`. Filtros por `startswith("REC,")` retornam 0. A ponte agora distingue os dois formatos.
   - **Restrição:** gateway e robô precisam de canal coincidente. O canal do robô é o do AP (medido: 11); o gateway fixa o canal (`ESPNOW_CHANNEL`). O robô registra o peer com canal 0.
   - **Armadilha:** `esp_wifi_set_channel()` após o connect Wi-Fi derruba o enlace e trava o `setup()` — não usar.

2. **Monitor de bateria não implementado.**
   - `PIN_BATTERY_ADC = -1` (desabilitado) e nenhuma leitura de tensão no firmware.
   - Sem proteção contra LiPo 2S profunda: risco real de dano à bateria e de o robô parar no meio do percurso.
   - **Adiado por decisão do usuário (16/11/2026).** Projeto já levantado: divisor R1=100k/R2=47k em GPIO 34 (ADC1_CH6), `ADC_11db`, fundo de escala ~2,5 V, limiares 7,0 V (atenção) / 6,6 V (corte, 3,30 V/célula), telemetria `vbat` + corte em `ESTADO_EMERGENCIA`. Pendência: divisor não está montado; calibração com multímetro é obrigatória.

3. **Sem proteção de baixa tensão / sem telemetria de corrente.**
   - Relacionado ao item anterior. O ESC é brushless de 2-3S; operar abaixo de ~6,4 V danifica a célula.

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

6. ~~**Sem testes automatizados.**~~ **PARCIALMENTE ATENDIDO (17/11/2026).**
   - Criado o diretório `tests/` com testes que rodam **sem hardware**:
     - `tests/native/test_cinematica.cpp` — 12 testes sobre os headers **reais** do firmware (a única dependência de Arduino é suprida pelo stub `tests/native/Arduino.h`): `MM_POR_PULSO`, ticks por reta de 500 mm (10417), ticks do pivô de 90° (1963), interpolação da curva PWM×RPM e o feedforward de 55 RPM sob carga. Roda com `python tests/native/rodar.py` (escolhe `CXX`, `ziglang` ou `g++`/`clang++`).
     - `tests/python/test_ferramentas.py` — 11 testes: replay do dump para CSV (colunas, valores, contagem) e consistência de formato entre dump, logger, gateway e dashboard. É a classe de bug que já quebrou de verdade (o replay falhou quando as colunas mudaram 12→13→14).
   - **CI:** `.github/workflows/ci.yml` com dois jobs — `testes` (Python + nativo) e `build` (compila o firmware do robô e o gateway via `arduino/compile-sketches`).
   - Para viabilizar os testes nativos, `ticksParaMm()` e `ticksParaArcoPivo()` foram extraídas para `firmware_robo2wd_v4/robo_matematica.h` — **as fórmulas são idênticas** e o binário resultante tem o mesmo tamanho (1.123.859 bytes).
   - **O que falta para fechar o item:** testes do critério de corte do pivô (extrair `atingiuMetaPivo()`), do perfil trapezoidal e do `yawParaRumo()` do dashboard (JS). Os dois últimos exigem mais refactor; o do pivô é o mais valioso e o mais sensível (mexe no laço de controle).

7. ~~**Telemetria de 5 Hz é pouco para análise dinâmica.**~~ **ATENDIDO (16/11/2026).**
   - Implementada **telemetria adaptativa**: 20 Hz durante `RETA`/`PIVO`, 2 Hz em `AGUARDO`/`CONCLUIDO` (WebSocket e ESP-NOW).
   - Medido em bancada: **20,1 Hz em movimento, 2,2 Hz em repouso** (alvos 20/2 Hz).
   - Validação do impacto no controle: pivôs com desvio 0,40° → **0,21°** e amplitude 1,09° → **0,58°**; RPM 58,8 → 57,9; tempo de reta 3,70 → 3,80 s. Controle não degradou.
   - O gravador em RAM (20 Hz) já cobria a análise profunda; o ganho está no dashboard/logger em tempo real (~76 amostras por reta, contra ~19 antes).
   - Risco residual: o envio WebSocket é bloqueante; se o número de clientes crescer, o custo por envio multiplica. Com 1 cliente, ~3% do tempo.

### [P3] Higiene de código e documentação

8. **Comentários desatualizados em `config.h`.**
   - Linha 93: `VELOCIDADE_CRUZEIRO_RPM 55.0f // ... ~40 RPM real ...` → o real medido é ~58 RPM.
   - Linha 99: `RAMPA_ACEL_DIST_MM 50.0f // Acelera de 20 a 80 RPM` → o cruzeiro é 55 RPM, não 80.

9. **Macro `IMU_HABILITADA` é código morto.**
   - Definida em `config.h:77` como `true`, mas **nunca referenciada** em nenhum arquivo. O código usa `imu.isInitialized()` diretamente.

10. **`PID_KI` com comentário histórico no valor.**
    - `config.h:62`: o comentário explica "0.6 -> 1.5" — o valor vigente está correto (1.5), mas o comentário virou ruído.

11. **FSD ainda cita "~40 RPM real em solo".**
    - `docs/FSD.md:27`: texto de 13/11 não atualizado com o resultado de 16/11 (~58 RPM).

12. **Manual Técnico não documenta o gravador em RAM nem as ferramentas de dump.**
    - Não há seção sobre `telemetry_recorder.h`, o comando `dump`, `capturar_dump.py`, `capturar_dump_wifi.py` ou o `--replay`.
    - A seção 5 (Operação) menciona START/STOP e DOWNLOAD CSV, mas não o "Dump RAM" nem o teste no solo sem cabo.
    - A seção 5.2 ainda diz "aguarda 5s para timeout" — esse comportamento foi desativado quando o Wi-Fi está conectado.

13. **Diário com referências antigas a 89,5°.**
    - Etapas 9–11 citam "corte em 89,5°" (valor vigente: 89,3° com `PIVO_ANTECIPACAO_GRAUS`).
    - Como é um registro cronológico, pode ser intencional — mas convém uma nota de rodapé apontando o valor atual.

14. **`diag_motor_curve/secrets.h` existe no disco e está ignorado pelo `.gitignore`.**
    - Correto do ponto de vista de segurança (`**/secrets.h`), mas o sketch não tem um `secrets.h.example` próprio (o firmware V4 tem).
    - Sem o example, outro usuário não sabe o que configurar para compilar o diagnóstico.

15. **Artefato órfão `diag_motor_curve/gráfico.png`.**
    - Não rastreado, não ignorado. Ou entra no repositório (documentação da curva) ou vai para o `.gitignore`.

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

---

## 4. Sugestão de Ordem de Trabalho

1. ~~**[P1]** Decidir o destino do ESP-NOW~~ — **atendido**: implementado o lado do robô (`espnow_link.h`) + ponte `gateway_espnow.py`. Resta validar com o ESP32-S3 e, se desejado, trocar broadcast por MAC dedicado.
2. **[P1]** Monitor de bateria (hardware + `PIN_BATTERY_ADC` + alerta/telemetria).
3. ~~**[P2]** Chart.js local (baixar o arquivo para PROGMEM ou servir do SPIFFS).~~ **ATENDIDO (17/11/2026)** — servido de `/chartjs.min.js` pelo próprio ESP32 (gzip em PROGMEM).
4. ~~**[P2]** Mapa 2D com orientação real medida (remover `baseDeg` nominal).~~ **ATENDIDO (17/11/2026)** — ver item 5 dos problemas de fundo.
5. ~~**[P2]** Testes automatizados: ao menos um teste de regressão da interpolação da curva e do critério de corte do pivô, com CI.~~ **PARCIALMENTE ATENDIDO (17/11/2026)** — CI criada e testes da curva/cinemática/ferramentas passando; falta o teste do critério de corte do pivô.
6. **[P3]** Higiene: comentários do `config.h`, remover `IMU_HABILITADA`, atualizar FSD:27 e a seção 5 do Manual, `secrets.h.example` para o `diag_motor_curve`.
7. **[P3]** Decidir o destino de `diag_motor_curve/gráfico.png`.

---

## 5. Verificações Realizadas nesta Revisão

- **Código:** ausência de `esp_now` no firmware do robô confirmada por grep; `IMU_HABILITADA` sem referências; `PIN_BATTERY_ADC = -1` sem uso.
- **Documentação:** valores confrontados entre `config.h`, FSD, Manual, Diário e README (itens 8–13).
- **Git:** `git status` limpo exceto o PNG; `git ls-files` confirma que apenas `secrets.h.example` está rastreado (sem vazamento de credenciais).
- **Compilação:** 1.042.211 bytes (79%) flash, 116.648 bytes (35%) RAM.
- **Funcional:** replay do dump testado (452 amostras → CSV com cabeçalho correto).
