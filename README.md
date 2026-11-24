# Plataforma Robótica Diferencial 2WD — ESP32 (Versão 4 Oficial)

Este repositório contém o firmware embarcado, as ferramentas de diagnóstico, monitoramento sem fio e a documentação de engenharia para um robô móvel de tração diferencial (2WD) baseado no microcontrolador **ESP32-D0WD-V3**.

O sistema executa com extrema repetibilidade e controle em malha fechada uma trajetória de teste quadrada fechada (500 mm por lado com curvas de 90° em pivô central orientadas por IMU), agora com **telemetria bidirecional em tempo real via Wi-Fi/WebSocket**, **dashboard web embarcado** e **ponte ESP-NOW para ESP32**.

---

> [!IMPORTANT]
> **Versão Oficial Homologada: `firmware_robo2wd_v4/` (Versão 4)**  
> As implementações anteriores (V1 por software, V2 bloqueante e V3 com fio/serial) foram preservadas na pasta [`legacy/`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/legacy/) para histórico de engenharia.

---

## 🚀 Destaques da Versão 4 (Oficial)

* **Conectividade Sem Fio e Telemetria em Tempo Real:**
  * **Wi-Fi Station & WebSocket Server (telemetria adaptativa: 20 Hz em movimento / 2 Hz parado):** Transmissão contínua de telemetria completa (RPM alvo/real, pulsos de encoders, ângulo de Yaw da IMU, comandos PWM e estados da FSM).
  * **Dashboard Web Embarcado (PROGMEM):** Acessível diretamente pelo navegador via `http://robo.local/` ou pelo IP local, com gráficos em tempo real (Chart.js), visualização de estado, disparo remoto de trajetória e exportação direta para CSV.
  * **Ponte ESP-NOW (gateway ESP32):** Firmware dedicado em `gateway_espnow/` para operação transparente via rádio 2.4 GHz em campo ou bancada sem necessidade de roteador Wi-Fi.
  * **Monitor e Logger Python:** Utilitário assíncrono em `robo_monitor/` para gravação e análise automatizada de corridas.
* **Fusão Sensorial e Orientação Absoluta (IMU Bosch BMI160):**
  * Correção giroscópica contínua em retas (`IMU_KP_YAW_RETA = 2.0`) reduzindo o desvio angular para menos de 1°.
  * Parada de pivô por delta de Yaw com **antecipação calibrada**: `PIVO_ANTECIPACAO_GRAUS = 5.7°` e `VELOCIDADE_GIRO_RPM = 25`. O corte ocorre em ~84° e a **inércia de frenagem** (~5°) completa os ~90° — sem derrapagem nem overshoot.
* **Diagnóstico de Rumo e Odometria:**
  * Rumo por odometria (`g_yawOdom`), divergência IMU × encoders (`diverg`) e **autoteste de bias do giroscópio no boot**.
  * Telemetria com `yawReal` (corte + inércia), `inercia` e `inerciaOdom`.
  * Comandos de bancada no dashboard: `📏 Reta 500` e `🔄 Pivô 90`.
* **Compensação Dinâmica de Zona Morta e Atrito:**
  * Feedforward proporcional à velocidade (curva PWM×RPM medida em bancada, corrigida por `CURVA_FATOR_CARGA = 0.80`), eliminando tempo de estol na partida e entregando pulso adequado em qualquer velocidade alvo.
* **Periféricos Nativos de Silício (ESP-IDF):**
  * **MCPWM (Motor Control PWM):** Timer de 1 MHz dedicado gerando pulsos para ESC a 50 Hz com resolução de $1\,\mu\text{s}$.
  * **PCNT (Pulse Counter):** Decodificação por hardware de quadratura 4x com filtro digital de 1000 ns, desonerando 100% da CPU.
* **Máquina de Estados Não-Bloqueante (FSM):**
  * Loop orientado a tempo (`millis()`) com Task Watchdog Timer por hardware ativo (5s).

---

## 📁 Estrutura de Diretórios

```
Teste_car_0/
├── firmware_robo2wd_v4/       # [OFICIAL] Código-fonte ativo da Versão 4
│   ├── config.h               # Cinemática, ganhos, limites, Wi-Fi e portas de rede
│   ├── secrets.h              # Credenciais de rede local (ignorado pelo git)
│   ├── wifi_telemetry.h       # Driver Wi-Fi, servidor WebSocket e Dashboard HTML PROGMEM
│   ├── motor_control_mcpwm.h  # Driver MCPWM (1 MHz, 50 Hz, 1 us)
│   ├── encoder_reader_pcnt.h  # Driver PCNT (Quadratura 4x + Filtro EMA)
│   ├── imu_bmi160.h           # Driver I2C nativo para IMU Bosch BMI160
│   ├── pid_controller.h       # PID com feedforward por velocidade e Anti-Windup
│   ├── telemetry_recorder.h   # Gravador de telemetria em RAM (dump serial/WebSocket)
│   └── firmware_robo2wd_v4.ino# FSM não-bloqueante, partida remota e telemetria 5Hz
├── gateway_espnow/            # [GATEWAY] Firmware do gateway ESP32 (Ponte ESP-NOW <-> USB)
│   └── gateway_espnow.ino
├── robo_monitor/              # [FERRAMENTAS PC] Scripts de recepção e análise de dados
│   ├── robo_ws_logger.py      # Logger WebSocket assíncrono para terminal e CSV (+ --replay)
│   ├── capturar_dump.py       # Captura o dump do gravador em RAM via serial
│   ├── capturar_dump_wifi.py  # Captura o dump do gravador em RAM via WebSocket
│   ├── gateway_espnow.py      # Ponte serial do gateway ESP32 (ESP-NOW <-> PC)
│   ├── requirements.txt       # Dependências Python (websockets, rich, pyserial)
│   └── logs/                  # Diretório de armazenamento de corridas em CSV
├── legacy/                    # [LEGADO] Versões históricas preservadas
│   ├── firmware_robo2wd/      # Versão 1: LEDC + Interrupções de software (ISRs)
│   ├── firmware_robo2wd_v2/   # Versão 2: MCPWM + PCNT em loops bloqueantes
│   └── firmware_robo2wd_v3/   # Versão 3: FSM + Perfil Trapezoidal + IMU (com fio/serial)
├── diag_pcnt/                 # Utilitário de diagnóstico e calibração de encoders em hardware
├── diag_i2c_bmi160/           # Utilitário de diagnóstico I2C e leitura da IMU BMI160
├── calib_imu_polaridade/      # Ferramenta de validação da polaridade do eixo Z da IMU
├── diag_motor_curve/          # Utilitário de bancada: curva PWM x RPM dos motores
├── diag_encoders/             # [LEGADO V1] Utilitário de teste de encoders (pinout GPIO 23/22)
└── docs/                      # Documentação técnica e de engenharia
    ├── FSD.md                 # Especificação Funcional Definitiva (v4.0.0 em PT-BR)
    ├── MANUAL_TECNICO.md      # Manual completo de pinagem, cinemática, operação e compilação
    ├── ROTEIRO_ESPNOW.md      # Validação da ponte robô <-> gateway ESP32
    ├── REVISAO_ENGENHARIA.md  # Auditoria de código × doc e plano de melhorias
    └── DIARIO_ENGENHARIA.md   # Registro cronológico detalhado da evolução do projeto
```

---

## 📌 Mapeamento de Pinos (GPIO)

| Pino ESP32 | Periférico | Função Física |
| :---: | :---: | :--- |
| **GPIO 19** | MCPWM Op1 | Sinal do ESC Motor Esquerdo (Canal 1) |
| **GPIO 18** | MCPWM Op2 | Sinal do ESC Motor Direito (Canal 2) |
| **GPIO 16** | PCNT Unit0 | Fase A do Encoder Esquerdo (mesmo lado da placa) |
| **GPIO 17** | PCNT Unit0 | Fase B do Encoder Esquerdo (mesmo lado da placa) |
| **GPIO 33** | PCNT Unit1 | Fase A do Encoder Direito |
| **GPIO 32** | PCNT Unit1 | Fase B do Encoder Direito |
| **GPIO 21** | I2C SDA | **USADO** – SDA da IMU BMI160 |
| **GPIO 22** | I2C SCL | **USADO** – SCL da IMU BMI160 |
| **GPIO 23** | GPIO | **LIVRE** para uso geral / interrupção |

---

## 📖 Documentação Completa

* 📄 **[Manual Técnico e de Operação](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/docs/MANUAL_TECNICO.md)**
* 📄 **[Especificação Funcional (FSD v4.0)](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/docs/FSD.md)**
* 📄 **[Roteiro de Validação do Enlace ESP-NOW](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/docs/ROTEIRO_ESPNOW.md)**
* 📄 **[Diário de Engenharia](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/docs/DIARIO_ENGENHARIA.md)**
* 📄 **[Revisão de Engenharia](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/docs/REVISAO_ENGENHARIA.md)** — auditoria de código × documentação e plano de melhorias

## 📄 Atualizações Recentes (24/11/2026)

### Firmware
- **Rede fora do laço de controle:** a publicação de telemetria (WebSocket e ESP-NOW) passou a rodar em uma tarefa dedicada no core 0; o laço apenas enfileira pacotes, sem esperar o enlace. Essa era a causa da perda de trajetória — a escrita síncrona no socket TCP bloqueava ~1 s quando o peer não drenava, com os motores no último PWM.
- **Diagnóstico de travamento:** a telemetria ganhou `lm` (maior intervalo entre iterações do laço) e `sr`/`si` (pior bloco e qual bloco). Foram esses campos que localizaram a causa.
- **IMU BMI160:** barramento I2C de 400 kHz → **100 kHz** e `Wire.setTimeOut(5)` — o driver usa timeout padrão de 1000 ms por transação, e os motores escovados injetam EMI no barramento.
- **Resultado no solo:** 4 quadrados consecutivos com `lm` de 11–12 ms, fechamento de **6,4–21,4 mm** e erro de rumo de até 2,2°. Um dos testes teve um buraco de 1750 ms na telemetria sem qualquer reflexo na trajetória.
- **Precisão depende da carga da LiPo:** 4 pivôs somaram 360,96° a 8,20 V contra 356,58° a 7,46 V.

### Testes
- 16 + 17 + 9 testes nativos, 18 Python, CI verde.

## 📄 Atualizações Recentes (19/11/2026)

### Firmware
- **Rumo real com inércia:** `yawReal` = `g_yawAbs + g_inerciaTotal` — os cortes **mais** a inércia de frenagem de **todos** os pivôs. O valor anterior congelava no corte e escondia ~5–9° por pivô, e o `yawReal` somava só a inércia do último segmento (~14° errado no fim).
- **Pivô recalibrado em solo:** `VELOCIDADE_GIRO_RPM` 35 → **25 RPM** e `PIVO_ANTECIPACAO_GRAUS` 0,7° → **5,7°**. Pivôs medidos em 88,2–90,7° (média ≈ 89,7°).
- **Diagnóstico de rumo:** `g_yawOdom` (odometria com sinal), `diverg` (IMU × encoders), `gyroBias` com autoteste no boot, e acumuladores separados de inércia (`inercia`, `inerciaOdom`).
- **Comandos de bancada no dashboard:** `📏 Reta 500` (`t`) e `🔄 Pivô 90` (`g`).
- **Wi-Fi:** reconexão automática no `loop()` — antes `g_wifiOk` era setado uma única vez no `setup()` e uma queda deixava o dashboard "Desconectado".
- **Gravador:** 1700 amostras × 40 bytes (85 s) — as colunas de diagnóstico estouraram o DRAM em 1984.

### Resultados no solo
- **Quadrado:** fechamento de **7,8–8,7 mm**, lados de 503–504 mm pelos ticks, erro de direção de **−1,3° a −2,4°**.
- **Mapa do dashboard:** rastro desenhado com o **rumo real**; o quadrado ideal fecha em ~7 mm e o traçado coincide.

### Testes
- 16 + 17 testes nativos, 15 Python (inclui validação de sintaxe do JS do dashboard com `node --check`).

## 📄 Atualizações Recentes (16/11/2026)

### Firmware
- **Feedforward por velocidade:** o PID interpola a curva PWM×RPM medida em bancada (`diag_motor_curve`) e corrige por `CURVA_FATOR_CARGA = 0.80` (carga no solo). Substitui o feedforward fixo de 110 µs; `PID_KI = 1.5`.
- **Pivô calibrado em solo:** antecipação `PIVO_ANTECIPACAO_GRAUS = 0.7°` (corte em 89,3°). Resultado: 90,29° ± 0,54°, contra 89,7–91,0° do ajuste anterior.
- **Gravador de telemetria em RAM** (`telemetry_recorder.h`): 2000 amostras a 20 Hz (100 s), com dump por Serial (`d`) ou WebSocket (`dump`). Permite testar no solo sem cabo serial.
- **Comandos remotos centralizados:** `start`/`stop`/`pause`/`reset`/`dump` em um único ponto do `loop()` — a flag `_newCommand` antes era consumida por blocos concorrentes e descartava comandos.
- **Robustez:** timeout automático de 5 s desativado quando o Wi-Fi está conectado; estado final repetido gravado no máximo 1×/2 s para não descartar o teste do buffer circular.

### Ferramentas PC
- `robo_monitor/capturar_dump_wifi.py` — captura o dump do gravador via WebSocket, com reconexão automática.
- `robo_monitor/capturar_dump.py` — captura via serial.
- `robo_monitor/robo_ws_logger.py --replay dump.txt --saida out.csv` — converte o dump em CSV.
- `diag_motor_curve/` com Wi-Fi: varredura não-bloqueante, buffer em RAM e página web (`sweep`/`csv`/`clear`).
- **ESP-NOW no robô (`espnow_link.h`)**: telemetria JSON a 5 Hz ao gateway ESP32 e recepção dos comandos `start`/`stop`/`pause`/`reset`/`dump`. Compatível com ESP-IDF v4/v5. `robo_monitor/gateway_espnow.py` é a ponte serial do gateway.

## 📄 Atualizações Recentes (13/11/2026)

### Firmware
- **Origem da odometria:** ponto inicial fixado em **(500 mm, 0)**.
- **Ângulos de base específicos por lado:** cálculo de `baseDeg` agora depende do lado da roda, corrigindo a direção esperada.
- **Correção de sinal da seta de orientação:** durante pivôs, a seta agora usa `curHeadingDeg = baseDeg - d.yaw` (antes `+`).
- **Ganho de controle reforçado:** `K_SYNC = 0.15` e `IMU_KP_YAW_RETA = 2.0` para reduzir desvio em retas; pivô com antecipação calibrada (`PIVO_ANTECIPACAO_GRAUS = 0.7°`) e roda calibrada em 66 mm.
- **Clamping de PWM ampliado:** agora ±50 µs (antes ±30 µs).

### Dashboard Web
- **Desenho do quadrado ideal:** agora utiliza as coordenadas reais (500 mm × 500 mm) e segue a sequência **Início → L1 → L2 → L3 → Início**.
- **Rastro de trajetória real:** renderizado em linha verde usando odometria cumulativa, refletindo desvios reais.
- **Indicador de pose:** ponto + vetor direcional com valores de **X, Y (mm)** e **θ (°)** exibidos numericamente.
- **Seta de direção:** orientada corretamente durante pivôs, correspondendo ao movimento real.

Para mais detalhes, veja o [Diário de Engenharia](docs/DIARIO_ENGENHARIA.md).