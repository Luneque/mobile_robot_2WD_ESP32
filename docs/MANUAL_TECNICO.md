# Manual de Engenharia e Operação — Robô Diferencial 2WD com ESP32

---

> [!IMPORTANT]
> **Versão Oficial Atual: Versão 4 (`firmware_robo2wd_v4/`)**  
> As implementações anteriores (Versão 1 por software, Versão 2 bloqueante e Versão 3 com fio/serial) foram arquivadas no diretório [`legacy/`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/legacy/) para histórico de engenharia e rastreabilidade técnica.

---

## 1. Apresentação do Projeto

Este projeto consiste em uma plataforma robótica móvel de tração diferencial (2WD) baseada no microcontrolador **ESP32-D0WD-V3**. O sistema executa trajetórias de alta precisão com realimentação em malha fechada (PID), perfil de velocidade trapezoidal, compensação de zona morta por feedforward e sincronismo dinâmico entre rodas (*cross-coupling*), integrado com uma IMU Bosch BMI160.

A versão oficial do firmware (**Versão 4**) opera com:
* **Conectividade Sem Fio e Telemetria em Tempo Real:** Wi-Fi Station, WebSocket Server com telemetria adaptativa (20 Hz em movimento / 2 Hz parado) e Dashboard Web embarcado em memória flash (servido em `http://robo.local/`), permitindo visualização de gráficos de alta frequência e disparo remoto sem cabos.
* **Rede Fora do Laço de Controle:** a publicação de telemetria (WebSocket e ESP-NOW) roda em uma tarefa dedicada no core 0; o laço de controle apenas enfileira pacotes (`agendarTelemetria()`, sem espera) e nunca bloqueia no enlace. Se a fila encher, amostras de monitoração são descartadas — nunca se espera pelo peer. Um mutex serializa o acesso ao servidor Web/WebSocket.
* **Gateway ESP-NOW:** Suporte a um microcontrolador auxiliar ESP32 para transmissão de telemetria sem necessidade de ponto de acesso Wi-Fi.
* **Periféricos de Silício Dedicados:** **MCPWM** (50 Hz / resolução de $1\,\mu\text{s}$) e **PCNT** (decodificação de quadratura 4x por hardware com filtro anti-glitch de 1000 ns).
* **Compensação Dinâmica de Zona Morta:** Feedforward proporcional à velocidade, derivado da curva PWM×RPM medida em bancada e corrigido pelo fator de carga do robô no solo (`CURVA_FATOR_CARGA = 0.80`), vencendo o atrito estático já na partida.
* **Fusão Sensorial com IMU BMI160:** Correção contínua de rumo em linha reta (`IMU_KP_YAW_RETA = 2.0`) e controle estrito de parada de giro pivô por delta de yaw (corte em ~86° com antecipação de 3,7°, `VELOCIDADE_GIRO_RPM = 20` e piso de rampa `VELOCIDADE_MIN_PIVO_RPM = 12`).
* **Antecipação de parada:** tanto retas quanto pivôs cortam antes do alvo para acomodar a inércia de frenagem — `RETA_ANTECIPACAO_MM = 20` (alvo de 480 mm para parar em 500) e `PIVO_ANTECIPACAO_GRAUS = 3,7°`. A rampa de desaceleração de cada segmento usa o alvo antecipado.

---

## 2. Especificação e Conexões de Hardware

### 2.1 Componentes do Sistema
1. **Placa Controladora:** ESP32-D0WD-V3 acoplado à placa de expansão **Goouuu 38P / V4** com alimentação regulada.
2. **Atuadores:** 2 Micromotores DC com redução metálica homologada de **1:90**.
3. **Sensores de Velocidade:** Encoders magnéticos de efeito Hall (disco magnético de **12 PPR** por canal).
4. **Acionamento de Potência:** ESC escovado duplo bidirecional configurado no modo independente (*Mixed Control = OFF*).
5. **Alimentação:** Bateria LiPo 2S (~7,4 V), monitorada por divisor R1=100k/R2=47k com capacitor de 100 nF no GPIO 34 (ADC1_CH6). Limiares: 7,0 V (atenção) e 6,6 V (corte, 3,30 V/célula).

### 2.2 Diagrama de Pinagem e Conexões Físicas

```
                          ┌──────────────────────────┐
                          │       ESP32 WROOM        │
                          │   (Placa Goouuu 38P)     │
                          └─────────────┬────────────┘
                                        │
           ┌────────────────────────────┼────────────────────────────┐
           │                            │                            │
     [ MCPWM SAÍDAS ]             [ ENCODER 1 (ESQ) ]          [ ENCODER 2 (DIR) ]
   GPIO 19 ──► ESC Sinal CH1    GPIO 16 ◄── Fase A           GPIO 33 ◄── Fase A
   GPIO 18 ──► ESC Sinal CH2    GPIO 17 ◄── Fase B           GPIO 32 ◄── Fase B
                                (Pull-up interno +           (Pull-up interno +
     [ I2C DISPONÍVEL ]          Filtro PCNT 1000ns)          Filtro PCNT 1000ns)
   GPIO 21 ──► I2C SDA
   GPIO 22 ──► I2C SCL
```

| Sinal | Pino GPIO | Tipo de Periférico | Função |
| :--- | :---: | :---: | :--- |
| **ESC Motor Esquerdo (CH1)** | `GPIO 19` | MCPWM (Timer 0, Gen 1) | Sinal de velocidade/sentido do motor esquerdo |
| **ESC Motor Direito (CH2)** | `GPIO 18` | MCPWM (Timer 0, Gen 2) | Sinal de velocidade/sentido do motor direito |
| **Encoder Esquerdo — Fase A** | `GPIO 16` | PCNT (Unidade 0, Canal A) | Quadratura 4x do motor esquerdo (mesmo lado da placa) |
| **Encoder Esquerdo — Fase B** | `GPIO 17` | PCNT (Unidade 0, Canal B) | Quadratura 4x do motor esquerdo (mesmo lado da placa) |
| **Encoder Direito — Fase A** | `GPIO 33` | PCNT (Unidade 1, Canal A) | Quadratura 4x do motor direito |
| **Encoder Direito — Fase B** | `GPIO 32` | PCNT (Unidade 1, Canal B) | Quadratura 4x do motor direito |
| **I2C — SDA (Dados)** | `GPIO 21` | I2C Padrão Hardware | **USADO** – SDA da IMU BMI160 |
| **I2C — SCL (Clock)** | `GPIO 22` | I2C Padrão Hardware | **USADO** – SCL da IMU BMI160 |
| **GPIO Geral** | `GPIO 23` | GPIO Universal | **LIVRE** (ex: pino INT de IMU ou trigger) |

### 4.5 Sensores de Inércia – IMU BMI160
- **GPIO 21 (SDA)** e **GPIO 22 (SCL)** conectados ao sensor BMI160 (giroscópio + acelerômetro).
- **GPIO 23** pode ser usado como sinal de interrupção **INT** da IMU (opcional).
- O driver `imu_bmi160.h` fornece leitura de ângulo de guinada (yaw) e calibração de polaridade.


---

## 3. Modelo Cinemático e Calibração

### 3.1 Resolução dos Encoders (Quadratura 4x)
$$\text{PPR}_{\text{efetivo}} = 12 \text{ PPR} \times 90 \text{ (Redução)} \times 4\text{x} = \mathbf{4.320 \text{ pulsos/volta}}$$

### 3.2 Deslocamento Linear e Angular
* **Circunferência da Roda ($66\text{ mm}$ de diâmetro real):** $\pi \times 66 \approx \mathbf{207,345\text{ mm}}$
* **Resolução Linear ($K_{\text{dist}}$):** $\frac{207,345\text{ mm}}{4.320\text{ pulsos}} \approx \mathbf{0,047997\text{ mm/pulso}}$ ($\approx 10.417\text{ pulsos}$ por reta de 500 mm)
* **Bitola entre Rodas ($L$):** $120,0\text{ mm}$
* **Arco de Giro para Curva de 90°:** $\frac{\pi \times 120 \times 90^\circ}{360^\circ} \approx \mathbf{94,248\text{ mm}}$ ($\approx \mathbf{1.964\text{ pulsos}}$ por roda)

---

## 4. Arquitetura de Software da Versão 4 (Oficial)

A estrutura oficial reside em [`firmware_robo2wd_v4/`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v4/):

### 4.1 Módulos do Sistema
* **[`config.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v4/config.h):** Centraliza pinos, limites de pulso do ESC ($1150\,\mu\text{s}$ a $1850\,\mu\text{s}$), parâmetros cinemáticos (roda de 66 mm), distâncias de rampa, parâmetros do WDT e ganhos de fusão com a IMU (`IMU_KP_YAW_RETA = 2.0`) e sincronismo cruzado (`K_SYNC = 0.15`).
* **[`wifi_telemetry.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v4/wifi_telemetry.h):** Driver de conectividade Wi-Fi Station, servidor WebSocket (porta 81) e servidor HTTP (porta 80). Contém o Dashboard Web embarcado em PROGMEM com gráficos Chart.js e visualizador 2D da trajetória em tempo real (HTML5 Canvas).
* **[`imu_bmi160.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v4/imu_bmi160.h):** Driver nativo I2C (100 kHz — mais tolerante a EMI dos motores que 400 kHz; o driver usa `Wire.setTimeOut(5)` para uma transação perdida não travar o laço) para a IMU Bosch BMI160. Fornece calibração estática de *bias* do giroscópio, leitura direta do eixo Z (giro horário = Yaw positivo) e integração de ângulo de guinada (*Yaw*). O firmware usa o delta real de yaw (`deltaYaw = yawAtual − g_yawStartSeg`, capturado no início de cada segmento) para correção de rumo em retas (`IMU_KP_YAW_RETA = 2.0`). No pivô, a IMU atua pelo critério de parada (`|deltaYaw| ≥ (ânguloAlvo − PIVO_ANTECIPACAO_GRAUS)`, corte em ~85° com antecipação calibrada de 5,7°) com guarda mínima de 200 ticks (`MIN_TICKS_PIVO`) para evitar disparos prematuros por resíduo de yaw; o sincronismo cruzado é feito apenas pelos encoders. O acelerômetro é inicializado pelo driver (ODR 100 Hz, ±2 g), mas não é lido pelo firmware — apenas o utilitário `diag_i2c_bmi160` faz a leitura.
* **[`motor_control_mcpwm.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v4/motor_control_mcpwm.h):** Gerador determinístico de PWM a 50 Hz com resolução de $1\,\mu\text{s}$ via timer de 1 MHz.
* **[`encoder_reader_pcnt.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v4/encoder_reader_pcnt.h):** Decodificador de pulsos com filtro anti-glitch de 1000 ns e filtro exponencial de velocidade (EMA).
* **[`pid_controller.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v4/pid_controller.h):** Controlador PID com anti-windup e feedforward proporcional à velocidade, obtido por interpolação da curva PWM×RPM medida em bancada (`diag_motor_curve`) e corrigido pelo fator de carga (`CURVA_FATOR_CARGA`).
* **`telemetry_recorder.h`:** Gravador de telemetria em RAM (1700 amostras a 20 Hz, 85 s, 40 bytes por amostra), com dump por Serial (`d`) ou via WebSocket/ESP-NOW (`dump`). Permite testar o robô no solo sem cabo.
* **`espnow_link.h`:** Enlace ESP-NOW com o gateway ESP32. Envia a telemetria em JSON (20 Hz em movimento / 2 Hz parado) e recebe comandos de texto. Compatível com ESP-IDF v4 e v5. O dump do gravador é devolvido pelo canal que recebeu o pedido.
* **`firmware_robo2wd_v4.ino`:** Máquina de estados não-bloqueante (`ESTADO_AGUARDO_INICIO`, `ESTADO_EXECUTANDO_TRAJETORIA`, `ESTADO_CONCLUIDO`, `ESTADO_EMERGENCIA`), perfil trapezoidal, fusão sensorial IMU + Encoders e dispatch centralizado de comandos.

### 4.2 Dinâmica do Perfil Trapezoidal
* **Reta (500 mm nominais):**
  * Acelera de 20 RPM até **55 RPM** nos primeiros **50 mm** ($\approx 310\text{ mm/s}^2$, pela geometria da rampa: $v^2 = u^2 + 2as$ com roda de 66 mm).
  * Mantém velocidade de cruzeiro estável pela maior parte da reta.
  * O alvo de ticks é **antecipado em 20 mm** (`RETA_ANTECIPACAO_MM = 20`): a rampa de desaceleração começa nos últimos 60 mm em relação ao alvo de 480 mm (10.000 ticks). O robô corta antes e desliza ~20 mm até parar nos **500 mm nominais**.
  * Tempo total por reta: **$\approx 3{,}0\text{ s}$** medido em solo (reta isolada, 499,5–504,2 mm).
* **Curva Pivô (90°):**
  * Acelera em 10° e desacelera em 15°, com cruzeiro suavizado de **20 RPM** no giro pivô (`VELOCIDADE_GIRO_RPM`). O piso da rampa é `VELOCIDADE_MIN_PIVO_RPM = 12`, separado das retas — com 20/20 a rampa final era plana e o robô não desacelerava antes do corte.
  * Corte em ~86° com **antecipação de 3,7°** (`PIVO_ANTECIPACAO_GRAUS`); a inércia e o pulso de frenagem completam o giro (~92° medidos; ver seção de frenagem ativa).
  * Tempo total por giro: **$\approx 1{,}9\text{ s}$** de movimento + 0,8 s de pausa = **$\approx 2{,}7\text{ s}$** (medido em solo).

---

## 5. Guia Rápido de Operação e Comandos (Versão 4)

### 5.1 Operação Sem Fio pelo Navegador (Recomendado)

> **Requisito de boot:** o robô deve ser ligado com o **corpo parado** e não pode ser tocado nos primeiros segundos. A calibração do giroscópio (`imu.calibrateGyro`) roda na inicialização; se o corpo se mover nessa janela, o bias resultante faz o controlador "acreditar" que está girando — foi observado um boot contaminado que produziu deriva de ~13° por reta e fechamento de 337 mm (contra 5 mm com o boot parado, mesmo firmware).

1. Ligue a alimentação dos motores e do ESP32 na bateria.
2. **Aguarde o boot sem tocar no robô** (~10 s: calibração → Wi-Fi → armamento do ESC).
3. O robô se conectará automaticamente à rede Wi-Fi configurada (`secrets.h`).
4. No PC ou celular na mesma rede, abra:
   * **`http://robo.local/`** (ou o IP atribuído, ex: `http://192.168.1.102/`).
5. O painel exibirá:
   * **Mapa 2D da Trajetória:** Sobreposição do quadrado ideal de 500×500 mm (azul tracejado) com o rastro real do robô (verde contínuo), marcador do robô e vetor de direção atualizados em tempo real (20 Hz no movimento). O rastro é integrado com o **rumo real** (`yawReal` = corte + inércia de frenagem), e não com o rumo congelado no corte — sem isso o traçado sai torto ~5° por pivô. Inclui coordenadas instantâneas $(X, Y)$ em mm e ângulo $\theta$ em graus, além de botão para reset manual do rastro.
   * **Gráficos em Tempo Real:** RPM (Esquerda/Direita), Yaw (graus) e pulsos de PWM ($\mu\text{s}$).
6. Posicione o robô no solo e clique no botão verde **START** para iniciar a trajetória quadrada.
7. A qualquer momento, clique em **⏹ STOP** no dashboard para interromper o movimento (o botão existe desde 23/11/2026; antes só `stop` por WebSocket/ESP-NOW).
8. Ao término, clique em **DOWNLOAD CSV** no dashboard para salvar o registro completo dos dados da corrida.

### 5.2 Operação Local via Cabo Serial (Fallback de Bancada)
1. Conecte o robô via USB (`COM9`) e abra o monitor serial em **115200 baud**.
2. Pela serial o robô aceita **`s`**/**`S`** (inicia o quadrado), **`g`**/**`G`** (também inicia o quadrado — o pivô único fica nos comandos `t`/`g` via WebSocket/ESP-NOW) e **`d`** (**dump** do gravador). Enter também inicia.
   * `t` (reta única), `p` (pause), `x` (stop) e `r` (reset) existem **apenas** por WebSocket/ESP-NOW.
   * O *timeout* automático de 5 s só dispara quando o Wi-Fi **não** está conectado; com Wi-Fi ativo, o início é remoto.

### 5.3 Gravação e Leitura de Telemetria em Solo (sem cabo)
O robô grava telemetria em RAM a 20 Hz (1700 amostras, 85 s) — ideal para teste no solo, onde o cabo não pode ficar conectado.

1. **Iniciar a trajetória:** botão **START** no dashboard, ou `start` via WebSocket/ESP-NOW.
2. **Despejar o buffer:** envie `dump` pelo dashboard, por um dos scripts abaixo, ou `d` na serial.
3. **Capturar (robô sem cabo, via Wi-Fi):**
   ```
   python robo_monitor/capturar_dump_wifi.py --host robo.local --start --espera 45 --saida dump.txt
   ```
4. **Capturar (robô com cabo):**
   ```
   python robo_monitor/capturar_dump.py --port COM9 --start --espera 45 --saida dump.txt
   ```
5. **Converter o dump em CSV:**
   ```
   python robo_monitor/robo_ws_logger.py --replay dump.txt --saida teste.csv
   ```
6. **Atenção:** o buffer é volátil — o dump deve ser feito **antes** de desligar/reiniciar o robô.

#### Formatos de CSV (três esquemas, propósitos diferentes)

| Origem | Colunas | Observações |
| :--- | :--- | :--- |
| Dump do gravador (`d`/`dump`) e *replay* (`robo_ws_logger.py --replay`) | 16 | `timestamp_ms,estado,lado,yaw_deg,yaw_abs_deg,yaw_odom_deg,diverg_deg,gyro_bias_dps,rpm_esq,rpm_dir,ticks_esq,ticks_dir,ticks_alvo,pwm_esq_us,pwm_dir_us,rpm_alvo` |
| Logger WS (`robo_ws_logger.py`) | 16 | mesmo esquema do dump |
| Gateway ESP-NOW (`gateway_espnow.py`) | 17 | acrescenta `total_lados` |
| Dashboard (botão DOWNLOAD CSV) | 19 | acrescenta `totalLados`, `yawReal`, `inercia` |

O **dump do gravador é o registro de referência** do teste no solo; os demais existem para acompanhamento e trazem colunas próprias.

### 5.4 Operação via ESP-NOW (Gateway ESP32)
> **Hardware validado:** um **ESP32 clássico** (ESP-WROOM-32, fqbn `esp32:esp32:esp32`, porta `COM18`). O ESP32-S3 era o plano original, mas a placa apresentou defeito e não chegou a ser usado. O código é de placa genérica e roda em qualquer ESP32.
1. Grave `gateway_espnow/gateway_espnow.ino` no ESP32 auxiliar e anote o MAC impresso no Serial.
2. Ajuste `ESPNOW_CHANNEL` em **`gateway_espnow/gateway_espnow.ino`** — quem fixa o canal é o gateway (o robô usa o canal da interface associada; não existe mais macro de canal no `config.h` do robô).
3. Conecte o gateway ao PC e execute a ponte:
   ```
   python robo_monitor/gateway_espnow.py --port COM18 --csv telemetria_espnow.csv
   ```
4. No prompt da ponte, envie `start`, `stop`, `pause`, `reset` ou `dump`. A telemetria chega em `TELEM:{json}` e é gravada em CSV.

---

## 6. Procedimento de Compilação e Gravação

### 6.1 Compilação da Versão 4
```powershell
& "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn esp32:esp32:esp32 --export-binaries "C:\Users\luneque.junior\Projetos_IA\Teste_car_0\firmware_robo2wd_v4"
```

### 6.2 Gravação via `esptool.py` (fechar o monitor serial antes)
```powershell
& "C:\Users\luneque.junior\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\5.3.1\esptool.exe" --chip esp32 --port COM9 --baud 921600 --before default-reset --after hard-reset write_flash -z --flash-mode dio --flash-freq 80m --flash-size 4MB 0x1000 "C:\Users\luneque.junior\Projetos_IA\Teste_car_0\firmware_robo2wd_v4\build\esp32.esp32.esp32\firmware_robo2wd_v4.ino.bootloader.bin" 0x8000 "C:\Users\luneque.junior\Projetos_IA\Teste_car_0\firmware_robo2wd_v4\build\esp32.esp32.esp32\firmware_robo2wd_v4.ino.partitions.bin" 0xe000 "C:\Users\luneque.junior\Projetos_IA\Teste_car_0\firmware_robo2wd_v4\build\esp32.esp32.esp32\boot_app0.bin" 0x10000 "C:\Users\luneque.junior\Projetos_IA\Teste_car_0\firmware_robo2wd_v4\build\esp32.esp32.esp32\firmware_robo2wd_v4.ino.bin"
```

## 2.1 Particionamento da flash (4 MB)

Esquema sob medida em `firmware_robo2wd_v4/partitions.csv` (o Arduino copia o
CSV do sketch por cima do `default.csv`). Gravado no chip em 23/11/2026.

| Partição | Tipo | Offset | Tamanho | Uso |
| :--- | :--- | :--- | :--- | :--- |
| nvs | data/nvs | 0x009000 | 20 KB | Wi-Fi, calibração da IMU, namespace `bateria` |
| otadata | data/ota | 0x00E000 | 8 KB | estado do OTA |
| app0 | app/ota_0 | 0x010000 | **3,75 MB** | firmware (slot único) |
| spiffs | data/spiffs | 0x3D0000 | **64 KB** | dados gerais (mínimo) |
| coredump | data/coredump | 0x3E0000 | 64 KB | captura de panic |
| *(livre)* | — | 0x3F0000 | 64 KB | reserva |

**Decisão:** slot único + SPIFFS mínima. O firmware usa ~1,15 MB, não tem OTA
implementado e não usa SPIFFS/LittleFS (o dashboard é servido de PROGMEM).
Com um slot só a folga real aparece: **~2,6 MB** por imagem (contra ~100 KB
antes).

**Contrapartida assumida:** sem o segundo slot **não há rollback**. Se uma
imagem nova falhar, a recuperação é somente gravando pelo cabo USB. Para
recuperar o rollback, basta voltar ao esquema anterior (dois slots de 1,25 MB)
— o `partitions.csv` é versionado.

**Nota sobre o relatório de compilação:** o `boards.txt` do core tem
`upload.maximum_size=1310720` **fixo**, então o compilador informa
"Maximum is 1310720 bytes" mesmo com um slot de 3,75 MB. É cosmético (o que vai
para o chip é o `partitions.bin`), mas para o número correto:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 \
  --build-property upload.maximum_size=3932160 firmware_robo2wd_v4
# Sketch uses 1148411 bytes (29%) of program storage space. Maximum is 3932160 bytes.
```

**Atenção:** trocar de esquema apaga a NVS; como o offset da NVS é o mesmo
(0x9000), a calibração da bateria e as credenciais Wi-Fi **sobrevivem** —
confirmado no chip após a mudança ("Calibragem carregada da NVS").

---

## 📄 Atualizações Recentes (27/11/2026 — frenagem ativa)

- **Frenagem ativa no corte:** entre o atingimento da meta e a pausa entre segmentos, o firmware aplica um pulso de 60 ms com PWM oposto ao movimento de cada roda (`FRENAGEM_PWM_US = 1400`, simétrico: 1400 ↔ 1600 µs em torno do neutro). Em retas as duas rodas recebem 1400 µs; em pivôs, uma recebe 1400 e a outra 1600, conforme `g_sentidoGiro`. O pulso elimina quase toda a inércia pós-corte (deslizamento da reta ~20 mm → ~3 mm, salto de yaw ~2° → ~0,3°). Para desativar: `FRENAGEM_ATIVA = false` em `config.h`.
- **Constantes de parada vigentes:** `RETA_ANTECIPACAO_MM = 20`, `PIVO_ANTECIPACAO_GRAUS = 3,7°`, `VELOCIDADE_GIRO_RPM = 20`, `VELOCIDADE_MIN_PIVO_RPM = 12`, `FRENAGEM_TEMPO_MS = 60`.
- **Gráficos do monitor:** o dashboard só acrescenta pontos durante `RETA`/`PIVO`. Amostras de repouso são ignoradas e cada novo experimento reinicia a série com tempo relativo ao início do movimento — o gráfico não "foge" para a esquerda após o robô ficar parado.
- **Procedimento obrigatório:** após gravar firmware, **esperar o robô ser colocado no solo e confirmado pelo usuário antes de rodar qualquer teste**. Com o robô suspenso na bancada o corpo não gira e o teste não mede nada de útil (pivôs ~0° por fallback de ticks).
- **Resultado de referência (bateria ~7,8 V):** lados 499,2 mm, pivôs 92,2°, fechamento 55,8 mm. O desvio angular residual de ~2° por pivô está sob investigação; a alavanca a testar é a rampa/velocidade do pivô.

## 📄 Atualizações Recentes (27/11/2026)

### Firmware
- **Antecipação da reta (`RETA_ANTECIPACAO_MM = 20`):** `iniciarMovimentoReta()` calcula o alvo de ticks com `distanciaMm - 20` e a rampa de desaceleração usa o mesmo alvo. A reta isolada passou de 517–522 mm para **499,5–504,2 mm** (erro médio +19,5 mm → +1,9 mm). O encoder conta certo — o excesso era inércia pós-corte.
- **Pivô:** `VELOCIDADE_GIRO_RPM` 25 → **20** e novo `VELOCIDADE_MIN_PIVO_RPM = 12`, usado por `calcularRPMTrapezoidalCurva()` (as retas mantêm `VELOCIDADE_MIN_RPM = 20`). Com 20/20 a rampa era plana; agora há desaceleração real antes do corte. `PIVO_ANTECIPACAO_GRAUS` fica em 5,7° — reduzi-lo piora o giro real.
- **`config.h`:** antecipação da reta e piso do pivô documentados com as medições que os originaram.

### Resultados no solo
- Reta isolada: 499,5/504,2 mm; 4 pivôs: giro real 90,77° (inércia 5,5°).
- Quadrado (2 rodadas): lados 496,5 mm e 499,6 mm; pivôs 89,75° e 90,56°; fechamento **2,8 mm** e **17,6 mm**. Variação atribuída ao bias do giroscópio no boot (−0,020 a −0,010 °/s).

### Testes
- 16 + 17 testes nativos, 18 Python, todos verdes; compilação/upload OK.

## 📄 Atualizações Recentes (19/11/2026)

### Firmware
- **`yawReal` (rumo real):** `g_yawAbs + g_inerciaTotal` — os cortes **mais** a inércia de frenagem de todos os pivôs. O `yawAbs` é o rumo **congelado no corte** e esconde ~5–9° por pivô.
- **Pivô recalibrado em solo:** `VELOCIDADE_GIRO_RPM` 35 → **25 RPM** e `PIVO_ANTECIPACAO_GRAUS` 0,7° → **5,7°**.
- **Diagnóstico:** `g_yawOdom`, `diverg`, `gyroBias` com autoteste no boot, `inercia`/`inerciaOdom` em acumuladores separados.
- **Diagnóstico de travamento:** a telemetria publica `lm` (maior intervalo entre duas iterações do `loop()` desde a última amostra) e `sr`/`si` (duração do pior bloco e o identificador dele: 1 Wi-Fi, 2 telemetria/enfileiramento, 3 gravador, 4 bateria, 5 controle/FSM). São os campos que localizaram o travamento por escrita síncrona no socket TCP.
- **Comandos de bancada:** `📏 Reta 500` (`t`) e `🔄 Pivô 90` (`g`), com botões no dashboard.
- **Wi-Fi:** reconexão automática no `loop()`.
- **Gravador:** 1984 → **1700** amostras (as colunas de diagnóstico estouraram o DRAM).

### Dashboard
- O rastro do mapa usa o **rumo real** (`yawReal`), com fallback para `yawAbs` em firmware antigo.
- Alerta de divergência: limiar 5° → **30°** (a divergência acumulada é normal, ~5° por pivô).
- CSV exportado (dashboard) ganha `yawReal` e `inercia`. Os outros formatos — dump do gravador, logger WS e gateway ESP-NOW — não têm essas colunas; veja a seção "Formatos de CSV".

## 📄 Atualizações Recentes (13/11/2026)

- **Firmware:** odometria recalibrada para roda de 66 mm (10.417 ticks por reta de 500 mm); corte de pivô por delta real de yaw (`g_yawStartSeg`) com guarda de 200 ticks; `K_SYNC = 0.15` e `IMU_KP_YAW_RETA = 2.0`; limite de PWM ±50 µs; giro pivô suavizado para 35 RPM.
- **Dashboard Web:** quadrado ideal usando coordenadas reais (500 mm × 500 mm) em ordem **Início → L1 → L2 → L3 → Início**; rastro real em verde com odometria cumulativa; indicador de pose (X, Y, θ) numérico; seta de direção correta nos pivôs.
- **Documentação:** README, Diário de Engenharia, FSD e este Manual agora contém o resumo das mudanças.
