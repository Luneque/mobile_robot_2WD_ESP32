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
* **Gateway ESP-NOW:** Suporte a um microcontrolador auxiliar ESP32 para transmissão de telemetria sem necessidade de ponto de acesso Wi-Fi.
* **Periféricos de Silício Dedicados:** **MCPWM** (50 Hz / resolução de $1\,\mu\text{s}$) e **PCNT** (decodificação de quadratura 4x por hardware com filtro anti-glitch de 1000 ns).
* **Compensação Dinâmica de Zona Morta:** Feedforward proporcional à velocidade, derivado da curva PWM×RPM medida em bancada e corrigido pelo fator de carga do robô no solo (`CURVA_FATOR_CARGA = 0.80`), vencendo o atrito estático já na partida.
* **Fusão Sensorial com IMU BMI160:** Correção contínua de rumo em linha reta (`IMU_KP_YAW_RETA = 2.0`) e controle estrito de parada de giro pivô por delta de yaw (corte em ~84° com antecipação de 5,7°).

---

## 2. Especificação e Conexões de Hardware

### 2.1 Componentes do Sistema
1. **Placa Controladora:** ESP32-D0WD-V3 acoplado à placa de expansão **Goouuu 38P / V4** com alimentação regulada.
2. **Atuadores:** 2 Micromotores DC com redução metálica homologada de **1:90**.
3. **Sensores de Velocidade:** Encoders magnéticos de efeito Hall (disco magnético de **12 PPR** por canal).
4. **Acionamento de Potência:** ESC escovado duplo bidirecional configurado no modo independente (*Mixed Control = OFF*).
5. **Alimentação:** Bateria LiPo 2S (~7.4 V).

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
* **[`imu_bmi160.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v4/imu_bmi160.h):** Driver nativo I2C (400 kHz) para a IMU Bosch BMI160. Fornece calibração estática de *bias* do giroscópio, leitura direta do eixo Z (giro horário = Yaw positivo) e integração de ângulo de guinada (*Yaw*). O firmware usa o delta real de yaw (`deltaYaw = yawAtual − g_yawStartSeg`, capturado no início de cada segmento) para correção de rumo em retas (`IMU_KP_YAW_RETA = 2.0`). No pivô, a IMU atua pelo critério de parada (`|deltaYaw| ≥ (ânguloAlvo − PIVO_ANTECIPACAO_GRAUS)`, corte em ~84° com antecipação calibrada de 5,7°) com guarda mínima de 200 ticks (`MIN_TICKS_PIVO`) para evitar disparos prematuros por resíduo de yaw; o sincronismo cruzado é feito apenas pelos encoders. O acelerômetro é inicializado pelo driver (ODR 100 Hz, ±2 g), mas não é lido pelo firmware — apenas o utilitário `diag_i2c_bmi160` faz a leitura.
* **[`motor_control_mcpwm.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v4/motor_control_mcpwm.h):** Gerador determinístico de PWM a 50 Hz com resolução de $1\,\mu\text{s}$ via timer de 1 MHz.
* **[`encoder_reader_pcnt.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v4/encoder_reader_pcnt.h):** Decodificador de pulsos com filtro anti-glitch de 1000 ns e filtro exponencial de velocidade (EMA).
* **[`pid_controller.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v4/pid_controller.h):** Controlador PID com anti-windup e feedforward proporcional à velocidade, obtido por interpolação da curva PWM×RPM medida em bancada (`diag_motor_curve`) e corrigido pelo fator de carga (`CURVA_FATOR_CARGA`).
* **`telemetry_recorder.h`:** Gravador de telemetria em RAM (1700 amostras a 20 Hz, 85 s, 40 bytes por amostra), com dump por Serial (`d`) ou via WebSocket/ESP-NOW (`dump`). Permite testar o robô no solo sem cabo.
* **`espnow_link.h`:** Enlace ESP-NOW com o gateway ESP32. Envia a telemetria em JSON (20 Hz em movimento / 2 Hz parado) e recebe comandos de texto. Compatível com ESP-IDF v4 e v5. O dump do gravador é devolvido pelo canal que recebeu o pedido.
* **`firmware_robo2wd_v4.ino`:** Máquina de estados não-bloqueante (`ESTADO_AGUARDO_INICIO`, `ESTADO_EXECUTANDO_TRAJETORIA`, `ESTADO_CONCLUIDO`, `ESTADO_EMERGENCIA`), perfil trapezoidal, fusão sensorial IMU + Encoders e dispatch centralizado de comandos.

### 4.2 Dinâmica do Perfil Trapezoidal
* **Reta (500 mm):**
  * Acelera de 20 RPM até **55 RPM** nos primeiros **50 mm** ($\approx 600\text{ mm/s}^2$).
  * Mantém velocidade de cruzeiro estável por **390 mm** (78% da reta).
  * Desacelera nos últimos **60 mm** para parada cravada na marca de 500 mm (10.417 ticks).
  * Tempo total por reta: **$\approx 3,7\text{ s}$** medido em solo (com feedforward por velocidade).
* **Curva Pivô (90°):**
  * Acelera em 10° e desacelera em 15°, com cruzeiro suavizado de **25 RPM** no giro pivô.
  * Corte em ~84° com **antecipação calibrada de 5,7°** (`PIVO_ANTECIPACAO_GRAUS`); a **inércia de frenagem** (~5°) completa os ~90°.
  * Tempo total por giro: **$\approx 1,65\text{ s}$** de movimento + 0,8 s de pausa = **$\approx 2,45\text{ s}$** (medido em solo).

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
7. A qualquer momento, envie comando de parada remota pelo painel para interromper o movimento.
8. Ao término, clique em **DOWNLOAD CSV** no dashboard para salvar o registro completo dos dados da corrida.

### 5.2 Operação Local via Cabo Serial (Fallback de Bancada)
1. Conecte o robô via USB (`COM9`) e abra o monitor serial em **115200 baud**.
2. O robô aceita o envio de **`s`** (start), **`t`** (reta única de 500 mm), **`g`** (pivô único de 90°), **`d`** (dump), **`p`** (pause), **`x`** (stop) e **`r`** (reset) pela serial.
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

### 5.4 Operação via ESP-NOW (Gateway ESP32)
> **Hardware validado:** um **ESP32 clássico** (ESP-WROOM-32, fqbn `esp32:esp32:esp32`, porta `COM18`). O ESP32-S3 era o plano original, mas a placa apresentou defeito e não chegou a ser usado. O código é de placa genérica e roda em qualquer ESP32.
1. Grave `gateway_espnow/gateway_espnow.ino` no ESP32 auxiliar e anote o MAC impresso no Serial.
2. Ajuste `ESPNOW_CHANNEL` em `config.h` do robô para o mesmo canal do gateway.
3. Conecte o gateway ao PC e execute a ponte:
   ```
   python robo_monitor/gateway_espnow.py --port COM9 --csv telemetria_espnow.csv
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

## 📄 Atualizações Recentes (19/11/2026)

### Firmware
- **`yawReal` (rumo real):** `g_yawAbs + g_inerciaTotal` — os cortes **mais** a inércia de frenagem de todos os pivôs. O `yawAbs` é o rumo **congelado no corte** e esconde ~5–9° por pivô.
- **Pivô recalibrado em solo:** `VELOCIDADE_GIRO_RPM` 35 → **25 RPM** e `PIVO_ANTECIPACAO_GRAUS` 0,7° → **5,7°**.
- **Diagnóstico:** `g_yawOdom`, `diverg`, `gyroBias` com autoteste no boot, `inercia`/`inerciaOdom` em acumuladores separados.
- **Comandos de bancada:** `📏 Reta 500` (`t`) e `🔄 Pivô 90` (`g`), com botões no dashboard.
- **Wi-Fi:** reconexão automática no `loop()`.
- **Gravador:** 1984 → **1700** amostras (as colunas de diagnóstico estouraram o DRAM).

### Dashboard
- O rastro do mapa usa o **rumo real** (`yawReal`), com fallback para `yawAbs` em firmware antigo.
- Alerta de divergência: limiar 5° → **30°** (a divergência acumulada é normal, ~5° por pivô).
- CSV exportado ganha `yawReal` e `inercia`.

## 📄 Atualizações Recentes (13/11/2026)

- **Firmware:** odometria recalibrada para roda de 66 mm (10.417 ticks por reta de 500 mm); corte de pivô por delta real de yaw (`g_yawStartSeg`) com guarda de 200 ticks; `K_SYNC = 0.15` e `IMU_KP_YAW_RETA = 2.0`; limite de PWM ±50 µs; giro pivô suavizado para 35 RPM.
- **Dashboard Web:** quadrado ideal usando coordenadas reais (500 mm × 500 mm) em ordem **Início → L1 → L2 → L3 → Início**; rastro real em verde com odometria cumulativa; indicador de pose (X, Y, θ) numérico; seta de direção correta nos pivôs.
- **Documentação:** README, Diário de Engenharia, FSD e este Manual agora contém o resumo das mudanças.
