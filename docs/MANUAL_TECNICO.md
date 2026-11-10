# Manual de Engenharia e Operação — Robô Diferencial 2WD com ESP32

---

> [!IMPORTANT]
> **Versão Oficial Atual: Versão 3 (`firmware_robo2wd_v3/`)**  
> As implementações anteriores (Versão 1 baseada em interrupções por software e Versão 2 baseada em loops bloqueantes) foram arquivadas no diretório [`legacy/`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/legacy/) para fins de histórico e rastreabilidade técnica.

---

## 1. Apresentação do Projeto

Este projeto consiste em uma plataforma robótica móvel de tração diferencial (2WD) baseada no microcontrolador **ESP32-D0WD-V3**. O sistema executa trajetórias de alta precisão com realimentação em malha fechada (PID), perfil de velocidade trapezoidal e sincronismo dinâmico entre rodas (*cross-coupling*).

A versão oficial do firmware (**Versão 3**) opera com:
* **Periféricos de Silício Dedicados:** **MCPWM** (50 Hz / resolução de $1\,\mu\text{s}$) e **PCNT** (decodificação de quadratura 4x por hardware com filtro anti-glitch de 1000 ns).
* **Máquina de Estados Não-Bloqueante:** Execução orientada a tempo no `loop()`, liberando o microcontrolador para tarefas futuras (sensores, telemetria e conectividade).
* **Perfil Trapezoidal de Velocidade:** Rampas ágeis de aceleração e desaceleração, permitindo velocidade de cruzeiro de **80 RPM ($\approx 293\text{ mm/s}$)** sem derrapagem.
* **Segurança Ativa e Hardware Watchdog:** Task Watchdog Timer do ESP32 (`esp_task_wdt`) de 3 segundos e detecção de roda travada (*stall*) em retas e curvas pivô.

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
| **I2C — SDA (Dados)** | `GPIO 21` | I2C Padrão Hardware | **LIVRE** para IMU, OLED, ToF e barômetro |
| **I2C — SCL (Clock)** | `GPIO 22` | I2C Padrão Hardware | **LIVRE** para IMU, OLED, ToF e barômetro |
| **GPIO Geral** | `GPIO 23` | GPIO Universal | **LIVRE** (ex: pino INT de IMU ou trigger) |

---

## 3. Modelo Cinemático e Calibração

### 3.1 Resolução dos Encoders (Quadratura 4x)
$$\text{PPR}_{\text{efetivo}} = 12 \text{ PPR} \times 90 \text{ (Redução)} \times 4\text{x} = \mathbf{4.320 \text{ pulsos/volta}}$$

### 3.2 Deslocamento Linear e Angular
* **Circunferência da Roda ($70\text{ mm}$ de diâmetro):** $\pi \times 70 \approx \mathbf{219,911\text{ mm}}$
* **Resolução Linear ($K_{\text{dist}}$):** $\frac{219,911\text{ mm}}{4.320\text{ pulsos}} \approx \mathbf{0,050905\text{ mm/pulso}}$
* **Bitola entre Rodas ($L$):** $120,0\text{ mm}$
* **Arco de Giro para Curva de 90°:** $\pi \times 120 \times 0,25 \approx \mathbf{94,248\text{ mm}}$ ($\approx 1.851\text{ pulsos por roda}$)

---

## 4. Arquitetura de Software da Versão 3 (Oficial)

A estrutura oficial reside em [`firmware_robo2wd_v3/`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v3/):

### 4.1 Módulos do Sistema
* **[`config.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v3/config.h):** Centraliza pinos, limites de pulso do ESC ($1150\,\mu\text{s}$ a $1850\,\mu\text{s}$), parâmetros cinemáticos, distâncias de rampa e parâmetros do WDT.
* **[`motor_control_mcpwm.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v3/motor_control_mcpwm.h):** Gerador determinístico de PWM a 50 Hz com resolução de $1\,\mu\text{s}$ via timer de 1 MHz.
* **[`encoder_reader_pcnt.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v3/encoder_reader_pcnt.h):** Decodificador de pulsos com filtro anti-glitch de 1000 ns e filtro exponencial de velocidade (EMA).
* **[`pid_controller.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v3/pid_controller.h):** Controlador PID com anti-windup e feedforward dinâmico de zona morta ($\pm 50\,\mu\text{s}$).
* **[`firmware_robo2wd_v3.ino`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v3/firmware_robo2wd_v3.ino):** Máquina de estados não-bloqueante (`ESTADO_AGUARDO_INICIO`, `ESTADO_EXECUTANDO_TRAJETORIA`, `ESTADO_CONCLUIDO`, `ESTADO_EMERGENCIA`), perfil trapezoidal e sincronismo cruzado em retas e curvas pivô.

### 4.2 Dinâmica do Perfil Trapezoidal
* **Reta (500 mm):**
  * Acelera de 20 RPM até **80 RPM** nos primeiros **50 mm** ($\approx 600\text{ mm/s}^2$).
  * Mantém velocidade de cruzeiro estável por **390 mm** (78% da reta).
  * Desacelera nos últimos **60 mm** para parada cravada na marca de 500 mm.
  * Tempo total por reta: **$\approx 1,88\text{ s}$**.
* **Curva Pivô (90°):**
  * Acelera em 10° e desacelera em 15°, mantendo cruzeiro de **50 RPM** em 65° do giro.
  * Tempo total por giro: **$\approx 0,52\text{ s}$**.

---

## 5. Guia de Operação e Comandos

1. Conecte o robô via USB (`COM9`) e abra o monitor serial em **115200 baud**.
2. Ligue a alimentação geral dos motores na bateria.
3. O robô valida os periféricos, arma o ESC por 3 segundos em neutro ($1500\,\mu\text{s}$) e ativa o Watchdog Timer de 3 segundos.
4. Envie o caractere **`s`** ou **`g`** pela serial (ou aguarde 5 segundos para partida automática).
5. O robô percorre os 4 lados de 500 mm com 4 giros de 90° e para com segurança.

---

## 6. Procedimento de Compilação e Gravação

### 6.1 Compilação da Versão 3
```powershell
& "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn esp32:esp32:esp32 --export-binaries "C:\Users\luneque.junior\Projetos_IA\Teste_car_0\firmware_robo2wd_v3"
```

### 6.2 Gravação via `esptool.py` (fechar o monitor serial antes)
```powershell
& "C:\Users\luneque.junior\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\5.3.1\esptool.exe" --chip esp32 --port COM9 --baud 921600 --before default-reset --after hard-reset write_flash -z --flash-mode dio --flash-freq 80m --flash-size 4MB 0x1000 "C:\Users\luneque.junior\Projetos_IA\Teste_car_0\firmware_robo2wd_v3\build\esp32.esp32.esp32\firmware_robo2wd_v3.ino.bootloader.bin" 0x8000 "C:\Users\luneque.junior\Projetos_IA\Teste_car_0\firmware_robo2wd_v3\build\esp32.esp32.esp32\firmware_robo2wd_v3.ino.partitions.bin" 0xe000 "C:\Users\luneque.junior\Projetos_IA\Teste_car_0\firmware_robo2wd_v3\build\esp32.esp32.esp32\boot_app0.bin" 0x10000 "C:\Users\luneque.junior\Projetos_IA\Teste_car_0\firmware_robo2wd_v3\build\esp32.esp32.esp32\firmware_robo2wd_v3.ino.bin"
```
