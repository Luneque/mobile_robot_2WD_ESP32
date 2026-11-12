# Manual de Engenharia e Operação — Robô Diferencial 2WD com ESP32

---

> [!IMPORTANT]
> **Versão Oficial Atual: Versão 4 (`firmware_robo2wd_v4/`)**  
> As implementações anteriores (Versão 1 por software, Versão 2 bloqueante e Versão 3 com fio/serial) foram arquivadas no diretório [`legacy/`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/legacy/) para histórico de engenharia e rastreabilidade técnica.

---

## 1. Apresentação do Projeto

Este projeto consiste em uma plataforma robótica móvel de tração diferencial (2WD) baseada no microcontrolador **ESP32-D0WD-V3**. O sistema executa trajetórias de alta precisão com realimentação em malha fechada (PID), perfil de velocidade trapezoidal, compensação de zona morta por feedforward e sincronismo dinâmico entre rodas (*cross-coupling*), integrado com uma IMU Bosch BMI160.

A versão oficial do firmware (**Versão 4**) opera com:
* **Conectividade Sem Fio e Telemetria em Tempo Real:** Wi-Fi Station, WebSocket Server a 5 Hz e Dashboard Web embarcado em memória flash (servido em `http://robo.local/`), permitindo visualização de gráficos de alta frequência e disparo remoto sem cabos.
* **Gateway ESP-NOW:** Suporte ao microcontrolador auxiliar ESP32-S3 Super Mini para transmissão de telemetria sem necessidade de ponto de acesso Wi-Fi.
* **Periféricos de Silício Dedicados:** **MCPWM** (50 Hz / resolução de $1\,\mu\text{s}$) e **PCNT** (decodificação de quadratura 4x por hardware com filtro anti-glitch de 1000 ns).
* **Compensação Dinâmica de Zona Morta:** Feedforward calibrado em `110 µs` (`ESC_DEADBAND_US`), vencendo o atrito estático imediatamente na partida.
* **Fusão Sensorial com IMU BMI160:** Correção contínua de rumo em linha reta (`IMU_KP_YAW_RETA = 1.2`) e controle estrito de parada de giro pivô por integração giroscópica de 90°.

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
* **Circunferência da Roda ($70\text{ mm}$ de diâmetro):** $\pi \times 70 \approx \mathbf{219,911\text{ mm}}$
* **Resolução Linear ($K_{\text{dist}}$):** $\frac{219,911\text{ mm}}{4.320\text{ pulsos}} \approx \mathbf{0,050905\text{ mm/pulso}}$
* **Bitola entre Rodas ($L$):** $120,0\text{ mm}$
* **Arco de Giro para Curva de 90°:** $\pi \times 120 \times 0,25 \approx \mathbf{94,248\text{ mm}}$ ($\approx 1.851\text{ pulsos por roda}$)

---

## 4. Arquitetura de Software da Versão 4 (Oficial)

A estrutura oficial reside em [`firmware_robo2wd_v4/`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v4/):

### 4.1 Módulos do Sistema
* **[`config.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v4/config.h):** Centraliza pinos, limites de pulso do ESC ($1150\,\mu\text{s}$ a $1850\,\mu\text{s}$), parâmetros cinemáticos, distâncias de rampa, parâmetros do WDT e ganhos de fusão com a IMU (`IMU_KP_YAW_RETA`).
* **[`imu_bmi160.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v4/imu_bmi160.h):** Driver nativo I2C (400 kHz) para a IMU Bosch BMI160. Fornece calibração estática de *bias* do giroscópio, inversão de polaridade (giro horário = Yaw positivo) e integração de ângulo de guinada (*Yaw*). O firmware usa `yawError = yawAtual` (erro relativo ao início de cada segmento, pois `resetYaw()` zera o ângulo) para correção de rumo em retas (`IMU_KP_YAW_RETA = 1.2`). No pivô, a IMU atua pelo critério de parada (`|yawError| ≥ (ânguloAlvo − 2°)`) e pelo mapeamento do perfil trapezoidal; o sincronismo cruzado é feito apenas pelos encoders. O acelerômetro é inicializado pelo driver (ODR 100 Hz, ±2 g), mas não é lido pelo firmware — apenas o utilitário `diag_i2c_bmi160` faz a leitura.
* **[`motor_control_mcpwm.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v4/motor_control_mcpwm.h):** Gerador determinístico de PWM a 50 Hz com resolução de $1\,\mu\text{s}$ via timer de 1 MHz.
* **[`encoder_reader_pcnt.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v4/encoder_reader_pcnt.h):** Decodificador de pulsos com filtro anti-glitch de 1000 ns e filtro exponencial de velocidade (EMA).
* **[`pid_controller.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v4/pid_controller.h):** Controlador PID com anti-windup e feedforward dinâmico de zona morta ($\pm 110\,\mu\text{s}$).
* **[`firmware_robo2wd_v4.ino`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v4/firmware_robo2wd_v4.ino):** Máquina de estados não-bloqueante (`ESTADO_AGUARDO_INICIO`, `ESTADO_EXECUTANDO_TRAJETORIA`, `ESTADO_CONCLUIDO`, `ESTADO_EMERGENCIA`), perfil trapezoidal e fusão sensorial IMU + Encoders.

### 4.2 Dinâmica do Perfil Trapezoidal
* **Reta (500 mm):**
  * Acelera de 20 RPM até **55 RPM** nos primeiros **50 mm** ($\approx 600\text{ mm/s}^2$).
  * Mantém velocidade de cruzeiro estável por **390 mm** (78% da reta).
  * Desacelera nos últimos **60 mm** para parada cravada na marca de 500 mm.
  * Tempo total por reta: **$\approx 2,50\text{ s}$** (em 55 RPM).
* **Curva Pivô (90°):**
  * Acelera em 10° e desacelera em 15°, mantendo cruzeiro de **50 RPM** em 65° do giro.
  * Tempo total por giro: **$\approx 1,80\text{ s}$** (com parada precisa a 89° e pausa).

---

## 5. Guia Rápido de Operação e Comandos (Versão 4)

### 5.1 Operação Sem Fio pelo Navegador (Recomendado)
1. Ligue a alimentação dos motores e do ESP32 na bateria.
2. O robô se conectará automaticamente à rede Wi-Fi configurada (`secrets.h`).
3. No PC ou celular na mesma rede, abra:
   * **`http://robo.local/`** (ou o IP atribuído, ex: `http://192.168.1.102/`).
4. O painel exibirá os gráficos de RPM, Yaw e PWM em tempo real.
5. Posicione o robô no solo e clique no botão verde **START** para iniciar a trajetória quadrada.
6. A qualquer momento, envie comando de parada remota pelo painel para interromper o movimento.
7. Ao término, clique em **DOWNLOAD CSV** no dashboard para salvar o registro completo dos dados da corrida.

### 5.2 Operação Local via Cabo Serial (Fallback de Bancada)
1. Conecte o robô via USB (`COM9`) e abra o monitor serial em **115200 baud**.
2. O robô aceita o envio de **`s`** ou **`g`** pela serial para início manual (ou aguarda 5s para timeout).

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
