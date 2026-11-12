# Plataforma Robótica Diferencial 2WD — ESP32 (Versão 4 Oficial)

Este repositório contém o firmware embarcado, as ferramentas de diagnóstico, monitoramento sem fio e a documentação de engenharia para um robô móvel de tração diferencial (2WD) baseado no microcontrolador **ESP32-D0WD-V3**.

O sistema executa com extrema repetibilidade e controle em malha fechada uma trajetória de teste quadrada fechada (500 mm por lado com curvas de 90° em pivô central orientadas por IMU), agora com **telemetria bidirecional em tempo real via Wi-Fi/WebSocket**, **dashboard web embarcado** e **ponte ESP-NOW para ESP32-S3**.

---

> [!IMPORTANT]
> **Versão Oficial Homologada: `firmware_robo2wd_v4/` (Versão 4)**  
> As implementações anteriores (V1 por software, V2 bloqueante e V3 com fio/serial) foram preservadas na pasta [`legacy/`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/legacy/) para histórico de engenharia.

---

## 🚀 Destaques da Versão 4 (Oficial)

* **Conectividade Sem Fio e Telemetria em Tempo Real:**
  * **Wi-Fi Station & WebSocket Server (5 Hz):** Transmissão contínua de telemetria completa (RPM alvo/real, pulsos de encoders, ângulo de Yaw da IMU, comandos PWM e estados da FSM).
  * **Dashboard Web Embarcado (PROGMEM):** Acessível diretamente pelo navegador via `http://robo.local/` ou pelo IP local, com gráficos em tempo real (Chart.js), visualização de estado, disparo remoto de trajetória e exportação direta para CSV.
  * **Ponte ESP-NOW (ESP32-S3 Super Mini):** Firmware dedicado em `gateway_espnow/` para operação transparente via rádio 2.4 GHz em campo ou bancada sem necessidade de roteador Wi-Fi.
  * **Monitor e Logger Python:** Utilitário assíncrono em `robo_monitor/` para gravação e análise automatizada de corridas.
* **Fusão Sensorial e Orientação Absoluta (IMU Bosch BMI160):**
  * Correção giroscópica contínua em retas (`IMU_KP_YAW_RETA = 1.2`) reduzindo o desvio angular para menos de 2°.
  * Parada exata de pivô por integração de Yaw (`|yaw| >= 88.0°`), eliminando erros de derrapagem nas curvas de 90°.
* **Compensação Dinâmica de Zona Morta e Atrito:**
  * Feedforward otimizado (`ESC_DEADBAND_US = 110 µs`) eliminando tempo de estol na partida e permitindo arrancada instantânea em t=0.
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
│   ├── pid_controller.h       # PID com Feedforward de zona morta e Anti-Windup
│   └── firmware_robo2wd_v4.ino# FSM não-bloqueante, partida remota e telemetria 5Hz
├── gateway_espnow/            # [GATEWAY] Firmware para ESP32-S3 Super Mini (Ponte ESP-NOW <-> USB)
│   └── gateway_espnow.ino
├── robo_monitor/              # [FERRAMENTAS PC] Scripts de recepção e análise de dados
│   ├── robo_ws_logger.py      # Logger WebSocket assíncrono para terminal e CSV
│   ├── requirements.txt       # Dependências Python (websockets, rich)
│   └── logs/                  # Diretório de armazenamento de corridas em CSV
├── legacy/                    # [LEGADO] Versões históricas preservadas
│   ├── firmware_robo2wd/      # Versão 1: LEDC + Interrupções de software (ISRs)
│   ├── firmware_robo2wd_v2/   # Versão 2: MCPWM + PCNT em loops bloqueantes
│   └── firmware_robo2wd_v3/   # Versão 3: FSM + Perfil Trapezoidal + IMU (com fio/serial)
├── diag_pcnt/                 # Utilitário de diagnóstico e calibração de encoders em hardware
├── diag_i2c_bmi160/           # Utilitário de diagnóstico I2C e leitura da IMU BMI160
├── calib_imu_polaridade/      # Ferramenta de validação da polaridade do eixo Z da IMU
├── diag_encoders/             # [LEGADO V1] Utilitário de teste de encoders (pinout GPIO 23/22)
└── docs/                      # Documentação técnica e de engenharia
    ├── FSD.md                 # Especificação Funcional Definitiva (v4.0.0 em PT-BR)
    ├── MANUAL_TECNICO.md      # Manual completo de pinagem, cinemática, operação e compilação
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
* 📄 **[Diário de Engenharia](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/docs/DIARIO_ENGENHARIA.md)**
