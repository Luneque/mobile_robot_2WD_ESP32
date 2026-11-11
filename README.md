# Plataforma Robótica Diferencial 2WD — ESP32 (Versão 3 Oficial)

Este repositório contém o firmware embarcado, as ferramentas de diagnóstico e a documentação de engenharia para um robô móvel de tração diferencial (2WD) baseado no microcontrolador **ESP32-D0WD-V3**.

O sistema executa com extrema repetibilidade e controle em malha fechada uma trajetória de teste quadrada fechada (500 mm por lado com curvas de 90° em pivô central).

---

> [!IMPORTANT]
> **Versão Oficial Homologada: `firmware_robo2wd_v3/` (Versão 3)**  
> As implementações anteriores (V1 por software e V2 bloqueante) foram preservadas na pasta [`legacy/`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/legacy/) para histórico de engenharia.

---

## 🚀 Destaques da Versão 3 (Oficial)

* **Periféricos Nativos de Silício (ESP-IDF):**
  * **MCPWM (Motor Control PWM):** Timer de 1 MHz dedicado gerando pulsos para ESC a 50 Hz com resolução de $1\,\mu\text{s}$.
  * **PCNT (Pulse Counter):** Decodificação por hardware de quadratura 4x com filtro digital de 1000 ns, desonerando 100% da CPU.
* **Máquina de Estados Não-Bloqueante (FSM):**
  * O loop principal roda orientado a tempo (`millis()`), pronto para acomodar futuros sensores (LiDAR, ultrassom) ou nós ROS 2 (a IMU BMI160 já está integrada).
* **Perfil de Velocidade Trapezoidal Otimizado (80 RPM):**
  * Rampas progressivas de aceleração (50 mm) e desaceleração (60 mm).
  * Velocidade de cruzeiro de **80 RPM ($\approx 293\text{ mm/s}$)** cobrindo 78% do trajeto em apenas **$\approx 1,88\text{ s}$ por reta**, sem derrapagens na partida ou parada.
  * Curvas de pivô rápidas a **50 RPM ($\approx 0,52\text{ s}$)**.
* **Sincronismo Cruzado (*Cross-Coupling*):**
  * Monitoramento dinâmico de avanço relativo ($e_{\text{sync}} = t_1 - t_2$) travando ambas as rodas em reta e giro pivô.
* **Segurança e Robustez Operacional:**
  * Task Watchdog Timer por hardware (`esp_task_wdt`) ativo em 3 segundos.
  * Detecção ativa de roda travada / desbalanceamento (*stall*) com parada de emergência.
  * Validação estrita no boot com bloqueio seguro contra falhas de periféricos.

---

## 📁 Estrutura de Diretórios

```
Teste_car_0/
├── firmware_robo2wd_v3/       # [OFICIAL] Código-fonte ativo da Versão 3
│   ├── config.h               # Configuração cinemática, limites, ganhos e rampas
│   ├── motor_control_mcpwm.h  # Driver MCPWM (1 MHz, 50 Hz, 1 us)
│   ├── encoder_reader_pcnt.h  # Driver PCNT (Quadratura 4x + Filtro EMA)
│   ├── pid_controller.h       # PID com Feedforward e Anti-Windup
│   └── firmware_robo2wd_v3.ino# FSM não-bloqueante, watchdog e perfil trapezoidal
├── legacy/                    # [LEGADO] Versões históricas preservadas
│   ├── firmware_robo2wd/      # Versão 1: LEDC + Interrupções de software (ISRs)
│   └── firmware_robo2wd_v2/   # Versão 2: MCPWM + PCNT em loops bloqueantes
├── diag_pcnt/                 # Utilitário de diagnóstico e calibração de encoders em hardware
├── diag_encoders/             # Utilitário legado de teste de encoders
└── docs/                      # Documentação técnica e de engenharia
    ├── FSD.md                 # Especificação Funcional Definitiva (v3.0.0 em PT-BR)
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
* 📄 **[Especificação Funcional (FSD v3.0)](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/docs/FSD.md)**
* 📄 **[Diário de Engenharia](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/docs/DIARIO_ENGENHARIA.md)**
