# Plataforma Robótica Diferencial 2WD — ESP32 (MCPWM + PCNT Hardware)

Este repositório contém o firmware embarcado, os utilitários de diagnóstico e a documentação técnica completa de um robô de tração diferencial (2WD) baseado no microcontrolador **ESP32-D0WD-V3**.

O sistema executa com precisão trajetórias fechadas (como um quadrado de 500 mm com giros de 90° em pivô central) utilizando controle de velocidade em malha fechada (PID), decodificação de quadratura 4x por hardware e sincronismo dinâmico entre rodas.

---

## 🚀 Principais Características

* **Arquitetura 100% em Hardware Dedicado (ESP-IDF):**
  * **MCPWM (Motor Control PWM):** Geração determinística de pulsos a 50 Hz com resolução de $1\,\mu\text{s}$ para controle de ESC.
  * **PCNT (Pulse Counter):** Decodificação por hardware de quadratura 4x com filtro digital de ruído de 1000 ns, liberando a CPU de qualquer interrupção de encoder.
* **Algoritmo de Sincronismo Cruzado (*Cross-Coupling*):**
  * Elimina variações de atrito mecânico ou diferenças de resposta elétrica entre os motores, travando as duas rodas com o mesmo avanço milimétrico em retas e curvas.
* **Calibração de Alta Resolução:**
  * Redução real comprovada de **1:90**.
  * Resolução de **4.320 pulsos por volta completa da roda** ($12\text{ PPR} \times 90 \times 4\text{x}$).
* **Desempenho Otimizado:**
  * Velocidade de cruzeiro de **65 RPM** ($\approx 238\text{ mm/s}$ em reta).
  * Giros rápidos de 90° a **40 RPM** com pausas de dissipação inercial de 800 ms.

---

## 📁 Estrutura de Diretórios

```
Teste_car_0/
├── docs/
│   ├── FSD.md                 # Especificação Funcional Definitiva (v2.0.0 em PT-BR)
│   ├── MANUAL_TECNICO.md      # Manual completo de operação, pinagem, cinemática e compilação
│   └── DIARIO_ENGENHARIA.md   # Registro cronológico das decisões de projeto e calibração
├── firmware_robo2wd_v2/       # Código-fonte oficial do robô (MCPWM + PCNT)
│   ├── config.h               # Parâmetros de cinemática, pinos, ganhos e velocidades
│   ├── motor_control_mcpwm.h  # Driver MCPWM (50 Hz, 1 us de resolução)
│   ├── encoder_reader_pcnt.h  # Driver PCNT (Quadratura 4x + Filtro EMA de RPM)
│   ├── pid_controller.h       # Controlador PID com Anti-Windup e Feedforward de Deadband
│   └── firmware_robo2wd_v2.ino# Máquina de estados, sincronismo cruzado e trajetória
├── diag_pcnt/                 # Utilitário de validação direta de contagem e quadratura em hardware
└── diag_encoders/             # Utilitário legado de diagnóstico por software
```

---

## 📌 Mapeamento de Pinos (GPIO)

| Pino ESP32 | Periférico | Função Conectada |
| :---: | :---: | :--- |
| **GPIO 19** | MCPWM Op1 | Sinal do ESC Motor Esquerdo (CH1) |
| **GPIO 18** | MCPWM Op2 | Sinal do ESC Motor Direito (CH2) |
| **GPIO 23** | PCNT Unit0 | Fase A do Encoder Esquerdo |
| **GPIO 22** | PCNT Unit0 | Fase B do Encoder Esquerdo |
| **GPIO 33** | PCNT Unit1 | Fase A do Encoder Direito |
| **GPIO 32** | PCNT Unit1 | Fase B do Encoder Direito |

---

## 📖 Documentação Completa

Para detalhes aprofundados sobre a cinemática, equacionamento matemático, pinagem e instruções passo a passo de compilação e gravação, consulte:
* 📄 **[Manual Técnico e Operacional](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/docs/MANUAL_TECNICO.md)**
* 📄 **[Documento de Especificação Funcional (FSD)](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/docs/FSD.md)**
* 📄 **[Diário de Engenharia](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/docs/DIARIO_ENGENHARIA.md)**
