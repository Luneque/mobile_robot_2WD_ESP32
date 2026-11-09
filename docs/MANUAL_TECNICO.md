# Manual de Engenharia e Operação — Robô Diferencial 2WD com ESP32 (MCPWM + PCNT)

---

## 1. Apresentação do Projeto

Este projeto consiste em uma plataforma robótica móvel de tração diferencial (2WD) baseada no microcontrolador **ESP32-D0WD-V3**. O sistema foi concebido para executar trajetórias de alta precisão com realimentação em malha fechada, operando em baixa e média velocidade com estabilização angular e linear.

A versão atual e definitiva do firmware (**Versão 2**) se diferencia por utilizar **100% de periféricos dedicados de hardware do silício do ESP32**:
* **MCPWM (Motor Control PWM):** Geração determinística de trem de pulsos de servo/ESC a 50 Hz com resolução de $1\,\mu\text{s}$.
* **PCNT (Pulse Counter):** Decodificação por hardware de sinais em quadratura 4x com filtragem digital de ruído (*glitch filter*), sem qualquer interrupção ou sobrecarga da CPU.
* **Algoritmo de Sincronismo Cruzado (*Cross-Coupling*):** Travamento de avanço e velocidade relativa entre os motores esquerdo e direito em tempo real.

---

## 2. Especificação e Conexões de Hardware

### 2.1 Componentes do Sistema
1. **Placa Controladora:** ESP32-D0WD-V3 acoplado à placa de expansão **Goouuu 38P / V4** com barramento de alimentação servo dedicado.
2. **Atuadores:** 2 Micromotores DC com redutores metálicos com redução real homologada de **1:90**.
3. **Sensores de Velocidade:** Encoders magnéticos de efeito Hall montados nos motores, com disco magnético de **12 PPR** por canal.
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
   GPIO 19 ──► ESC Sinal CH1    GPIO 23 ◄── Fase A           GPIO 33 ◄── Fase A
   GPIO 18 ──► ESC Sinal CH2    GPIO 22 ◄── Fase B           GPIO 32 ◄── Fase B
                                (Pull-up interno +           (Pull-up interno +
                                 Filtro PCNT 1000ns)          Filtro PCNT 1000ns)
```

| Sinal | Pino GPIO | Tipo de Periférico | Função |
| :--- | :---: | :---: | :--- |
| **ESC Motor Esquerdo (CH1)** | `GPIO 19` | MCPWM (Timer 0, Gen 1) | Controle de velocidade/sentido do motor esquerdo |
| **ESC Motor Direito (CH2)** | `GPIO 18` | MCPWM (Timer 0, Gen 2) | Controle de velocidade/sentido do motor direito |
| **Encoder Esquerdo — Fase A** | `GPIO 23` | PCNT (Unidade 0, Canal A) | Leitura de quadratura do motor esquerdo |
| **Encoder Esquerdo — Fase B** | `GPIO 22` | PCNT (Unidade 0, Canal B) | Leitura de quadratura do motor esquerdo |
| **Encoder Direito — Fase A** | `GPIO 33` | PCNT (Unidade 1, Canal A) | Leitura de quadratura do motor direito |
| **Encoder Direito — Fase B** | `GPIO 32` | PCNT (Unidade 1, Canal B) | Leitura de quadratura do motor direito |

---

## 3. Modelo Cinemático e Calibração

### 3.1 Resolução dos Encoders (Quadratura 4x)
A contagem de pulsos por volta completa da roda considera a decodificação em quadratura 4x (bordas de subida e descida de ambas as fases):

$$\text{PPR}_{\text{efetivo}} = \text{PPR}_{\text{disco}} \times \text{Redução} \times 4 = 12 \times 90 \times 4 = \mathbf{4.320 \text{ pulsos/volta}}$$

### 3.2 Deslocamento Linear e Angular
* **Diâmetro da Roda ($D$):** $70,0\text{ mm}$
* **Circunferência da Roda ($C$):** $\pi \times 70,0\text{ mm} \approx \mathbf{219,911\text{ mm}}$
* **Resolução Linear ($K_{\text{dist}}$):**
  $$K_{\text{dist}} = \frac{219,911\text{ mm}}{4.320\text{ pulsos}} \approx \mathbf{0,050905\text{ mm/pulso}}$$
* **Bitola entre Eixos das Rodas ($L$):** $120,0\text{ mm}$
* **Arco de Giro para Curva de 90°:**
  $$\text{Arco}_{90^\circ} = \pi \times L \times \frac{90^\circ}{360^\circ} = \pi \times 120 \times 0,25 \approx \mathbf{94,248\text{ mm}}$$
* **Pulsos por Roda em um Giro de 90°:**
  $$\text{Target Ticks}_{90^\circ} = \frac{94,248\text{ mm}}{0,050905\text{ mm/pulso}} \approx \mathbf{1.851 \text{ pulsos}}$$

---

## 4. Arquitetura de Software e Controle

O código-fonte está localizado em `firmware_robo2wd_v2/` e possui organização modular:

### 4.1 Módulo [`config.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v2/config.h)
Centraliza todas as definições cinemáticas, limites elétricos, ganhos de PID e metas de trajetória:
```cpp
#define PPR_EFETIVO              4320.0f
#define DIAMETRO_RODA_MM         70.0f
#define DIST_RODAS_MM            120.0f
#define VELOCIDADE_CRUZEIRO_RPM  65.0f   // ~238 mm/s
#define VELOCIDADE_GIRO_RPM      40.0f   // ~1,4 s por curva de 90 graus
#define ESC_DEADBAND_US          50.0f   // Compensação de zona morta do ESC
#define K_SYNC                   0.05f   // Ganho do sincronismo cruzado
```

### 4.2 Módulo [`motor_control_mcpwm.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v2/motor_control_mcpwm.h)
Implementa a inicialização e controle direto do periférico ESP-IDF MCPWM:
* Timer de hardware de 1 MHz com período de 20.000 contagens ($20\text{ ms} = 50\text{ Hz}$).
* Funções `setMicrosecondsM1()` e `setMicrosecondsM2()` com resolução de $1\,\mu\text{s}$.
* Ponto neutro em $1500\,\mu\text{s}$ com limites seguros de proteção ($1150\,\mu\text{s}$ a $1850\,\mu\text{s}$).

### 4.3 Módulo [`encoder_reader_pcnt.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v2/encoder_reader_pcnt.h)
Configura as unidades de hardware PCNT com:
* Filtro anti-glitch de 1000 ns ativo nos pinos.
* Configuração cruzada de bordas e níveis para decodificação 4x direta no silício.
* Filtro Passa-Baixas Exponencial (EMA) no cálculo da velocidade angular para atenuar o ruído de quantização discreta da janela de amostragem de 20 ms:
  $$\text{RPM}_k = (\text{RPM}_{k-1} \times 0,70) + (\text{RPM}_{\text{bruto}} \times 0,30)$$

### 4.4 Módulo [`pid_controller.h`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v2/pid_controller.h)
Controlador Proporcional-Integral-Derivativo por roda:
* **Ganhos Calibrados:** $K_p = 1,8$, $K_i = 0,6$, $K_d = 0,02$.
* **Feedforward Dinâmico:** Aplica $\pm 50\,\mu\text{s}$ imediatamente ao iniciar movimento para superar a zona morta do ESC.
* **Anti-Windup:** Saturação de acumulador integral em $\pm 250$.

### 4.5 Malha de Sincronismo Cruzado (*Cross-Coupling*)
Implementada no laço principal de [firmware_robo2wd_v2.ino](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/firmware_robo2wd_v2/firmware_robo2wd_v2.ino):
1. **Em Linha Reta (`moverFrente`):**
   * Avalia $e_{\text{sync}} = t_1 - t_2$.
   * Se o motor esquerdo adiantar, seu comando é decrementado e o direito incrementado proporcionalmente por $K_{\text{sync}}$.
2. **Em Curvas Pivô (`girar`):**
   * Avalia $e_{\text{sync\_pivô}} = |t_1| - |t_2|$.
   * Força ambas as rodas a percorrerem exatamente a mesma extensão de arco no mesmo intervalo de tempo, mantendo o centro de rotação milimetricamente alinhado com o centro do eixo.

---

## 5. Guia de Operação e Comandos

### 5.1 Procedimento de Inicialização e Execução
1. Conecte o robô à porta USB do computador (reconhecida como `COM9`).
2. Abra o monitor serial configurado para **115200 baud**.
3. Ligue a chave de alimentação geral dos motores (bateria).
4. O firmware entrará no estado de armamento do ESC, mantendo sinal de $1500\,\mu\text{s}$ por 3 segundos.
5. Após o aviso no terminal, o robô aguarda 3 segundos em contagem regressiva e executa automaticamente a trajetória do quadrado (4 lados de 500 mm com giros de 90° à direita).
6. Ao final do percurso, os motores são desligados automaticamente no neutro e o robô emite o relatório de tempo e odometria na serial.

### 5.2 Ferramentas de Diagnóstico Isolado
O projeto conta com ferramentas de bancada para testes individuais de hardware:
* **[`diag_pcnt`](file:///C:/Users/luneque.junior/Projetos_IA/Teste_car_0/diag_pcnt/):** Permite verificar a contagem de pulsos por volta com as rodas girando manualmente com o robô desligado. Envie `z` ou `r` na serial para zerar contadores.

---

## 6. Procedimento de Compilação e Gravação

Caso necessite alterar qualquer parâmetro em `config.h`, siga os comandos abaixo no PowerShell:

### 6.1 Compilação do Firmware
```powershell
& "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn esp32:esp32:esp32 --export-binaries "C:\Users\luneque.junior\Projetos_IA\Teste_car_0\firmware_robo2wd_v2"
```

### 6.2 Gravação via `esptool.py` (Lembre-se de fechar o Monitor Serial antes)
```powershell
& "C:\Users\luneque.junior\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\5.3.1\esptool.exe" --chip esp32 --port COM9 --baud 921600 --before default-reset --after hard-reset write_flash -z --flash-mode dio --flash-freq 80m --flash-size 4MB 0x1000 "C:\Users\luneque.junior\Projetos_IA\Teste_car_0\firmware_robo2wd_v2\build\esp32.esp32.esp32\firmware_robo2wd_v2.ino.bootloader.bin" 0x8000 "C:\Users\luneque.junior\Projetos_IA\Teste_car_0\firmware_robo2wd_v2\build\esp32.esp32.esp32\firmware_robo2wd_v2.ino.partitions.bin" 0xe000 "C:\Users\luneque.junior\Projetos_IA\Teste_car_0\firmware_robo2wd_v2\build\esp32.esp32.esp32\boot_app0.bin" 0x10000 "C:\Users\luneque.junior\Projetos_IA\Teste_car_0\firmware_robo2wd_v2\build\esp32.esp32.esp32\firmware_robo2wd_v2.ino.bin"
```

---

## 7. Diretrizes para Expansões e Projetos Futuros

Como este projeto foi desenhado de forma estritamente modular, ele pode servir como base para:
1. **Adição de Perfil Trapezoidal de Velocidade:** Implementar rampa linear de aceleração para elevar a velocidade além de 70 RPM sem qualquer derrapagem (*wheel slip*).
2. **Integração com ROS 2 / micro-ROS:** Publicar odometria e velocidade em tópicos `/odom` e assinar comandos em `/cmd_vel` aproveitando o segundo núcleo do ESP32 via FreeRTOS.
3. **Fusão Sensorial com IMU (MPU-6050 / BNO-055):** Usar o barramento I2C livre nos pinos GPIO 21 (SDA) e GPIO 17/16 para fundir a orientação angular do giroscópio com os encoders, eliminando desvios em pisos irregulares.
