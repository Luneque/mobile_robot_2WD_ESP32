# Diário de Engenharia — Projeto Robô Diferencial 2WD com ESP32

---

## Registro Cronológico de Desenvolvimento e Homologação

### 1. Contexto Inicial e Objetivo
* **Meta do Projeto:** Desenvolver um firmware para robô de tração diferencial 2WD com microcontrolador ESP32-D0WD-V3 capaz de percorrer um quadrado fechado de 500 mm por lado com curvas de 90° em pivô central, sob malha fechada de controle.
* **Componentes:** Placa de expansão Goouuu 38P, 2 micromotores DC com redução por engrenagens e encoders magnéticos Hall (12 PPR), ESC escovado duplo operando com sinal PWM padrão servo (50 Hz).

---

### 2. Etapa 1: Implementação Inicial e Diagnóstico de Limitações (Versão 1)
* **Abordagem da V1:** Utilizou o periférico LEDC do ESP32 para o sinal dos motores e rotinas de interrupção de software (ISRs via `attachInterrupt`) nas bordas de GPIO dos encoders.
* **Problemas Identificados:**
  * O ESC apresentou atrasos e ruídos devido à comutação de software.
  * O microcontrolador sofria sobrecarga de interrupções por causa da alta taxa de transições nos encoders em quadratura, gerando perda intermitente de pulsos e imprecisão na odometria.
  * Assimetria nos motores fazia o robô derivar de rota em linhas retas.

---

### 3. Etapa 2: Migração para Periféricos Nativos de Silício (Versão 2 — MCPWM + PCNT)
Para solucionar em definitivo o gargalo de CPU e garantir precisão de hardware, foi realizada a reestruturação completa da arquitetura:
1. **MCPWM (Motor Control PWM):** Configurado timer dedicado de 1 MHz com resolução de $1\,\mu\text{s}$ para controle rigoroso dos canais do ESC a 50 Hz, sincronizado no zeramento do timer (`TEZ`).
2. **PCNT (Pulse Counter):** Configurados os contadores internos de pulso com decodificação direta de quadratura 4x no silício e filtro anti-ruído digital (*glitch filter*) de 1000 ns.
3. **Mapeamento de Pinos Físicos Homologado:**
   * Motor Esquerdo (CH1): `GPIO 19`
   * Motor Direito (CH2): `GPIO 18`
   * Encoders Esquerdo: Fases A/B nos pinos `GPIO 23` e `GPIO 22`
   * Encoders Direito: Fases A/B nos pinos `GPIO 33` e `GPIO 32`

---

### 4. Etapa 3: Calibração Experimental da Relação de Redução e Resolução
* Inicialmente, a documentação preliminar indicava uma redução incerta entre 1:45 e 1:90.
* Foi construído um utilitário de validação direta em hardware (`diag_pcnt`).
* **Teste de Rotação Manual 360° com PCNT:**
  * Ao girar as rodas 360° manualmente, o PCNT registrou com exatidão **~4.425 e ~4.489 pulsos**.
  * Esse teste comprovou a redução real de **1:90** com resolução efetiva de **4.320 pulsos por volta completa** ($12 \text{ PPR} \times 90 \times 4\text{x}$).
  * O motivo pelo qual medições anteriores por software retornavam ~2160 era a perda de metade das bordas pelas interrupções de CPU na V1.

---

### 5. Etapa 4: Eliminação da Assimetria de RPM e Controle de Trajetória
Com a resolução exata parametrizada, foi solucionada a diferença de rotação entre as rodas através de três medidas:
1. **Filtro Passa-Baixas Exponencial (EMA):** Suavização do cálculo de RPM instantâneo na janela de amostragem de 20 ms ($0,70 \times \text{RPM}_{k-1} + 0,30 \times \text{RPM}_{\text{atual}}$).
2. **Feedforward de Zona Morta do ESC:** Injeção direta de $\pm 50\,\mu\text{s}$ configurável em `config.h` para vencer o atrito estático imediatamente.
3. **Sincronismo Cruzado (*Cross-Coupling Synchronization*):**
   * Malha de acoplamento mútuo atuando em tempo real na rota reta ($e_{\text{sync}} = t_1 - t_2$) e nas curvas pivô ($e_{\text{sync\_pivô}} = |t_1| - |t_2|$).
   * Se qualquer motor encontrar resistência mecânica maior, o firmware rebalanceia a potência de ambos os motores instantaneamente.

---

### 6. Etapa 5: Otimização de Desempenho e Homologação Final
* A velocidade de avanço em cruzeiro foi elevada com segurança de **45 RPM para 65 RPM** ($\approx 238\text{ mm/s}$, ganho de **+44%** de agilidade), reduzindo o tempo de cada lado de 500 mm para apenas ~2,1 s sem derrapagem.
* As curvas de 90° foram calibradas para **40 RPM** (~1,4 s por giro).
* O comportamento dinâmico final no piso atingiu fechamento perfeito do quadrado geométrico de 500 mm com estabilidade e repetibilidade completas.
