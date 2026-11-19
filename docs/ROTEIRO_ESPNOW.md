# Roteiro de Validação do Enlace ESP-NOW

Validar a ponte **robô (ESP32) ↔ gateway (ESP32) ↔ PC**. Pré-requisito: o
gateway compilado e gravado, e o robô com o firmware V4 (já validado em bancada).

Tempo estimado: ~20 min.

---

## 0. Pré-requisitos

- [ ] Um ESP32 auxiliar disponível (além do ESP32 do robô). O gateway **validado** foi um ESP-WROOM-32 clássico.
- [ ] Cabo USB para o ESP32.
- [ ] `gateway_espnow/gateway_espnow.ino` compilado (validado: 885.292 bytes, 67% flash).
- [ ] Robô com o firmware V4 gravado e no Wi-Fi (`robo.local`).
- [ ] Python com `pyserial` e `websockets`: `python -m pip install pyserial websockets rich`.

Verifique as portas antes de começar:

```bash
python -c "import serial.tools.list_ports as lp; print([p.device for p in lp.comports()])"
```

Esperado: uma porta para o robô (ex.: `COM9`) e outra para o gateway (ex.: `COM10`).

---

## 1. Gravar o gateway no ESP32

1. Abra `gateway_espnow/gateway_espnow.ino` na Arduino IDE.
2. Placa: **ESP32 Dev Module** (o gateway validado foi um ESP-WROOM-32 clássico, em `COM18`).
3. Selecione a porta do ESP32 e grave.
4. Abra o Monitor Serial em **115200 baud** e confira:

```
DBG:Iniciando Gateway ESP-NOW...
MAC: AA:BB:CC:DD:EE:FF          <-- anote este MAC
DBG:Canal ESP-NOW: 11
DBG:Peer broadcast registrado
DBG:Gateway pronto. Aguardando dados...
```

5. Anote o **MAC**. Se quiser endereçamento dedicado (recomendado), cole-o em
   `firmware_robo2wd_v4/config.h`:

```cpp
#define ESPNOW_GATEWAY_MAC   {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}
```

   e regrave o robô. Sem isso, o robô usa broadcast (`FF:FF:...`), que funciona
   mas não é dedicado.

---

## 2. Alinhar o canal (ponto que mais quebra)

O canal do gateway é fixo; o canal do robô é o **canal do AP**.

1. Com o robô ligado e conectado, leia o canal real no Serial do robô:

```
[ESPNOW] Canal atual da interface: 11
```

2. Garanta que `ESPNOW_CHANNEL` em `gateway_espnow/gateway_espnow.ino` é **o mesmo valor**.
   - O valor atual é `11` (medido no AP do projeto). Se o seu AP usar outro canal,
     ajuste e regrave o gateway.

> **Por que importa:** com canais divergentes o envio falha com
> `E (...) ESPNOW: Peer channel is not equal to the home channel, send fail!`
> (era o comportamento observado antes do ajuste: ~39 erros/s).

> **Armadilha:** no lado do robô, **não** chamar `esp_wifi_set_channel()` depois
> da conexão Wi-Fi — isso derruba o enlace com o AP e trava o `setup()`. O robô
> registra o peer com canal 0 (canal atual da interface), que é o correto.

---

## 3. Subir a ponte no PC

Com o gateway conectado ao PC:

```bash
python robo_monitor/gateway_espnow.py --port COM10 --csv telemetria_espnow.csv
```

Substitua `COM10` pela porta do gateway. Esperado:

```
[csv] telemetria_espnow.csv
[serial] conectando a COM10 @ 115200
[gateway] MAC: AA:BB:CC:DD:EE:FF
[gateway] DBG:Canal ESP-NOW: 11
[pronto] comandos: start | stop | pause | reset | dump | sair
```

Se as linhas `MAC:`/`DBG:` não aparecerem, o gateway precisa de um reset
(botão EN) ou a porta está errada.

---

## 4. Teste de telemetria (robô → PC)

1. Digite `start` no prompt da ponte.
2. O robô deve iniciar a trajetória. A ponte deve imprimir a telemetria:

```
[pc->robo] start
[robo] {"ts":12345,"st":"RETA","sd":1,"of":4,"yaw":0.01,"r1":57.9,"r2":58.1,...}
[robo] {"ts":12545,"st":"RETA","sd":1,...}
```

3. Confira se `telemetria_espnow.csv` está sendo gravado com as linhas.

**Critério de aceite:** telemetria fluindo a ~20 Hz em movimento (2 Hz parado), sem mensagens
`DBG:ERRO ao enviar CMD` no Monitor Serial do gateway.

---

## 5. Teste de comando (PC → robô)

1. Com o robô em execução, envie `stop`:

```
[pc->robo] stop
```

2. Esperado:
   - Serial do robô: `[LOOP] comando ESP-NOW: stop` e `[PARADA REMOTA] Trajetoria interrompida!`
   - Serial do gateway: `DBG:CMD enviado com sucesso`
   - A ponte para de receber `RETA` e passa a receber `CONCLUIDO`.

3. Repita com `start` (reinicia), `pause` (para) e `reset` (reinicia).

---

## 6. Teste do dump do gravador via ESP-NOW

1. Inicie uma trajetória (`start`).
2. Aguarde o término (~45 s).
3. Envie `dump`.
4. Esperado: a ponte imprime as linhas `REC,...` do buffer, e o CSV gravado
   contém o teste completo.

> O buffer é volátil: o dump precisa acontecer antes de desligar/reiniciar o robô.

---

## 7. Verificação de robustez

- [ ] Robô executa o quadrado com Wi-Fi **e** ESP-NOW ativos, sem reiniciar.
- [ ] Dashboard Wi-Fi (`http://robo.local/`) continua respondendo durante o enlace.
- [ ] Desligar o gateway (desconectar o USB) **não** derruba o robô nem o Wi-Fi.
- [ ] Reconectar o gateway retoma a telemetria sem reiniciar o robô.

---

## 8. Solução de problemas

| Sintoma | Causa provável | Ação |
|---|---|---|
| `Peer channel is not equal to the home channel` | Canal do gateway ≠ canal do AP | Ajustar `ESPNOW_CHANNEL` no gateway e regravar |
| Gateway não imprime `MAC:` | Porta errada / precisa de reset | Trocar a porta; apertar EN |
| Ponte não recebe nada | Robô e gateway em canais diferentes, ou robô sem firmware novo | Verificar seção 2; regravar o robô |
| `DBG:ERRO ao enviar CMD` | Robô fora de alcance ou desligado | Aproximar; conferir alimentação |
| Dump viaja pelo canal errado | Pedido veio por WebSocket mas voltou por ESP-NOW | Comportamento esperado: o dump volta pelo canal que recebeu o pedido |
| Robô trava ao ligar | `esp_wifi_set_channel()` após o connect | Já corrigido no firmware; não reintroduzir |
| CSV da ponte vazio | Comando `start` não foi enviado | Enviar `start` pelo prompt da ponte |

---

## 9. Registro do resultado

Após rodar, anotar aqui (ou no Diário de Engenharia):

- Canal do AP medido: ____
- MAC do gateway: ____
- Telemetria robô → PC: [ ] OK [ ] falhou
- Comandos PC → robô: [ ] OK [ ] falhou
- Dump via ESP-NOW: [ ] OK [ ] falhou
- Observações: ______________________________________________

---

## Referências

- `docs/FSD.md` — seção de ESP-NOW (escopo e restrição de canal).
- `docs/DIARIO_ENGENHARIA.md` — Etapa 13.
- `TODO.md` — item P1.1 (restrições registradas).
- `robo_monitor/gateway_espnow.py` — ponte serial.
