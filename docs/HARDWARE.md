# Hardware — ESP32 ↔ Pixhawk 2.4.8

## A placa

**ESP32 (NodeMCU-32S, ESP32-WROOM-32).** Não ESP8266.

O projeto nasceu pedindo um NodeMCU ESP8266, e ele não serve — não por gosto,
por três motivos que não têm contorno:

1. **Não existe micro-ROS para ESP8266.** O `micro_ros_arduino` lista Portenta
   H7, Nano RP2040, Teensy, ESP32 Dev Module e OpenCR como oficiais, mais
   Due/Zero/Pico/XIAO da comunidade. O ESP8266 não aparece em nenhuma das duas
   listas, e não há port do cliente Micro-XRCE-DDS para o toolchain
   `xtensa-lx106`.
2. **A RAM não fecha.** O ESP8266 deixa ~40 kB de heap livre com o WiFi ligado.
   O cliente micro-ROS sozinho pede mais do que isso, antes do parser MAVLink e
   dos buffers.
3. **Só há uma UART bidirecional.** A UART0 é a mesma do bootloader e do console
   USB. O MAVLink teria que disputar os mesmos pinos com a gravação e com todo
   `Serial.print` de depuração.

O ESP32-WROOM resolve os três de uma vez: 520 kB de SRAM, três UARTs de
hardware, e suporte de primeira classe no micro-ROS. Mesmo formato de placa,
mesma faixa de preço, mesmo cabo USB.

> **ESP32-WROVER não.** No WROVER os GPIO16 e GPIO17 são usados pela PSRAM, que
> são exatamente os pinos default da UART2 abaixo. Ou use um WROOM, ou remapeie
> os pinos em `firmware/include/config.h` (a UART2 do ESP32 passa pela matriz de
> GPIO e aceita quase qualquer par).

## A ligação

TELEM1 da Pixhawk é um conector DF13 de 6 vias. A numeração começa no pino
marcado na placa:

| TELEM1 | Sinal | Vai para o ESP32 |
|---|---|---|
| 1 | +5 V | `VIN` — *leia a seção de energia antes* |
| 2 | TX (sai da Pixhawk) | `GPIO16` (RX2) |
| 3 | RX (entra na Pixhawk) | `GPIO17` (TX2) |
| 4 | CTS | não ligar |
| 5 | RTS | não ligar |
| 6 | GND | `GND` |

```
   Pixhawk 2.4.8                     ESP32 NodeMCU-32S
   ┌──────────────┐                  ┌────────────────┐
   │       TELEM1 │                  │                │
   │   1  +5V ────┼──── (ver abaixo) ┼──► VIN         │
   │   2  TX  ────┼──────────────────┼──► GPIO16 RX2  │
   │   3  RX  ◄───┼──────────────────┼─── GPIO17 TX2  │
   │   4  CTS  ×  │                  │                │
   │   5  RTS  ×  │                  │                │
   │   6  GND ────┼──────────────────┼──► GND         │
   └──────────────┘                  └────────────────┘
```

**TX de um vai no RX do outro.** Trocar os dois é o erro mais comum e o mais
silencioso: nada esquenta, nada acende, o `/ozzy/diagnostics` mostra
`mav_rx_hz: 0` para sempre.

**Não é preciso conversor de nível.** A Pixhawk e o ESP32 usam lógica de 3,3 V
nos dois lados. Um conversor aqui só acrescenta um ponto de falha.

**Não ligue nada em CTS/RTS.** O firmware não usa controle de fluxo, e o
`BRD_SER1_RTSCTS=0` da [configuração do ArduPilot](ARDUPILOT.md) desliga isso do
outro lado. Com os fios ligados e o parâmetro errado, a serial trava depois de
alguns segundos de tráfego.

**Nunca ligue o pino `3V3` do ESP32 no pino 1 da TELEM1.** São 5 V contra uma
saída de regulador. Isso queima o ESP32.

## Energia — a parte que morde

A TELEM1 entrega 5 V vindos do rail do módulo de potência, com uns 1 A no total
e compartilhados. O ESP32 consome ~250 mA em média e **puxa picos acima de
500 mA** nas rajadas de transmissão do WiFi.

- **Na bancada** dá para alimentar pela TELEM1. Ponha um capacitor eletrolítico
  de 470 µF entre `VIN` e `GND`, o mais perto possível da placa. Sem ele, o pico
  de WiFi afunda o rail de 5 V, e o que aparece é a Pixhawk reiniciando "sozinha"
  no meio de um teste — sem nenhuma mensagem apontando para o ESP32.
- **Em voo, use um BEC de 5 V separado**, ligado direto na bateria, com o GND
  em comum com a Pixhawk. Alimentar o rádio do link pelo mesmo regulador que
  alimenta a FMU é economizar o fio errado.

## Posicionamento

Mantenha a antena do ESP32 a pelo menos 10 cm do módulo de GPS/bússola. A
bússola do GPS é sensível a corrente e o ESP32 pulsa meio ampère a cada rajada;
o sintoma é HDOP piorando e "Compass variance" no `/ozzy/statustext` só quando o
link está ativo.

## Gravação

O firmware é gravado pela USB, que usa a UART0 — independente da UART2 que fala
com a Pixhawk. Dá para gravar com tudo ligado, e o `Serial` de depuração
(115200) continua livre no monitor da USB. Foi para isso que a segunda UART foi
escolhida.

## Lista de compras

| Item | Observação |
|---|---|
| ESP32 NodeMCU-32S (WROOM-32) | não WROVER, não ESP8266 |
| Cabo DF13 6 vias | costuma vir com a Pixhawk |
| Capacitor eletrolítico 470 µF / 10 V | junto ao `VIN` do ESP32 |
| BEC 5 V / 1 A | só para voo |
