# ozzy_bridge

**Ponte entre o ROS 2 do seu computador e uma Pixhawk 2.4.8 rodando ArduPilot,
com um ESP32 no meio.**

```
  computador (ROS 2 + máquina de estados)
        │  DDS
        ├── MicroXRCEAgent udp4 -p 8888
        │        ╎ WiFi (UDP)
        │   ESP32 — firmware/
        │     ├─ cliente micro-ROS (rclc)
        │     └─ parser MAVLink v2
        │        ╎ UART2, 3V3, 115200 8N1
        └── Pixhawk 2.4.8 · ArduCopter · TELEM1
```

Os outros projetos da equipe usam PX4 e falam com ele pelo `drone_lib`. Este
usa **ArduPilot**, e a Pixhawk 2.4.8 é antiga demais para o `AP_DDS` — o
STM32F427 tem 1 MB de flash útil (2 MB nas revisões boas) e a biblioteca de
ROS 2 nativo do ArduPilot não cabe. Sobra MAVLink pela TELEM1, e alguém precisa
traduzir. Esse alguém é o ESP32.

## ⚠️ Estado: compila, nunca voou

Nada aqui foi validado em hardware. O CI compila as duas metades — o pacote ROS 2
em Humble e Jazzy, e o firmware nos dois ambientes do PlatformIO — e é só isso
que está provado.

Os números que aparecem nos comentários (o buffer da UART enchendo em ~22 ms, o
consumo de pico do ESP32, as taxas de stream) são **contas de projeto, não
medições** — ao contrário dos perfis do `env/` do `evtol-dev`, onde "verificado"
quer dizer verificado. Quando a placa existir, meça e corrija aqui.

A ordem de validação está em [docs/CONTRATO.md](docs/CONTRATO.md); comece pela
bancada, com as hélices fora.

## Por que ESP32 e não o ESP8266

O projeto nasceu pedindo um NodeMCU ESP8266. Ele não serve, e não é questão de
preferência:

- **Não existe micro-ROS para ESP8266.** Nem oficial nem da comunidade — não há
  port do cliente Micro-XRCE-DDS para o toolchain `xtensa-lx106`.
- **A RAM não fecha.** ~40 kB de heap livre com o WiFi ligado; o cliente
  micro-ROS sozinho pede mais que isso.
- **Só há uma UART bidirecional**, e é a mesma do bootloader e do console USB.

O ESP32-WROOM resolve os três: 520 kB de SRAM, três UARTs, suporte oficial no
micro-ROS. Mesmo formato, mesmo preço, mesmo cabo. Detalhes e as pegadinhas do
WROVER em [docs/HARDWARE.md](docs/HARDWARE.md).

## Começando

**1. Ligue o hardware** — [docs/HARDWARE.md](docs/HARDWARE.md). Três fios, sem
conversor de nível. Leia a seção de energia antes de alimentar pela TELEM1.

**2. Configure o ArduPilot** — [docs/ARDUPILOT.md](docs/ARDUPILOT.md).
`SERIAL1_PROTOCOL`, `SERIAL1_BAUD`, as taxas de stream e o failsafe de GCS.

**3. Grave o firmware.** O PlatformIO tem que vir do **instalador oficial**, não
de `pip install platformio`:

```bash
curl -fsSL -o /tmp/get-platformio.py \
  https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 /tmp/get-platformio.py
export PATH="$PATH:$HOME/.platformio/penv/bin"
```

O `micro_ros_platformio` compila micro-ROS da fonte dentro do virtualenv
próprio do PlatformIO (`~/.platformio/penv`), que só o instalador oficial cria —
ele isola o `empy 3.3.4` que o micro-ROS exige do `empy 4.x` do sistema. Com o
PlatformIO vindo do pip, tudo funciona até o micro-ROS começar a compilar, e aí
sai `cannot open .platformio/penv/bin/activate`, que não menciona pip nem
instalação.

```bash
cd firmware
cp include/secrets.h.example include/secrets.h   # SSID, senha e IP do agente
pio run -e humble -t upload                      # ou -e jazzy
pio device monitor
```

A primeira compilação demora vários minutos: ela baixa o toolchain do ESP32 e
compila micro-ROS da fonte.

O ambiente (`humble`/`jazzy`) tem que casar com a distro da máquina que roda a
máquina de estados. Cliente e workspace de distros diferentes conectam,
aparecem no `ros2 node list` e não trocam mensagem nenhuma — sem erro nenhum.

**4. Suba o lado do computador:**

```bash
ros2 launch ozzy_bridge ozzy.launch.py
```

Isso põe de pé o `MicroXRCEAgent` (o mesmo binário e a mesma porta que o
workspace já usa para o PX4) e o `link_monitor`, que diz de uma vez o que está
funcionando e o que não está.

**5. Prove que obedece, antes de a FSM entrar:**

```bash
ros2 run ozzy_bridge teleop
```

## O contrato

Tudo que a máquina de estados enxerga está em
**[docs/CONTRATO.md](docs/CONTRATO.md)** — tópicos, tipos, frames, taxas, QoS e
os três níveis de failsafe. Em resumo:

| Publica | | Assina |
|---|---|---|
| `/ozzy/pose` · `/ozzy/twist` | | `/ozzy/cmd_vel` · `/ozzy/setpoint` |
| `/ozzy/battery` · `/ozzy/gps` | | `/ozzy/set_mode` · `/ozzy/takeoff` |
| `/ozzy/armed` · `/ozzy/mode` | | serviço `/ozzy/arm` |
| `/ozzy/statustext` · `/ozzy/diagnostics` | | |

Tudo em **ENU/FLU**, metros, radianos e segundos. O ESP32 é a fronteira: NED/FRD
não sai dele, exatamente como o `drone_lib` faz do lado do PX4.

Duas coisas que economizam uma tarde:

- **`/ozzy/pose` e `/ozzy/twist` são BEST_EFFORT.** Um assinante com o QoS
  default do `rclpy` (RELIABLE) não casa: o tópico aparece no `ros2 topic list`,
  o `ros2 topic info` mostra o publicador, e nenhuma mensagem chega. Nenhum erro
  em lugar nenhum. Veja `FAST_QOS` em
  [ozzy_bridge/link_monitor.py](ozzy_bridge/link_monitor.py).
- **`/ozzy/statustext` é onde o ArduPilot explica por que não armou.** Deixe um
  `echo` aberto.

## Layout

| Caminho | O que é |
|---|---|
| [docs/CONTRATO.md](docs/CONTRATO.md) | **Comece por aqui** — a interface entre as duas metades |
| [docs/HARDWARE.md](docs/HARDWARE.md) | Ligação, energia, escolha da placa |
| [docs/ARDUPILOT.md](docs/ARDUPILOT.md) | Parâmetros da Pixhawk |
| [firmware/](firmware/) | Projeto PlatformIO do ESP32 — **não é pacote colcon** |
| [firmware/include/frames.h](firmware/include/frames.h) | A única conversão NED/FRD ↔ ENU/FLU |
| [ozzy_bridge/](ozzy_bridge/) | Pacote ROS 2: `link_monitor`, `teleop` |
| [scripts/agent.sh](scripts/agent.sh) | Sobe o agente sozinho |

## O que ainda não existe

- **Uma fachada no formato do `drone_lib`**, para que um estado escrito para
  PX4 rode aqui sem mudança. Hoje a FSM assina os tópicos direto.
- **Voo sem GPS.** `GUIDED` exige estimativa de posição; em arena fechada isso
  quer dizer fluxo óptico ou posição externa, configurados em `EK3_SRC1_*`.
  Decida isso *antes* de escrever os estados — muda metade do contrato.
- **Missões, waypoints, `AUTO`.** Só `GUIDED`. Missão é da máquina de estados.
