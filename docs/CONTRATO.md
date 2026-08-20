# O contrato do `ozzy_bridge`

Este arquivo é a interface entre o firmware que roda no ESP32 e a máquina de
estados que roda no seu computador. **Mudar qualquer coisa aqui é mudar as duas
pontas ao mesmo tempo** — e as duas pontas são compiladas por ferramentas
diferentes, em máquinas diferentes, em momentos diferentes. Por isso o contrato
mora num arquivo só, e não espalhado entre o `main.cpp` e um nó de Python.

---

## Por que este projeto existe

A Pixhawk 2.4.8 é FMUv2/FMUv3: um STM32F427 com 1 MB de flash útil (2 MB nas
revisões que passam no teste de silício). O ArduPilot que cabe ali **não inclui
o `AP_DDS`** — a biblioteca que fala XRCE-DDS nativamente e que, numa
controladora moderna, dispensaria completamente uma ponte. Não há espaço.

Sobra o MAVLink pela TELEM1. Alguém precisa traduzir MAVLink em tópicos ROS 2,
e esse alguém é o ESP32.

```
  computador (ROS 2 + máquina de estados)
        │  DDS
        ├── MicroXRCEAgent udp4 -p 8888
        │        ╎ WiFi (UDP)
        │   ESP32 — firmware/ deste repositório
        │     ├─ cliente micro-ROS (rclc)
        │     └─ parser MAVLink v2
        │        ╎ UART2, 3V3, 115200 8N1
        └── Pixhawk 2.4.8 · ArduCopter 4.5.x · TELEM1
```

Note que o `MicroXRCEAgent` é **o mesmo binário** que o workspace já usa para o
PX4 (pinado em `v3.0.1` no `env/<perfil>.yaml` do `evtol-dev`) e a **mesma porta
8888** do `templates/scripts/agent.sh`. Não há ferramenta nova para instalar do
lado do computador.

---

## Frames e unidades

O `ARCHITECTURE.md` do workspace diz que nada acima do `drone_lib` toca NED/FRD.
Aqui o `ozzy_bridge` **ocupa o lugar do `drone_lib`**: ele é a fronteira. O
MAVLink é NED/FRD dos dois lados do fio; tudo que sai em tópico ROS 2 já está
convertido.

| | Mundo | Corpo |
|---|---|---|
| Dentro do firmware, antes da conversão | NED | FRD |
| Em todo tópico `/ozzy/...` | **ENU** | **FLU** |

A conversão vive em [`firmware/include/frames.h`](../firmware/include/frames.h),
num arquivo só, sem cópias. Posição e velocidade: `(x,y,z)_ENU = (y, x, -z)_NED`.
Atitude: `roll_FLU = roll_FRD`, `pitch_FLU = -pitch_FRD`, `yaw_FLU = π/2 - yaw_FRD`.

Unidades: metros, radianos, m/s, rad/s — como manda o contrato do workspace.
Nenhum campo em graus, centímetros ou milivolts, que é como o MAVLink entrega
quase tudo.

**Todo header sai com `frame_id` preenchido.** `frame_id` vazio é bug.

---

## Publicações — ESP32 → ROS 2

| Tópico | Tipo | `frame_id` | Taxa | Vem de (MAVLink) |
|---|---|---|---|---|
| `/ozzy/pose` | `geometry_msgs/PoseStamped` | `map` | 20 Hz | `LOCAL_POSITION_NED` + `ATTITUDE` |
| `/ozzy/twist` | `geometry_msgs/TwistStamped` | `map` | 20 Hz | `LOCAL_POSITION_NED` + `ATTITUDE` |
| `/ozzy/battery` | `sensor_msgs/BatteryState` | `base_link` | 2 Hz | `SYS_STATUS` |
| `/ozzy/gps` | `sensor_msgs/NavSatFix` | `base_link` | 2 Hz | `GLOBAL_POSITION_INT` + `GPS_RAW_INT` |
| `/ozzy/armed` | `std_msgs/Bool` | — | 2 Hz | `HEARTBEAT.base_mode` |
| `/ozzy/mode` | `std_msgs/String` | — | 2 Hz | `HEARTBEAT.custom_mode` |
| `/ozzy/statustext` | `std_msgs/String` | — | evento | `STATUSTEXT` |
| `/ozzy/diagnostics` | `diagnostic_msgs/DiagnosticStatus` | — | 1 Hz | interno |

### QoS

`/ozzy/pose`, `/ozzy/twist`, `/ozzy/battery` e `/ozzy/gps` são **BEST_EFFORT**.
`/ozzy/armed`, `/ozzy/mode`, `/ozzy/statustext` e `/ozzy/diagnostics` são
**RELIABLE**.

O critério: num link de rádio, QoS confiável transforma um pacote perdido em
retransmissão, e a retransmissão atrasa o dado *seguinte* — que já é mais novo e
mais útil. Telemetria de alta taxa prefere perder; evento prefere esperar.

O preço é uma armadilha, e ela é silenciosa: **um assinante RELIABLE não recebe
nada de um publicador BEST_EFFORT**. O tópico aparece no `ros2 topic list`, o
`ros2 topic info` mostra o publicador, e nenhuma mensagem chega — sem erro em
lugar nenhum. Verificado em Humble:

```bash
ros2 topic echo /ozzy/pose geometry_msgs/msg/PoseStamped --qos-reliability best_effort  # imprime
ros2 topic echo /ozzy/pose geometry_msgs/msg/PoseStamped --qos-reliability reliable     # silêncio
```

O default do `rclpy` e do `rclcpp` é RELIABLE. Todo nó que assinar os quatro
primeiros precisa declarar BEST_EFFORT explicitamente — veja `FAST_QOS` em
[`ozzy_bridge/link_monitor.py`](../ozzy_bridge/link_monitor.py).

### Duas observações que economizam uma tarde

**`/ozzy/twist` mistura frames de propósito.** O `linear` é velocidade de mundo
em ENU (`frame_id: map`); o `angular` são as taxas de corpo em FLU. É
exatamente o que o `mavros` faz em `local_position/velocity_local`, e é o que os
dados do MAVLink permitem: `LOCAL_POSITION_NED` só tem velocidade de mundo e
`ATTITUDE` só tem taxa de corpo. Preencher `angular` com uma conversão inventada
seria pior do que documentar a mistura.

**`/ozzy/statustext` não é enfeite.** Quando o ArduPilot recusa armar, o motivo
("PreArm: Need 3D Fix", "PreArm: Compass not calibrated") sai por `STATUSTEXT` e
por nenhum outro lugar. Sem este tópico, a máquina de estados vê um `arm` que
falha e nada mais. Deixe um `ros2 topic echo /ozzy/statustext` aberto na bancada.

### `/ozzy/mode`

String, com o nome do modo do ArduCopter — `STABILIZE`, `ALT_HOLD`, `GUIDED`,
`LOITER`, `RTL`, `LAND`, `BRAKE`, `POSHOLD`, `GUIDED_NOGPS`. Modo desconhecido
sai como `MODE_<n>`, nunca vazio.

### `/ozzy/diagnostics`

`DiagnosticStatus` com `level` (`OK`/`WARN`/`ERROR`), `name = "ozzy_bridge"` e
os pares:

| Chave | Significado |
|---|---|
| `mav_rx_hz` | mensagens MAVLink/s vindas da Pixhawk |
| `mav_parse_err` | erros de parsing acumulados (ruído no fio, baud errado) |
| `mav_last_hb_ms` | há quanto tempo veio o último `HEARTBEAT` |
| `ros_cmd_age_ms` | idade do último comando vindo do ROS 2 |
| `wifi_rssi` | dBm |
| `free_heap` | bytes livres no ESP32 |

`ERROR` quando a Pixhawk está muda há mais de 3 s. `WARN` quando o RSSI passa de
-80 dBm ou o heap cai abaixo de 20 kB — os dois anunciam a queda antes dela.

---

## Assinaturas — ROS 2 → ESP32

| Tópico | Tipo | Efeito |
|---|---|---|
| `/ozzy/cmd_vel` | `geometry_msgs/TwistStamped` | velocidade em GUIDED (`linear` ENU mundo, `angular.z` taxa de guinada FLU) |
| `/ozzy/setpoint` | `geometry_msgs/PoseStamped` | posição-alvo em GUIDED (ENU, `frame_id: map`) |
| `/ozzy/set_mode` | `std_msgs/String` | pede o modo pelo nome (`GUIDED`, `LAND`, `RTL`, ...) |
| `/ozzy/takeoff` | `std_msgs/Float32` | decolagem para a altitude dada, em metros |

| Serviço | Tipo | Efeito |
|---|---|---|
| `/ozzy/arm` | `std_srvs/SetBool` | arma (`true`) ou desarma (`false`) |

### Por que modo é tópico e não serviço

Um serviço devolveria o `COMMAND_ACK`, que diz apenas *"o comando foi aceito"* —
não que o veículo entrou no modo. Quem responde a essa pergunta é o `HEARTBEAT`
seguinte. Então o padrão correto para a máquina de estados é o mesmo nos dois
casos: publique em `/ozzy/set_mode` e **espere `/ozzy/mode` mudar**. Um serviço
aqui só daria uma confirmação que engana.

`arm` continua serviço porque a máquina de estados precisa de um ponto de
sincronização explícito antes de decolar, e porque um `arm` perdido é caro.

### Por que só mensagens padrão

Nenhum tipo custom, de propósito. Tipo custom em micro-ROS obriga a recompilar
a biblioteca do cliente com `extra_packages` — o firmware passaria a depender de
uma geração local que ninguém consegue reproduzir seis meses depois. É a mesma
classe de bug que os manifestos do `evtol-dev` existem para impedir. Se um dia
um `ozzy_msgs` for inevitável, ele entra no `evtol.repos` como pacote pinado e o
`platformio.ini` passa a apontar para a tag — não antes.

---

## O orçamento de entidades

O micro-ROS aloca tudo estaticamente. Os limites default do
`micro_ros_platformio` e o que este firmware usa:

| Recurso | Limite | Em uso |
|---|---|---|
| `RMW_UXRCE_MAX_PUBLISHERS` | 10 | 8 |
| `RMW_UXRCE_MAX_SUBSCRIPTIONS` | 5 | 4 |
| `RMW_UXRCE_MAX_SERVICES` | 5 | 1 |
| `RMW_UXRCE_MAX_NODES` | 1 | 1 |
| handles do executor | — | 7 (4 subs + 2 timers + 1 serviço) |

**Acrescentar um assinante a mais estoura o limite**, e o sintoma é um
`rclc_subscription_init_default` devolvendo erro num `RCCHECK` — no boot, antes
de qualquer voo, o que é a hora boa de descobrir. Quem precisar do sexto
assinante mexe no `board_microros_user_meta` do `platformio.ini`, e paga em RAM.

---

## As três camadas de failsafe

Ponte de rádio que some é o modo normal de falhar, não a exceção. Três defesas
independentes, e nenhuma delas depende das outras duas:

**1. ROS 2 sumiu — o ESP32 percebe.** Se nenhum comando chega em
`ROS_CMD_TIMEOUT_MS` (500 ms), o firmware para de retransmitir o setpoint e
manda velocidade zero uma vez. O veículo fica parado em GUIDED, não continua na
última velocidade recebida. Sem isso, um cabo de rede desconectado no computador
vira um drone que atravessa a arena na última velocidade comandada.

**2. O ESP32 sumiu — o ArduPilot percebe.** O firmware manda `HEARTBEAT` de GCS
a 1 Hz. Com `FS_GCS_ENABLE=1`, o ArduPilot dispara o failsafe dele quando esses
batimentos param. Configuração em [ARDUPILOT.md](ARDUPILOT.md).

**3. Tudo sumiu — o piloto percebe.** O rádio continua ligado e armado durante
todo voo autônomo. Mudar de modo no rádio tira o veículo de GUIDED
imediatamente, e nada neste repositório pode impedir isso. **Essa é a defesa que
realmente conta**; as outras duas existem para que ela quase nunca seja usada.

---

## Carimbo de tempo

O ESP32 não tem relógio. No boot ele nasce em 1970, e um `PoseStamped` com
`stamp` de 1970 quebra `tf2`, `rosbag` e qualquer coisa que compare tempos.

O firmware chama `rmw_uros_sync_session()` ao conectar no agente e usa
`rmw_uros_epoch_nanos()` em todo header. Se a sincronização falhar, ele tenta de
novo a cada 5 s e **publica mesmo assim**, marcando `WARN` no
`/ozzy/diagnostics` — dado com carimbo ruim é melhor que silêncio, desde que o
silêncio esteja anunciado em algum lugar.

---

## O que este repositório deliberadamente não faz

- **Não expõe uma classe `Drone`.** A máquina de estados assina os tópicos
  diretamente. Uma fachada no formato do `drone_lib` — para que um estado
  escrito para PX4 rode aqui sem mudança — é o passo natural seguinte, e não
  cabe na v0.1.0.
- **Não faz missões, waypoints nem `AUTO`.** Só GUIDED. Missão é
  responsabilidade da máquina de estados, que é onde o time já sabe escrevê-la.
- **Não tenta ser um `mavros`.** Traduz o subconjunto de MAVLink que a FSM usa,
  e nada além.
