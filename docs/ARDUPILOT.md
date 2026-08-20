# Configuração do ArduPilot

Todos os outros projetos da equipe usam PX4. Este usa ArduPilot, e as diferenças
que importam aqui não são de opinião — são de interface.

## Qual firmware gravar

A Pixhawk 2.4.8 é a placa FMUv2/FMUv3: STM32F427. As duas variantes usam o mesmo
silício; a diferença é a flash utilizável — 1 MB na FMUv2, 2 MB na FMUv3. Quase
toda 2.4.8 vendida hoje tem os 2 MB e roda a build **Pixhawk1 (fmuv3)**.

Confira antes de gravar: conecte no Mission Planner e leia a versão declarada no
boot. Se a placa só aceitar a build de 1 MB, ela funciona, mas com módulos
cortados por falta de espaço.

**Em nenhuma das duas o `AP_DDS` está incluído.** É por isso que este
repositório existe: sem `AP_DDS`, não há ROS 2 nativo, e a única interface é
MAVLink pela TELEM1.

## Parâmetros

Ligue pelo USB no Mission Planner (ou `mavproxy`), ajuste, **grave e reinicie** —
os parâmetros de `SERIAL*` só valem depois do reboot.

### A porta

| Parâmetro | Valor | Por quê |
|---|---|---|
| `SERIAL1_PROTOCOL` | `2` | MAVLink v2. O firmware do ESP32 fala v2. |
| `SERIAL1_BAUD` | `115` | 115200. Sobra banda para as taxas abaixo. |
| `BRD_SER1_RTSCTS` | `0` | Sem controle de fluxo — CTS/RTS não estão ligados. |

`SERIAL1_*` é a TELEM1. Se você ligou o ESP32 na TELEM2, tudo vira `SERIAL2_*` e
`SR2_*`. Trocar as duas coisas de lugar sem trocar o prefixo dá um link mudo
sem nenhuma mensagem de erro.

Subir para `SERIAL1_BAUD 921` (921600) funciona e o ESP32 acompanha, mas só faça
isso depois que o link estiver estável a 115200 — a 921600, um fio ruim vira
`mav_parse_err` subindo no `/ozzy/diagnostics` em vez de um erro claro.

### As taxas de stream

O ArduPilot só manda o que você pedir. Os defaults são pensados para uma estação
de solo com tela, não para uma ponte; deixá-los como estão gasta banda com
coisas que ninguém assina.

| Parâmetro | Valor | O que libera |
|---|---|---|
| `SR1_EXTRA1` | `20` | `ATTITUDE` — atitude e taxas de corpo |
| `SR1_POSITION` | `10` | `LOCAL_POSITION_NED`, `GLOBAL_POSITION_INT` |
| `SR1_EXT_STAT` | `2` | `SYS_STATUS`, `GPS_RAW_INT` |
| `SR1_EXTRA2` | `0` | `VFR_HUD` — redundante com o que já vem acima |
| `SR1_EXTRA3` | `0` | — |
| `SR1_RAW_SENS` | `0` | IMU crua — não usada, e é o stream mais caro |
| `SR1_RC_CHAN` | `0` | canais do rádio |
| `SR1_RAW_CTRL` | `0` | — |
| `SR1_PARAMS` | `0` | download de parâmetros; use o USB para isso |

Isso põe as taxas de `/ozzy/pose` e `/ozzy/twist` em 20 Hz — a atitude é o
stream rápido e a posição chega a 10 Hz, então a pose publicada a 20 Hz repete
posição entre atualizações. É deliberado: a máquina de estados quer uma cadência
previsível, e atitude fresca vale mais que uma pose que chega em soluços.

### O failsafe de GCS

| Parâmetro | Valor | Por quê |
|---|---|---|
| `FS_GCS_ENABLE` | `5` (Land) em arena fechada, `1` (RTL) em campo aberto | dispara quando o `HEARTBEAT` do ESP32 some |
| `FS_GCS_TIMEOUT` | `5` | segundos de silêncio antes de disparar |
| `SYSID_MYGCS` | `255` | o firmware se identifica como sysid 255 |

`SYSID_MYGCS` é o parâmetro que morde em silêncio: o ArduPilot ignora comandos
vindos de um sysid diferente do declarado ali. O default é 255 e o firmware usa
255, então normalmente não há nada a fazer — mas se alguém já mexeu nesse
parâmetro para acomodar outra estação de solo, os comandos do `ozzy_bridge`
passam a ser descartados **sem resposta e sem erro**. Em versões recentes do
ArduPilot ele aparece como `MAV_GCS_SYSID`.

### GUIDED precisa saber onde está

`GUIDED` é modo de posição: sem estimativa válida, o ArduPilot recusa entrar
nele e diz o motivo no `/ozzy/statustext`.

- **Em campo aberto**, o GPS resolve. Espere fix 3D e HDOP < 1,5 antes de armar.
- **Em arena fechada** não há GPS. As opções são fluxo óptico ou sistema externo
  de posição, configurados em `EK3_SRC1_*` — e isso é um projeto por si só. Sem
  nenhum dos dois, o que sobra é `GUIDED_NOGPS`, que aceita só atitude e
  empuxo: nada de `/ozzy/setpoint`, e `/ozzy/cmd_vel` deixa de fazer o que o
  nome diz.

Vale decidir isso **antes** de escrever os estados da missão, porque muda qual
metade do contrato está disponível.

### Não desligue as checagens

`ARMING_CHECK` fica em `1` (todas). Quando o veículo não arma, a resposta está
no `/ozzy/statustext`, e o caminho é corrigir o que ele apontou. Zerar
`ARMING_CHECK` para "destravar o teste" é como o time perde uma hélice.

## Modos, pelo número

O `HEARTBEAT.custom_mode` do ArduCopter é um inteiro. O firmware traduz nos dois
sentidos; a tabela está aqui porque em algum momento você vai ler o número cru
num log:

| Nº | Modo | | Nº | Modo |
|---|---|---|---|---|
| 0 | `STABILIZE` | | 9 | `LAND` |
| 1 | `ACRO` | | 11 | `DRIFT` |
| 2 | `ALT_HOLD` | | 13 | `SPORT` |
| 3 | `AUTO` | | 16 | `POSHOLD` |
| 4 | `GUIDED` | | 17 | `BRAKE` |
| 5 | `LOITER` | | 18 | `THROW` |
| 6 | `RTL` | | 20 | `GUIDED_NOGPS` |
| 7 | `CIRCLE` | | 21 | `SMART_RTL` |

Esses números são do **Copter**. Plane e Rover usam outra tabela — se um dia
este repositório voar num Rover, `mav_link.cpp` precisa saber disso.

## Sequência mínima de um voo

```bash
ros2 topic pub --once /ozzy/set_mode std_msgs/String "{data: 'GUIDED'}"
ros2 topic echo --once /ozzy/mode          # confirme que virou GUIDED
ros2 service call /ozzy/arm std_srvs/srv/SetBool "{data: true}"
ros2 topic echo --once /ozzy/armed         # confirme que armou
ros2 topic pub --once /ozzy/takeoff std_msgs/Float32 "{data: 2.0}"
```

Repare que cada passo é seguido de uma confirmação lida de volta. O
`COMMAND_ACK` do MAVLink diz que o comando foi aceito, não que o veículo fez —
e a diferença entre as duas coisas é onde moram os acidentes.
