"""Monitor do link do ozzy_bridge — o `doctor.sh` desta ponte.

    ros2 run ozzy_bridge link_monitor

Responde, uma vez por segundo, à única pergunta que importa antes de armar:
*o que está funcionando e o que não está*. Sem ele, o modo de falha normal é um
`ros2 topic list` que mostra os tópicos certos e um `echo` que nunca imprime —
e a partir daí a suspeita cai no lugar errado.

As três causas que ele separa, e que sozinhas são indistinguíveis:

  1. **O ESP32 não chegou no agente.** Nenhum tópico existe.
  2. **O ESP32 chegou, a Pixhawk não responde.** Tópicos existem, `mav_rx_hz`
     é zero. Fio trocado, baud errado, `SERIAL1_PROTOCOL` errado.
  3. **Tudo conectado, dados velhos.** Tópicos existem e param de atualizar.
"""

import time

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

#: QoS dos tópicos de telemetria rápida. TEM que casar com o firmware.
#
# O firmware publica /ozzy/pose e /ozzy/twist em BEST_EFFORT (veja o comentário
# de QoS em firmware/src/ros_link.cpp). Um assinante com o QoS default do rclpy
# — que é RELIABLE — simplesmente NÃO CASA: o tópico aparece no `ros2 topic
# list`, o `ros2 topic info` mostra um publicador, e nenhuma mensagem chega.
# Nenhum erro é impresso em lugar nenhum. É a armadilha mais cara deste
# repositório do lado do computador.
FAST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

STALE_S = 1.0     # sem pose por mais que isto, o dado é velho
SILENT_S = 3.0    # sem nada por mais que isto, o link caiu


class LinkMonitor(Node):

    def __init__(self):
        super().__init__('ozzy_link_monitor')

        self._pose_t = None
        self._diag = None
        self._diag_t = None
        self._armed = None
        self._mode = None
        self._pose = None

        self.create_subscription(PoseStamped, '/ozzy/pose', self._on_pose, FAST_QOS)
        self.create_subscription(DiagnosticStatus, '/ozzy/diagnostics', self._on_diag, 10)
        self.create_subscription(Bool, '/ozzy/armed', self._on_armed, 10)
        self.create_subscription(String, '/ozzy/mode', self._on_mode, 10)
        self.create_subscription(String, '/ozzy/statustext', self._on_statustext, 10)

        self.create_timer(1.0, self._report)
        self.get_logger().info('monitorando /ozzy/... — Ctrl-C para sair')

    # --- assinaturas ---------------------------------------------------------

    def _on_pose(self, msg):
        self._pose = msg
        self._pose_t = time.monotonic()

    def _on_diag(self, msg):
        self._diag = {kv.key: kv.value for kv in msg.values}
        self._diag['_level'] = msg.level
        self._diag['_message'] = msg.message
        self._diag_t = time.monotonic()

    def _on_armed(self, msg):
        self._armed = msg.data

    def _on_mode(self, msg):
        self._mode = msg.data

    def _on_statustext(self, msg):
        # Repassado como warning porque é sempre isto que explica um arm
        # recusado. Aparece no terminal com timestamp, junto do resto.
        self.get_logger().warning(f'[ArduPilot] {msg.data}')

    # --- relatório -----------------------------------------------------------

    def _report(self):
        now = time.monotonic()

        if self._diag_t is None:
            self.get_logger().error(
                'nenhum /ozzy/diagnostics. O ESP32 não chegou no agente. '
                'Confira: o MicroXRCEAgent está de pé? O OZZY_AGENT_IP do '
                'secrets.h é o IP desta máquina? O ESP32 entrou na rede?')
            return

        if now - self._diag_t > SILENT_S:
            self.get_logger().error(
                f'/ozzy/diagnostics parado há {now - self._diag_t:.1f} s — '
                'o ESP32 caiu ou saiu da rede.')
            return

        rx_hz = float(self._diag.get('mav_rx_hz', 0.0))
        errs = self._diag.get('mav_parse_err', '?')
        rssi = self._diag.get('wifi_rssi', '?')
        heap = int(self._diag.get('free_heap', 0))
        level = self._diag.get('_level', 0)

        if rx_hz == 0.0:
            self.get_logger().error(
                'ESP32 conectado, mas a Pixhawk está muda (mav_rx_hz = 0). '
                'Confira: TX/RX trocados, SERIAL1_BAUD (115) e '
                'SERIAL1_PROTOCOL (2). Veja docs/ARDUPILOT.md.')
            return

        pose_age = '—' if self._pose_t is None else f'{now - self._pose_t:.2f}s'
        pos = ''
        if self._pose is not None:
            p = self._pose.pose.position
            pos = f'  x={p.x:+.2f} y={p.y:+.2f} z={p.z:+.2f}'

        line = (f'link {"OK" if level == 0 else "WARN" if level == 1 else "ERRO"}'
                f'  mav {rx_hz:.1f} Hz  errs {errs}  rssi {rssi} dBm'
                f'  heap {heap // 1024} kB'
                f'  | modo {self._mode or "?"}'
                f'  armado {"SIM" if self._armed else "não"}'
                f'  | pose {pose_age}{pos}')

        if level >= 2:
            self.get_logger().error(f'{line}  <- {self._diag.get("_message", "")}')
        elif level == 1:
            self.get_logger().warning(f'{line}  <- {self._diag.get("_message", "")}')
        else:
            self.get_logger().info(line)

        if self._pose_t is None:
            self.get_logger().warning(
                'diagnostics chega mas /ozzy/pose não. Se o resto do link está '
                'bom, o suspeito é QoS: assinantes de /ozzy/pose precisam de '
                'BEST_EFFORT.')
        elif now - self._pose_t > STALE_S:
            self.get_logger().warning(
                f'/ozzy/pose parado há {now - self._pose_t:.1f} s')


def main(args=None):
    rclpy.init(args=args)
    node = LinkMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
