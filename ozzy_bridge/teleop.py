"""Teleop de teclado para bancada — publica em /ozzy/cmd_vel.

    ros2 run ozzy_bridge teleop

Existe para uma coisa: provar que o caminho
`computador -> agente -> ESP32 -> MAVLink -> ArduPilot` obedece, antes de a
máquina de estados entrar na história. Quando um estado não move o drone, este
programa responde se o problema é do estado ou da ponte — e essa é uma bifurcação
que custa horas quando não dá para respondê-la em trinta segundos.

**Não é ferramenta de voo.** Sem rádio ligado e sem alguém com o dedo no modo,
não use.

As velocidades são de MUNDO, em ENU, como manda docs/CONTRATO.md: `w` vai para
o NORTE, não para onde o nariz aponta. Num veículo girado, isso surpreende — e
surpreender na bancada é o objetivo.
"""

import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

FAST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

HELP = """
  ozzy_bridge — teleop de bancada     (velocidades de MUNDO, ENU)

    w / s     norte / sul          (+y / -y)
    d / a     leste / oeste        (+x / -x)
    r / f     subir / descer       (+z / -z)
    q / e     guinar esq. / dir.   (+yaw / -yaw)

    espaço    parar (zera tudo)
    + / -     passo de velocidade
    Ctrl-C    sair (publica zero antes de sair)

  O firmware solta o controle se ficar 500 ms sem receber comando, então este
  programa publica a 10 Hz mesmo parado. Um terminal travado equivale a soltar
  o manche, não a manter a última velocidade.
"""

RATE_HZ = 10.0

LINEAR = {
    'w': (0.0, +1.0, 0.0), 's': (0.0, -1.0, 0.0),
    'd': (+1.0, 0.0, 0.0), 'a': (-1.0, 0.0, 0.0),
    'r': (0.0, 0.0, +1.0), 'f': (0.0, 0.0, -1.0),
}
YAW = {'q': +1.0, 'e': -1.0}


class Teleop(Node):

    def __init__(self):
        super().__init__('ozzy_teleop')
        self.declare_parameter('step', 0.3)        # m/s
        self.declare_parameter('yaw_step', 0.5)    # rad/s

        self._step = self.get_parameter('step').value
        self._yaw_step = self.get_parameter('yaw_step').value
        self._vel = [0.0, 0.0, 0.0]
        self._yaw = 0.0
        self._lock = threading.Lock()

        self._pub = self.create_publisher(TwistStamped, '/ozzy/cmd_vel', FAST_QOS)
        self.create_timer(1.0 / RATE_HZ, self._publish)

    def set_from_key(self, key):
        with self._lock:
            if key == ' ':
                self._vel = [0.0, 0.0, 0.0]
                self._yaw = 0.0
            elif key in LINEAR:
                x, y, z = LINEAR[key]
                self._vel = [x * self._step, y * self._step, z * self._step]
                self._yaw = 0.0
            elif key in YAW:
                self._vel = [0.0, 0.0, 0.0]
                self._yaw = YAW[key] * self._yaw_step
            elif key == '+':
                self._step = min(self._step + 0.1, 3.0)
                self.get_logger().info(f'passo = {self._step:.1f} m/s')
            elif key == '-':
                self._step = max(self._step - 0.1, 0.1)
                self.get_logger().info(f'passo = {self._step:.1f} m/s')

    def stop(self):
        with self._lock:
            self._vel = [0.0, 0.0, 0.0]
            self._yaw = 0.0
        self._publish()

    def _publish(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        with self._lock:
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = self._vel
            msg.twist.angular.z = self._yaw
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Teleop()

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    print(HELP)
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            key = sys.stdin.read(1)
            if key == '\x03':      # Ctrl-C
                break
            node.set_from_key(key)
    except (KeyboardInterrupt, OSError):
        pass
    finally:
        # Sai sempre mandando zero. Um teleop que morre deixando o último
        # comando de pé é a definição de ferramenta perigosa — mesmo com o
        # timeout de 500 ms do firmware como rede de segurança.
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
