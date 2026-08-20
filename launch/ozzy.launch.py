"""Sobe o lado do computador: agente micro-ROS + monitor de link.

    ros2 launch ozzy_bridge ozzy.launch.py
    ros2 launch ozzy_bridge ozzy.launch.py monitor:=false

O agente é o MESMO binário que o workspace usa para o PX4 — MicroXRCEAgent,
pinado em v3.0.1 no env/<perfil>.yaml do evtol-dev — na MESMA porta 8888 do
templates/scripts/agent.sh. Não há ferramenta nova para instalar aqui.

Depois deste launch, ligue o ESP32. Se o `link_monitor` continuar dizendo que
nada chegou, o problema está entre o ESP32 e esta máquina, não no ROS 2 —
e a mensagem dele diz o que conferir.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    monitor = LaunchConfiguration('monitor')
    verbose = LaunchConfiguration('verbose')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='8888',
            description='porta UDP do agente; casa com OZZY_AGENT_PORT do secrets.h'),
        DeclareLaunchArgument(
            'monitor', default_value='true',
            description='sobe também o link_monitor'),
        DeclareLaunchArgument(
            'verbose', default_value='4',
            description='verbosidade do agente (6 mostra cada mensagem XRCE; '
                        'útil para provar que o ESP32 chegou)'),

        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', port, '-v', verbose],
            output='screen',
            # O agente morrendo derruba o launch inteiro de propósito: sem ele,
            # os outros nós ficariam de pé assinando tópicos que nunca vão
            # existir, o que parece "esperando o drone" e não é.
            on_exit=Shutdown(),
        ),

        Node(
            package='ozzy_bridge',
            executable='link_monitor',
            name='ozzy_link_monitor',
            output='screen',
            condition=IfCondition(monitor),
        ),
    ])
