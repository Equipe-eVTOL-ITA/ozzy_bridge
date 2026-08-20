import glob

from setuptools import find_packages, setup

package_name = 'ozzy_bridge'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

launch_files = glob.glob('launch/*.py')
if launch_files:
    data_files.append((f'share/{package_name}/launch', launch_files))

setup(
    name=package_name,
    version='0.1.0',
    # firmware/ não é pacote Python e não pode entrar no wheel: ele é compilado
    # pelo PlatformIO, não pelo colcon.
    packages=find_packages(exclude=['test', 'firmware', 'firmware.*']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marconipavan',
    maintainer_email='angelo.marconi.pavan@gmail.com',
    description='Ponte ROS 2 <-> ArduPilot via ESP32 (micro-ROS + MAVLink)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'link_monitor = ozzy_bridge.link_monitor:main',
            'teleop = ozzy_bridge.teleop:main',
        ],
    },
)
