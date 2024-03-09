# px4_sitl_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4',
            executable='px4',
            name='px4_sitl',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'vehicle': 'x500'},  # 选择无人机型号
                {'namespace': 'px4_sitl'},
            ],
        ),
    ])
