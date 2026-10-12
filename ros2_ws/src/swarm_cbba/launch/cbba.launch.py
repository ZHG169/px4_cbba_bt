import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('swarm_cbba')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='v60'),   # same namespace as task_manager.launch.py: /<ns>/current_task ...
        DeclareLaunchArgument('params', default_value=os.path.join(pkg, 'config', 'cbba_v60.yaml')),
        Node(
            package='swarm_cbba', executable='cbba_node', name='cbba_node',
            namespace=LaunchConfiguration('namespace'), output='screen', emulate_tty=True,
            parameters=[LaunchConfiguration('params')],   # the ns parameter stays empty so the launch namespace applies
        ),
    ])
