import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher

def generate_launch_description():
    package_dir = get_package_share_directory('neroFly')
    world_path = os.path.join(package_dir, 'worlds', 'bebop.wbt')

    webots = WebotsLauncher(world=world_path, mode='realtime')

    return LaunchDescription([
        webots,
        Node(
            package='neroFly',
            executable='simulator_node',
            name='neroFlyulator',
            additional_env={'WEBOTS_CONTROLLER_URL': 'bebop'}
        ),
        Node(
            package='neroFly',
            executable='visualizer_node',
            name='target_visualizer',
            additional_env={'WEBOTS_CONTROLLER_URL': 'target_visualizer'}
        ),
    ])