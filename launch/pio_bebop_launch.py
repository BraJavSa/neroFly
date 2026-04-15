import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher

def generate_launch_description():
    package_dir = get_package_share_directory('neroFly')
    
    # Referencia al nuevo mundo con el Pioneer 3-AT
    world_path = os.path.join(package_dir, 'worlds', 'bebop_pionner.wbt')

    webots = WebotsLauncher(world=world_path, mode='realtime')

    # Nodo del Dron Bebop
    simulator_node = Node(
        package='neroFly',
        executable='simulator_node',
        name='neroFlyulator',
        additional_env={'WEBOTS_CONTROLLER_URL': 'bebop'}
    )

    pionner_node = Node(
        package='neroFly',
        executable='pionner_node',
        name='pioneer_controller',
        additional_env={'WEBOTS_CONTROLLER_URL': 'pioneer_3at'}
    )

    # Nodo de la Flecha de Referencia
    visualizer_node = Node(
        package='neroFly',
        executable='visualizer_node',
        name='target_visualizer',
        additional_env={'WEBOTS_CONTROLLER_URL': 'target_visualizer'}
    )

    return LaunchDescription([
        webots,
        simulator_node,
        visualizer_node,
        pionner_node
    ])