from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    server_pkg = get_package_share_directory('clean_room_server')
    map_yaml = os.path.join(server_pkg, 'maps', 'house_map.yaml')
    rooms_yaml = os.path.join(server_pkg, 'config', 'rooms.yaml')

    return LaunchDescription([
        Node(
            package='clean_room_server',
            executable='clean_room_server',
            name='clean_room_server',
            output='screen',
            parameters=[
                {'rooms_config': rooms_yaml},
                {'map_yaml': map_yaml},
                {'use_sim_time': True}
            ]
        )
    ])
