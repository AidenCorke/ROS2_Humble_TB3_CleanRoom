from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    server_pkg = get_package_share_directory('clean_room_server')
    client_pkg = get_package_share_directory('clean_room_client')
    map_yaml = os.path.join(server_pkg, 'maps', 'house_map.yaml')
    #rooms_yaml = os.path.join(ops_pkg, 'config', 'rooms.yaml')
    '''
    # Gazebo House World
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_pkg, 'launch', 'turtlebot3_house.launch.py')
        )
    )

    # Nav2 Bringup
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml, 
            'use_sim_time': 'true',
            'autostart': 'true'
        }.items()
    )
    '''
    # Clean Room Server
    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(server_pkg, 'launch', 'clean_room_server.launch.py')
        )
    )

    # Clean Room Client
    client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(client_pkg, 'launch', 'clean_room_client.launch.py')
        )
    )

    # List of all launched items
    return LaunchDescription([
        #gazebo,
        #nav2,
        server,
        client
    ])
