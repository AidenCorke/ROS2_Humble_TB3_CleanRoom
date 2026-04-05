from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    tb3_nav_pkg = get_package_share_directory('turtlebot3_navigation2')
    #nav2_pkg = get_package_share_directory('nav2_bringup')
    server_pkg = get_package_share_directory('clean_room_server')

    map_yaml = os.path.join(server_pkg, 'maps', 'house_map.yaml')
    print("MAP YAML:", map_yaml)
    #params_file = os.path.join(server_pkg, 'config', 'nav2_params.yaml')

    # ----- Gazebo House World -----
    tb3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_pkg,'launch', 'turtlebot3_house.launch.py')
        )
    )
    
    
    # ----- Nav2 Bringup -----
    tb3_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_nav_pkg, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'map_yaml_file': map_yaml,
            'use_sim_time': 'true',
        }.items()
    )
    
    '''
    # ----- Nav2 Bringup -----
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml, 
            'params_file': params_file,
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_rviz': 'true'
        }.items()
    )
    '''
    

    initial_pose = Node(
        package='clean_room_server',
        executable='initial_pose_publisher',
        output='screen'
    )

    # ----- Clean Room Server -----
    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(server_pkg, 'launch', 'clean_room_server.launch.py')
        )
    )   

    # List of all launched items
    return LaunchDescription([
        tb3_gazebo,
        TimerAction(period=(5.0),actions=[tb3_nav]),
        TimerAction(period=(10.0),actions=[initial_pose]),
        TimerAction(period=(15.0),actions=[server]),
    ])
