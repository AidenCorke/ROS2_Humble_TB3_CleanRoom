from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

from launch.actions import RegisterEventHandler
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

import os

def generate_launch_description():
    # -------------------------------------------------------------------
    # --- Gazebo Launch ---
    # ------------------------------------------------------------------- 
    tb3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_house.launch.py'
            )
        )
    )


    # -------------------------------------------------------------------
    # --- Nav2 launch with map file ---
    # ------------------------------------------------------------------- 
    map_yaml = PathJoinSubstitution([
        FindPackageShare('clean_room'),
        'maps',
        'house_map.yaml'
    ])
    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': 'true'
        }.items()
    )


    # -------------------------------------------------------------------
    # --- RViz Launch ---
    # ------------------------------------------------------------------- 
    rviz_config = PathJoinSubstitution([
        FindPackageShare('clean_room'),
        'rviz',
        'cleaning_view.rviz'
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        on_exit = Shutdown()  # Shuts down the entire launch if this node exits
    )


    # -------------------------------------------------------------------
    # --- Clean Room Launch---
    # ------------------------------------------------------------------- 
    clean_room = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('clean_room'),
                'launch',
                'clean_room.launch.py'
            )
        )
    )

    '''    
    # Delay function - launch clean room when Nav2 is ready
    nav2_ready = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node='controller_server',
            goal_state='active',
            entities=[clean_room]
        )
    ) '''


    return LaunchDescription([
        tb3_gazebo,
        nav2,
        rviz,
        clean_room,
    ])
