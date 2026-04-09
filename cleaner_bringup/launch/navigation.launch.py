"""
navigation.launch.py — go_to_room package
==========================================
Launches Nav2 (navigate_to_pose/Recover) + RViz2 + go_to_room_node.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Fix for some rendering environments
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
    os.environ['ogre_gl_version'] = '2'
    
    pkg = FindPackageShare('go_to_room')
    nav2_pkg = FindPackageShare('nav2_bringup')

    # File Paths
    default_map = PathJoinSubstitution([pkg, 'config', 'map.yaml'])
    default_rviz = PathJoinSubstitution([pkg, 'rviz', 'nav2_go_to_room.rviz'])
    safe_bt_xml = PathJoinSubstitution([pkg, 'config', 'navigate_to_pose.xml'])
    params_file = PathJoinSubstitution([pkg, 'config', 'nav2_params.yaml'])
    rooms_config = PathJoinSubstitution([pkg, 'config', 'rooms.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        
        # 1. Nav2 Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_pkg, '/launch/bringup_launch.py']),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': params_file,
                'default_nav_to_pose_bt_xml': safe_bt_xml,
                'default_nav_through_poses_bt_xml': safe_bt_xml,
            }.items(),
        ),

        # 2. RViz2
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', default_rviz],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
        
        # 3. Your Go To Room Node
        Node(
            package='go_to_room', 
            executable='go_to_room_node',
            name='go_to_room_node', 
            output='screen',
            parameters=[
                rooms_config,  # Loads ROOM_BOUNDS and room goals
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'initial_x': -2.0,   # Robot's starting X
                    'initial_y': 0.5,   # Robot's starting Y
                    'initial_yaw': 0.0, # Robot's starting Orientation
                    'max_retries': 3
                }
            ]
        )
    ])

