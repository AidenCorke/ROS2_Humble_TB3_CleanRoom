from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # ----- Create paths to relevant folders and files -----
    # -- Package paths --
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    #tb3_nav_pkg = get_package_share_directory('turtlebot3_navigation2')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    bringup_pkg = get_package_share_directory('cleaner_bringup')

    # -- Config and parameter paths --
    map_yaml = os.path.join(bringup_pkg, 'maps', 'house_map.yaml')
    nav2_params = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')
    rviz_params = os.path.join(bringup_pkg, 'config', 'rviz_params.rviz')
    NavToPose_params = os.path.join(bringup_pkg, 'config', 'navigate_to_pose.xml')

    # ----- Launch Descriptions for packages -----
    # --- Gazebo House World ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_pkg,'launch', 'turtlebot3_house.launch.py')
        ),
        launch_arguments={'x_pose': '-2.0',
                          'y_pose': '0.5',
                          'yaw': '1.57',
                          }.items()
    )
    
    # --- Nav2 Bringup ---
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'map_yaml_file': map_yaml,
            'use_sim_time': 'True',
            'params_file': nav2_params,
            'default_nav_to_pose_bt_xml': NavToPose_params,
            'default_nav_through_poses_bt_xml': NavToPose_params,
        }.items()
    )

    # --- RViz Bringup ---
    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_params],
                )

    # List of all launched items
    return LaunchDescription([
        gazebo,
        TimerAction(period=(5.0),actions=[nav2]),
        TimerAction(period=(15.0),actions=[rviz]),
    ])
