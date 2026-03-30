from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clean_room_client',
            executable='clean_room_client',
            name='clean_room_client',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
