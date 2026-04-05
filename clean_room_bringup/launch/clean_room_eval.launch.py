from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    server_pkg = get_package_share_directory('clean_room_server')

    # ----- Clean Room Server -----
    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(server_pkg, 'launch', 'clean_room_server.launch.py')
        )
    )   

    # ----- Clean Room Evaluator -----
    evaluator = Node(package='evaluator_core',
                     executable='evaluator_node',
                     name='evaluator_node',
                     output='screen',
                     )

    # List of all launched items
    return LaunchDescription([
        server,
        evaluator,
    ])
