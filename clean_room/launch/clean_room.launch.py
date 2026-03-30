from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('clean_room')
    rooms_yaml = PathJoinSubstitution([pkg_share, 'config', 'rooms.yaml'])
    map_yaml = PathJoinSubstitution([pkg_share, 'maps', 'house_map.yaml'])

    cleaning_node = Node(
        package='clean_room',
        executable='clean_room',
        name='clean_room',
        output='screen',
        parameters=[{'rooms_config': rooms_yaml},
                    {'map_yaml': map_yaml},
                    ]
    )

    return LaunchDescription([
        cleaning_node
    ])
