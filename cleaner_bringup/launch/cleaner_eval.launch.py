from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # ----- Define package and parameter paths -----
    # --- Package Paths ---
    bringup_pkg = get_package_share_directory('cleaner_bringup')
    go_to_room_pkg = get_package_share_directory('go_to_room')
    clean_room_pkg = get_package_share_directory('clean_room')

    # --- Parameter Paths ---
    map_yaml = os.path.join(bringup_pkg, 'maps', 'house_map.yaml')
    cr_rooms_yaml = os.path.join(clean_room_pkg, 'config', 'rooms.yaml')
    gtr_rooms_yaml = os.path.join(go_to_room_pkg, 'config', 'rooms.yaml')


    # ----- Launch Descriptions -----
    # --- Clean Room Server ---
    clean_room  = Node(package='clean_room',
                       executable='clean_room_server',
                       name='clean_room_server',
                       output='screen',
                       parameters=[{'rooms_config': cr_rooms_yaml},
                                   {'map_yaml': map_yaml},
                                   {'use_sim_time': True}
                                   ]
                                   )

    # --- Clean Room Evaluator ---
    evaluator = Node(package='evaluator_core',
                     executable='evaluator_node',
                     name='evaluator_node',
                     output='screen',
                     )
    
    # --- Clean Room Evaluator ---
    go_to_room = Node(package='go_to_room',
                      executable='go_to_room_node',
                      name='go_to_room_node',
                      output='screen',parameters=[gtr_rooms_yaml,  # Loads ROOM_BOUNDS and room goals
                                                  {'use_sim_time': True,
                                                   'initial_x': -2.0,   # Robot's starting X        
                                                   'initial_y': 0.5,   # Robot's starting Y        
                                                   'initial_yaw': 0.0, # Robot's starting Orientation        
                                                   'max_retries': 3
                                                   }
                                                   ]
                                                   )


    # List of all launched items
    return LaunchDescription([
        clean_room,
        go_to_room,
        evaluator,
    ])
