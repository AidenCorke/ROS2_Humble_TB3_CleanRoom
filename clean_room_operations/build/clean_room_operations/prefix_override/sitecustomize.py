import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aiden/ros2_ws_lab/src/clean_room_operations/install/clean_room_operations'
