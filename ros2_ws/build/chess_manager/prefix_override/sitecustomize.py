import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/angel/Projects/chess-project/ros2_ws/install/chess_manager'
