import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nemanja/pfe/letnji_2026/lk-s-2026-miloje-slam/src/my_robot_slam/install/my_robot_slam'
