import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pfe/SlamGit/lk-s-2026-miloje-slam/install/my_robot_slam'
