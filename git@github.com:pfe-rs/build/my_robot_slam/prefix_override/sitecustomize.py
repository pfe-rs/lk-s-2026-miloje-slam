import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/veljkogalovic/robot_ws/git@github.com:pfe-rs/install/my_robot_slam'
