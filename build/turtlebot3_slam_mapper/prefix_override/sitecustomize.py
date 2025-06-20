import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/agustin/Documentos/4to/robotica/TPF_robotica/install/turtlebot3_slam_mapper'
