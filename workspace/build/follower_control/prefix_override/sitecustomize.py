import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/quanmh25/quanmh/diffdrive_aruco_follow/workspace/install/follower_control'
