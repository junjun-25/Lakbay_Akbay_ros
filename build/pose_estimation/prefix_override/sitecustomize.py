import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kenji/Lakbay_Akbay_ros/install/pose_estimation'
