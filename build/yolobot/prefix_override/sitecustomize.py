import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ros2-kev/Dev/img_det_ros2_ws/install/yolobot'
