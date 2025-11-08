import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/henry/unity_udp_ros_bridge/src/unity_udp_ros_bridge/install/unity_udp_ros_bridge'
