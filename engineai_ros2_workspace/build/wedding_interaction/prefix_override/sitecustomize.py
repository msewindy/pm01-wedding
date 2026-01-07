import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lingjing/project/engine_ai/engineai_ros2_workspace/install/wedding_interaction'
