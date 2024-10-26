import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/CSE276A_Fall24/cse276a_ws/install/opencv_tests'
