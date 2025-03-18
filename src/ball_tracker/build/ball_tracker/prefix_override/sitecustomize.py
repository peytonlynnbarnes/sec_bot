import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/peyton/dev_ws/src/ball_tracker/install/ball_tracker'
