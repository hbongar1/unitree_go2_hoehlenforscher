import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/workspace/unitree_go2_hoehlenforscher/install/unitree_sdk2py'
