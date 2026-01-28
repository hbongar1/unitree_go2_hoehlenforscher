import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/workspace/unitree_go2_hoehlenforscher/src/go2_my_nodes_py/install/go2_my_nodes_py'
