import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/work/Caramelo_workspace/src/caramelo_manipulation/install/caramelo_manipulation'
