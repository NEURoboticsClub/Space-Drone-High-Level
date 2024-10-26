import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aaron/Space-Drone-High-Level/spacedrone/install/camera'
