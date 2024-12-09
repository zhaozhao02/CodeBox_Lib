import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/khadas/roaf3d_ws_loop/src/pwm_control/install/pwm_control'
