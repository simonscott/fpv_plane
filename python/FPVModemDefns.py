"""
Constant Definitions
"""
STATE_NORMAL = 1
STATE_NO_SIG = 2
STATE_NO_LOCK = 3


"""
Structure to store battery configuration info
"""
class FPVBatteryConfig:
    bat12_max_volt = 0
    bat12_min_volt = 0
    bat12_cap = 0
    bat6_max_volt = 0
    bat6_min_volt = 0
    bat6_cap = 0
