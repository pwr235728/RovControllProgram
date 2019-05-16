from RovPID import *
import math

class ZCtrl:
    __speedPID_limit = 10  # maksymalna moc - ograniczenie wyjścia PID od prędkości
    __positionPID_limit = 90  # maksymalna prędkość - ograniczenie wyjścia PID od pozycji

    __pid_params_out = PidParams(Kp=1.0, Ki=0.0, Kd=0.0, Limit=__positionPID_limit)
    __pid_params_in = PidParams(Kp=1.0, Ki=0.0, Kd=0.0, Limit=__speedPID_limit)