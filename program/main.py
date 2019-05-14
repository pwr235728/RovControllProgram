from RovPID import *
from XyCtrl import *

ahrs = "AHRS"

SampleTime = 0.01

xyCtrl = XyCtrl(SampleTime)


while True:
    xyCtrl.update(AHRS=ahrs)