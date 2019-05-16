from RovPID import *
from XyCtrl import *

ahrs = "AHRS"

SampleTime = 0.01

xyCtrl = XyCtrl(ahrs=ahrs, sampleTime=SampleTime)

xyCtrl.Heading = 90     # kierunek "patrzenia" rova
xyCtrl.Direction = 0    # płyń do przodu
xyCtrl.Power = 30       # 30% macoy silnikow


while True:
    xyCtrl.update()