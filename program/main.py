from RovPID import *
from XyCtrl import *
from AHRS_sim import *

import time

ahrs = AHRS()

SampleTime = 0.01

xyCtrl = XyCtrl(ahrs=ahrs, sampleTime=SampleTime)

xyCtrl.Heading = 90   # kierunek "patrzenia" rova
xyCtrl.Direction = 0    # płyń do przodu
xyCtrl.Power = 10   # 30% macoy silnikow


while True:
    xyCtrl.update()
    rot = -xyCtrl.thrusterA+xyCtrl.thrusterB-xyCtrl.thrusterC+xyCtrl.thrusterD
    pow = +xyCtrl.thrusterA+xyCtrl.thrusterB+xyCtrl.thrusterC+xyCtrl.thrusterD
    print("rot, pow: ", rot, pow, " ;heading, speed ", ahrs.HEADING, ahrs.YAW_SPEED)

    ahrs.YAW_SPEED = ahrs.YAW_SPEED + rot*SampleTime
    ahrs.HEADING = ahrs.HEADING + ahrs.YAW_SPEED*0.01

    time.sleep(SampleTime)
