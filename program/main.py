from RovPID import *
from XyCtrl import *
from ZCtrl import *
from AHRS_sim import *

import time

ahrs = AHRS()
bar02 = "bar02"

SampleTime = 0.01

xyCtrl = XyCtrl(ahrs=ahrs, sampleTime=SampleTime)
xyCtrl.Heading = 0   # kierunek "patrzenia" rova
xyCtrl.Direction = 0    # płyń do przodu
xyCtrl.Power = 100   # 30% macoy silnikow

ZCtrl = ZCtrl(ahrs=ahrs, bar02=bar02, sampleTime=SampleTime)


while True:
    xyCtrl.update()
    rot = -xyCtrl.thrusterA + xyCtrl.thrusterB - xyCtrl.thrusterC + xyCtrl.thrusterD
    pow = math.fabs(xyCtrl.thrusterA) + math.fabs(xyCtrl.thrusterB) + math.fabs(xyCtrl.thrusterC) + math.fabs(xyCtrl.thrusterD)
    print("", "{0:07.2f}".format(rot),"; ",  "{0:07.2f}".format(pow), " ;", "{0:07.2f}".format(ahrs.HEADING), "; ", "{0:07.2f}".format(ahrs.YAW_SPEED))

    ahrs.YAW_SPEED = ahrs.YAW_SPEED + rot*SampleTime
    ahrs.HEADING = ahrs.HEADING + ahrs.YAW_SPEED*0.01

    time.sleep(SampleTime)
