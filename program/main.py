from RovPID import *
from XyCtrl import *
from ZCtrl import *
from AHRS_sim import *

import time

ahrs = AHRS()
bar02 = "bar02"

SampleTime = 0.01

xyCtrl = XyCtrl(ahrs=ahrs, sampleTime=SampleTime)
xyCtrl.heading = 90   # kierunek "patrzenia" rova
xyCtrl.direction = 0    # płyń do przodu
xyCtrl.power = 100   # 30% macoy silnikow

zCtrl = ZCtrl(ahrs=ahrs, bar02=bar02, sampleTime=SampleTime)
zCtrl.pitch = 0
zCtrl.depth = 0

while True:
    xyCtrl.update()
    zCtrl.update()


    rot = -xyCtrl.thrusterA + xyCtrl.thrusterB - xyCtrl.thrusterC + xyCtrl.thrusterD
    pow = math.fabs(xyCtrl.thrusterA) + math.fabs(xyCtrl.thrusterB) + math.fabs(xyCtrl.thrusterC) + math.fabs(xyCtrl.thrusterD)
    print("", "{0:07.2f}".format(rot),"; ",  "{0:07.2f}".format(pow), " ;", "{0:07.2f}".format(ahrs.HEADING), "; ", "{0:07.2f}".format(ahrs.YAW_SPEED))

    ahrs.YAW_SPEED = ahrs.YAW_SPEED + rot*SampleTime
    ahrs.HEADING = ahrs.HEADING + ahrs.YAW_SPEED*0.01

    time.sleep(SampleTime)
