from RovPID import *
from XyCtrl import *
from ZCtrl import *
from AHRS_sim import *
from Bar02 import *
from rovSim import *
import time

SampleTime = 0.01

ahrs = AHRS()
bar02 = Bar02()

rov_sim = RovSim(sample_time=SampleTime)


xyCtrl = XyCtrl(ahrs=ahrs, sampleTime=SampleTime)
xyCtrl.heading = 90   # kierunek "patrzenia" rova
xyCtrl.direction = 0    # płyń do przodu
xyCtrl.power = 100   # 30% macoy silnikow

zCtrl = ZCtrl(ahrs=ahrs, bar02=bar02, sampleTime=SampleTime)
zCtrl.pitch = 0
zCtrl.depth = 10

while True:
    ahrs.HEADING = rov_sim.heading
    ahrs.PITCH = rov_sim.pitch
    ahrs.PITCH_SPEED = rov_sim.pitch_speed
    ahrs.YAW_SPEED = rov_sim.heading_speed

    bar02.DEPTH = rov_sim.depth
    bar02.DEPTH_SPEED = rov_sim.depth_speed

    xyCtrl.update()
    zCtrl.update()

    rov_sim.thrA = xyCtrl.thrusterA
    rov_sim.thrB = xyCtrl.thrusterB
    rov_sim.thrC = xyCtrl.thrusterC
    rov_sim.thrD = xyCtrl.thrusterD

    rov_sim.thrZL = zCtrl.thruster_ZL
    rov_sim.thrZR = zCtrl.thruster_ZR
    rov_sim.thrZB = zCtrl.thruster_ZB

    rov_sim.update()

    print("", "{0:07.2f}".format(ahrs.HEADING), "; ", "{0:07.2f}".format(ahrs.YAW_SPEED),
          " ;", "{0:07.2f}".format(ahrs.PITCH), "; ", "{0:07.2f}".format(ahrs.PITCH_SPEED),
          " ;", "{0:07.2f}".format(bar02.DEPTH), "; ", "{0:07.2f}".format(bar02.DEPTH_SPEED))

    time.sleep(SampleTime)
"""
    rot = -xyCtrl.thrusterA + xyCtrl.thrusterB - xyCtrl.thrusterC + xyCtrl.thrusterD
    pow = math.fabs(xyCtrl.thrusterA) + math.fabs(xyCtrl.thrusterB) + math.fabs(xyCtrl.thrusterC) + math.fabs(xyCtrl.thrusterD)
    print("", "{0:07.2f}".format(rot),"; ",  "{0:07.2f}".format(pow), " ;", "{0:07.2f}".format(ahrs.HEADING), "; ", "{0:07.2f}".format(ahrs.YAW_SPEED))

    ahrs.YAW_SPEED = ahrs.YAW_SPEED + rot*SampleTime
    ahrs.HEADING = ahrs.HEADING + ahrs.YAW_SPEED*0.01
"""
