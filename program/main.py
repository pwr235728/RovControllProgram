from RovPID import *
from XyCtrl import *
from ZCtrl import *
from AHRS_sim import *
from Bar02 import *
from rovSim import *
from live_plot import *
import time
import threading

SampleTime = 0.005
ahrs_logger = ahrs_logger()

def sim():
    ahrs = AHRS()
    bar02 = Bar02()

    rov_sim = RovSim(sample_time=SampleTime)

    xyCtrl = XyCtrl(ahrs=ahrs, sampleTime=SampleTime)
    xyCtrl.heading = 30  # kierunek "patrzenia" rova
    xyCtrl.direction = 0  # płyń do przodu
    xyCtrl.power = 10  # 30% macoy silnikow

    zCtrl = ZCtrl(ahrs=ahrs, bar02=bar02, sampleTime=SampleTime)
    zCtrl.pitch = -10
    zCtrl.depth = 0

    ct = 0
    while True:
        ct = ct + SampleTime

        if ct > 10:
            ct = 0
            zCtrl.depth = -zCtrl.depth
            xyCtrl.heading = -xyCtrl.heading

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

        ahrs_logger.log(ahrs, bar02)

        time.sleep(SampleTime)
    """
        rot = -xyCtrl.thrusterA + xyCtrl.thrusterB - xyCtrl.thrusterC + xyCtrl.thrusterD
        pow = math.fabs(xyCtrl.thrusterA) + math.fabs(xyCtrl.thrusterB) + math.fabs(xyCtrl.thrusterC) + math.fabs(xyCtrl.thrusterD)
        print("", "{0:07.2f}".format(rot),"; ",  "{0:07.2f}".format(pow), " ;", "{0:07.2f}".format(ahrs.HEADING), "; ", "{0:07.2f}".format(ahrs.YAW_SPEED))
    
        ahrs.YAW_SPEED = ahrs.YAW_SPEED + rot*SampleTime
        ahrs.HEADING = ahrs.HEADING + ahrs.YAW_SPEED*0.01
    """


def main():
    main_thread = threading.Thread(target=sim)
    main_thread.start()

    plotter = Plotter()
    while True:
        plotter.plot(ahrs_logger)

main()