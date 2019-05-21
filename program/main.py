from RovPID import *
from XyCtrl import *
from ZCtrl import *
from ahrs_sim import *
from Bar02 import *
from rovSim import *
from live_plot import *
import time
import threading

SampleTime = 0.005
ahrs_logger = ahrs_logger()


def translate_thrusters(A, B, C, D, ZL, ZR, ZB):
    """
        Tłumaczy wartości zwracane przez kontrolery na słownik w zakresie <-1, 1>
        :return: {"fr", "fl", "bl", "br", "vl", "vb"}
     """
    dict = {
        "fr": A/100.0,
        "fl": B/100.0,
        "bl": C/100.0,
        "br": D/100.0,
        "vl": ZL/100.0,
        "vr": ZR/100.0,
        "vb": ZB/100.0}
    return dict

def sim():
    ahrs = ahrs_sim()
    bar02 = Bar02()

    rov_sim = RovSim(sample_time=SampleTime)

    xyCtrl = XyCtrl(ahrs=ahrs, sampleTime=SampleTime)
    xyCtrl.heading = 30  # kierunek "patrzenia" rova
    xyCtrl.direction = 0  # płyń do przodu
    xyCtrl.power = 10  # 30% macoy silnikow

    zCtrl = ZCtrl(ahrs=ahrs, bar02=bar02, sampleTime=SampleTime)
    zCtrl.pitch = -10
    zCtrl.depth = 0


    engines_values = translate_thrusters(
        xyCtrl.thruster_A,
        xyCtrl.thruster_B,
        xyCtrl.thruster_C,
        xyCtrl.thruster_D,
        zCtrl.thruster_ZL,
        zCtrl.thruster_ZR,
        zCtrl.thruster_ZB)



    ct = 0
    while True:
        ct = ct + SampleTime

        if ct > 10:
            ct = 0
            zCtrl.depth = -zCtrl.depth
            xyCtrl.heading = -xyCtrl.heading

        ahrs.yaw = rov_sim.heading
        ahrs.pitch = rov_sim.pitch
        ahrs.rate_of_turn[1] = rov_sim.pitch_speed
        ahrs.rate_of_turn[2] = rov_sim.heading_speed

        bar02.DEPTH = rov_sim.depth
        bar02.DEPTH_SPEED = rov_sim.depth_speed

        xyCtrl.update()
        zCtrl.update()

        rov_sim.thrA = xyCtrl.thruster_A
        rov_sim.thrB = xyCtrl.thruster_B
        rov_sim.thrC = xyCtrl.thruster_C
        rov_sim.thrD = xyCtrl.thruster_D

        rov_sim.thrZL = zCtrl.thruster_ZL
        rov_sim.thrZR = zCtrl.thruster_ZR
        rov_sim.thrZB = zCtrl.thruster_ZB

        rov_sim.update()

        print("", "{0:07.2f}".format(ahrs.yaw), "; ", "{0:07.2f}".format(ahrs.rate_of_turn[2]),
              " ;", "{0:07.2f}".format(ahrs.pitch), "; ", "{0:07.2f}".format(ahrs.rate_of_turn[1]),
              " ;", "{0:07.2f}".format(bar02.DEPTH), "; ", "{0:07.2f}".format(bar02.DEPTH_SPEED))

        ahrs_logger.log(ahrs, bar02)

        time.sleep(SampleTime)



def main():
    main_thread = threading.Thread(target=sim)
    main_thread.start()

    plotter = Plotter()
    while True:
        plotter.plot(ahrs_logger)

main()