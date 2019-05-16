from RovPID import *
import math

class ZCtrl:
    __speedPID_limit = 10  # maksymalna moc - ograniczenie wyjścia PID od prędkości
    __positionPID_limit = 90  # maksymalna prędkość - ograniczenie wyjścia PID od pozycji

    __pid_params_out = PidParams(Kp=1.0, Ki=0.0, Kd=0.0, Limit=__positionPID_limit)
    __pid_params_in = PidParams(Kp=1.0, Ki=0.0, Kd=0.0, Limit=__speedPID_limit)

    def __init__(self, ahrs, bar02, sampleTime):
        self.AHRS = ahrs
        self.Bar02 = bar02

        self.SampleTime = sampleTime

        self.Depth = 0.0
        self.Pitch = 0.0

        self.thrusterZL = 0.0
        self.thrusterZR = 0.0
        self.thrusterZB = 0.0

        # regulator pochylenia
        self.__PitchPid = RovPID(outer_loop_params=self.__pid_params_out,
                                 inner_loop_params=self.__pid_params_in,
                                 sample_time=self.SampleTime)

        # sterowania DEPTH
        self.__thr_depth_ZL = 0.0
        self.__thr_depth_ZR = 0.0
        self.__thr_depth_ZB = 0.0

        # sterowania PITCH
        self.__thr_pitch_ZL = 0.0
        self.__thr_pitch_ZR = 0.0
        self.__thr_pitch_ZB = 0.0

    def __pitch_control(self):
        self.__PitchPid.SetPoint = self.Pitch

        # oblicza rotacje pochylenia rova
        tmp = self.__PitchPid.update(outer_loop_feadback=self.AHR.PITCH,
                                     inner_loop_feadback=self.AHRS.PITCH_SPEED)

        self.__thr_pitch_ZB = -tmp
        self.__thr_pitch_ZL = tmp
        self.__thr_pitch_ZR = tmp

    def __depth_control(self):
        self.__thr_depth_ZL = self.Depth
        self.__thr_depth_ZR = self.Depth
        self.__thr_depth_ZB = self.Depth

    def update(self):
        self.__pitch_control()
        self.__depth_control()

        thrZL = self.__thr_depth_ZL + self.__thr_pitch_ZL
        thrZR = self.__thr_depth_ZR + self.__thr_pitch_ZR
        thrZB = self.__thr_depth_ZB + self.__thr_pitch_ZB

        total = math.fabs(thrZL) + math.fabs(thrZR) + math.fabs(thrZB)
        norm_factor = 1.0
        if total > 100:
            norm_factor = 100.0/total

        self.thrusterZL = thrZL
        self.thrusterZR = thrZR
        self.thrusterZB = thrZB