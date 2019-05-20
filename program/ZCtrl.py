from RovPID import *
import math

class ZCtrl:

    __speedPID_limit = 100  # maksymalna moc - ograniczenie wyjścia PID od prędkości
    __positionPID_limit = 90  # maksymalna prędkość - ograniczenie wyjścia PID od pozycji

    __pid_params_out = PidParams(Kp=2.0, Ki=0.0, Kd=0.0, Limit=__positionPID_limit)
    __pid_params_in = PidParams(Kp=4.0, Ki=0.0, Kd=0.0, Limit=__speedPID_limit)

    __max_current_consumption = 20.0  # 20A 100% ciagu

    def __init__(self, ahrs, bar02, sampleTime):
        self.ahrs = ahrs
        self.bar02 = bar02

        self.__sample_time = sampleTime
        self.__depth = 0.0
        self.__pitch = 0.0
        self.__power_limit = 10.0  # 10A lacznie na silniki ?

        self.__thruster_ZL = 0.0
        self.__thruster_ZR = 0.0
        self.__thruster_ZB = 0.0

        # regulator pochylenia
        self.__pitch_pid = RovPID(outer_loop_params=self.__pid_params_out,
                                 inner_loop_params=self.__pid_params_in,
                                 sample_time=self.__sample_time)
        # regulator glebokosci
        self.__depth_pid = RovPID(outer_loop_params=self.__pid_params_out,
                                 inner_loop_params=self.__pid_params_in,
                                 sample_time=self.__sample_time)
        # sterowania DEPTH
        self.__depth_ZL = 0.0
        self.__depth_ZR = 0.0
        self.__depth_ZB = 0.0

        # sterowania PITCH
        self.__pitch_ZL = 0.0
        self.__pitch_ZR = 0.0
        self.__pitch_ZB = 0.0

    @property
    def sample_time(self):
        return self.__sample_time
    @sample_time.setter
    def sample_time(self, sample_time):
        self.__sample_time = sample_time
        self.__pitch_pid.SampleTime = sample_time

    @property
    def depth(self):
        return self.__depth
    @depth.setter
    def depth(self, depth):
        self.__depth = depth
        self.__depth_pid.SetPoint = depth

    @property
    def pitch(self):
        return self.__pitch
    @pitch.setter
    def pitch(self, pitch):
        self.__pitch = pitch
        self.__pitch_pid.SetPoint = pitch

    @property
    def thruster_ZL(self):
        return self.__thruster_ZL

    @property
    def thruster_ZR(self):
        return self.__thruster_ZR

    @property
    def thruster_ZB(self):
        return self.__thruster_ZB

    def __pitch_controll(self):

        # oblicza rotacje pochylenia rova
        tmp = self.__pitch_pid.update(outer_loop_feadback=self.ahrs.PITCH,
                                     inner_loop_feadback=self.ahrs.PITCH_SPEED)

        self.__pitch_ZB = -tmp
        self.__pitch_ZL = tmp
        self.__pitch_ZR = tmp

    def __depth_controll(self):
        tmp = self.__depth_pid.update(outer_loop_feadback=self.bar02.DEPTH,
                                      inner_loop_feadback=self.bar02.DEPTH_SPEED)

        self.__depth_ZL = tmp
        self.__depth_ZR = tmp
        self.__depth_ZB = tmp

    def __get_current(self, value):
        return self.__max_current_consumption_per_thruster * abs(value) / 100.0


    def update(self):
        self.__pitch_controll()
        self.__depth_controll()

        # wyliczenie nastaw
        thrZL = self.__depth_ZL + self.__pitch_ZL
        thrZR = self.__depth_ZR + self.__pitch_ZR
        thrZB = self.__depth_ZB + self.__pitch_ZB

        # normalizacja do 100% mocy
        total = math.fabs(thrZL) + math.fabs(thrZR) + math.fabs(thrZB)
        norm_factor = 1.0
        if total > 100:
            norm_factor = 100.0/total

        self.__thruster_ZL = thrZL
        self.__thruster_ZR = thrZR
        self.__thruster_ZB = thrZB