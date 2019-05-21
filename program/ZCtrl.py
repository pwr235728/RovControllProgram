from RovPID import *
import math

class ZCtrl:

    # regulator pochylenia
    __pitch_speedPID_limit = 100.0  # maksymalna moc - ograniczenie wyjścia PID od prędkości
    __pitch_positionPID_limit = 40.0  # maksymalna prędkość - ograniczenie wyjścia PID od pozycji
    __pid_pitch_params_out = PidParams(Kp=2.0, Ki=0.0, Kd=0.0, Limit=__pitch_positionPID_limit)
    __pid_pitch_params_in = PidParams(Kp=4.0, Ki=0.0, Kd=0.0, Limit=__pitch_speedPID_limit)


    # regulator głębokości
    __depth_speedPID_limit = 100.0      # maksymalna moc - ograniczenie wyjścia PID od prędkości
    __depth_positionPID_limit = 1.0     # maksymalna prędkość zanurzania - ograniczenie wyjścia PID od pozycji
    __pid_depth_params_out = PidParams(Kp=1.0, Ki=0.0, Kd=0.0, Limit=__depth_positionPID_limit)
    __pid_depth_params_in = PidParams(Kp=1.0, Ki=0.0, Kd=0.0, Limit=__depth_speedPID_limit)

    # maksymalny możliwy prąd pobierany przez jeden silnik
    __max_current_consumption_per_thruster = 20.0  # 20A 100% ciagu


    # wartości zwracane - nastawy silników które trzeba ustawić <-100, 100> [%]
    thruster_ZL = 0.0
    thruster_ZR = 0.0
    thruster_ZB = 0.0

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
        self.__pitch_pid = RovPID(outer_loop_params=self.__pid_pitch_params_out,
                                 inner_loop_params=self.__pid_pitch_params_in,
                                 sample_time=self.__sample_time)
        # regulator glebokosci
        self.__depth_pid = RovPID(outer_loop_params=self.__pid_depth_params_out,
                                 inner_loop_params=self.__pid_depth_params_in,
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

    def __pitch_controll(self):
        ahrs_data = self.ahrs.get_data()

        # oblicza rotacje pochylenia rova
        pid_pitch_output = self.__pitch_pid.update(
            outer_loop_feadback=ahrs_data['pitch'],
            inner_loop_feadback=ahrs_data['angularA_y'])

        self.__pitch_ZB = -pid_pitch_output
        self.__pitch_ZL = pid_pitch_output
        self.__pitch_ZR = pid_pitch_output

    def __depth_controll(self):
        pid_depth_output = self.__depth_pid.update(
            outer_loop_feadback=self.bar02.DEPTH,
            inner_loop_feadback=self.bar02.DEPTH_SPEED)

        self.__depth_ZL = pid_depth_output
        self.__depth_ZR = pid_depth_output
        self.__depth_ZB = pid_depth_output

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
        max_thr = max(abs(thrZL), abs(thrZR), abs(thrZB))
        if max_thr > 100.0:
            thrZL = thrZL * 100.0/max_thr
            thrZR = thrZR * 100.0/max_thr
            thrZB = thrZB * 100.0/max_thr

        # ograniczenie mocy
        total = (self.__get_current(thrZL) + self.__get_current(thrZR) + self.__get_current(thrZB))
        norm_factor = 1.0
        if total > self.__power_limit:
            norm_factor = self.__power_limit/total

        self.thruster_ZL = thrZL * norm_factor
        self.thruster_ZR = thrZR * norm_factor
        self.thruster_ZB = thrZB * norm_factor