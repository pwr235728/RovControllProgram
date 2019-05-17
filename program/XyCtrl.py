from RovPID import *
import math


class XyCtrl:
    __speedPID_limit = 100  # maksymalna moc - ograniczenie wyjścia PID od prędkości
    __positionPID_limit = 90  # maksymalna prędkość - ograniczenie wyjścia PID od pozycji

    __pid_params_out = PidParams(Kp=1.0, Ki=0.0, Kd=0.0, Limit=__positionPID_limit)
    __pid_params_in = PidParams(Kp=3.0, Ki=0.0, Kd=0.0, Limit=__speedPID_limit)

    __max_current_consumtion_per_thruster = 20.0    # 20A 100% thrust

    def __init__(self, ahrs, sampleTime):
        # AHRS
        self.ahrs = ahrs

        # czas wykonania petli sterujacej
        self.__sample_time = sampleTime

        # parametry do kontroli rov-a
        self.__heading = 0.0      # orientacja rov-a
        self.__direction = 0.0    # kierunek ruchu rov-a
        self.__power = 0.0        # moc nastaw o zakresie: <-100.0, 100.0>
        self.__power_limit = 10.0   # 10A lacznie na silniki ?

        # nastawy silnikow
        self.__thruster_A = 0.0
        self.__thruster_B = 0.0
        self.__thruster_C = 0.0
        self.__thruster_D = 0.0

        # nastawy obrotu rov-a : zakres <-100.0, 100.0>
        self.__A_h = 0.0
        self.__B_h = 0.0
        self.__C_h = 0.0
        self.__D_h = 0.0

        # wagi poruszanie rov-a : zakres <-1.0, 1.0>
        self.__A_w = 0.0
        self.__B_w = 0.0
        self.__C_w = 0.0
        self.__D_w = 0.0

        # regulator obrotu
        self.__heading_pid = RovPID(outer_loop_params=self.__pid_params_out,
                                    inner_loop_params=self.__pid_params_in,
                                    sample_time=self.__sample_time)

    @property
    def sample_time(self):
        return self.__sample_time
    @sample_time.setter
    def sample_time(self, sample_time):
        self.__sample_time = sample_time
        self.__heading_pid.SampleTime = sample_time

    @property
    def heading(self):
        return self.__heading
    @heading.setter
    def heading(self, heading):
        self.__heading = heading
        self.__heading_pid.SetPoint = heading

    @property
    def direction(self):
        return self.__direction
    @direction.setter
    def direction(self, direction):
        self.__direction = direction

    @property
    def power(self):
        return self.__power
    @power.setter
    def power(self, power):
        self.__power = power

    @property
    def power_limit(self):
        return self.__power_limit
    @power_limit.setter
    def power_limit(self, power_limit):
        self.__power_limit = power_limit

    def __heading_control(self):
        # oblicza rotacje rova
        tmp = self.__heading_pid.update(outer_loop_feadback=self.ahrs.HEADING,
                                 inner_loop_feadback=self.ahrs.YAW_SPEED)

        self.__A_h = 0.0
        self.__B_h = 0.0
        self.__C_h = 0.0
        self.__D_h = 0.0

        if tmp > 0.0:
            self.__B_h = tmp
            self.__D_h = tmp

        if tmp < 0.0:
            self.__A_h = -tmp
            self.__C_h = -tmp

    def __direction_control(self):
        self.__B_w = math.cos(self.__direction - math.radians(45))
        self.__D_w = -math.cos(self.__direction - math.radians(45))

        self.__A_w = math.cos(self.__direction + math.radians(45))
        self.__C_w = -math.cos(self.__direction + math.radians(45))

    def __get_current(self, value):
        return self.__max_current_consumtion_per_thruster * abs(value) / 100.0

    def update(self):
        self.__heading_control()
        self.__direction_control()

        # wyliczenie nastaw
        thrA = self.__A_w * self.__power + self.__A_h
        thrB = self.__B_w * self.__power + self.__B_h
        thrC = self.__C_w * self.__power + self.__C_h
        thrD = self.__D_w * self.__power + self.__D_h

        # normalizacaj do 100% zakresu
        max_thr = max(abs(thrA), abs(thrB), abs(thrC), abs(thrD))
        if max_thr > 100.0:
            thrA = thrA * 100.0/max_thr
            thrB = thrB * 100.0/max_thr
            thrC = thrC * 100.0/max_thr
            thrD = thrD * 100.0/max_thr


        # ograniczenie mocy
        total = (self.__get_current(thrA) + self.__get_current(thrB)
                 + self.__get_current(thrC) + self.__get_current(thrD))
        norm_factor = 1.0
        if total > self.__power_limit:
            norm_factor = self.__power_limit/total

        # aktualizacja nastaw silnikow
        self.thrusterA = thrA * norm_factor
        self.thrusterB = thrB * norm_factor
        self.thrusterC = thrC * norm_factor
        self.thrusterD = thrD * norm_factor
