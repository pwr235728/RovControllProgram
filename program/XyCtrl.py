from RovPID import *
import math


class XyCtrl:
    __speedPID_limit = 100  # maksymalna moc - ograniczenie wyjścia PID od prędkości
    __positionPID_limit = 90  # maksymalna prędkość - ograniczenie wyjścia PID od pozycji

    __pid_params_out = PidParams(Kp=1.0, Ki=0.0, Kd=0.0, Limit=__positionPID_limit)
    __pid_params_in = PidParams(Kp=1.0, Ki=0.0, Kd=0.0, Limit=__speedPID_limit)

    def __init__(self, ahrs, sampleTime):
        # AHRS
        self.ahrs = ahrs

        # czas wykonania petli sterujacej
        self.__sample_time = sampleTime

        # parametry do kontroli rov-a
        self.__heading = 0.0      # orientacja rov-a
        self.__direction = 0.0    # kierunek ruchu rov-a
        self.__power = 0.0        # moc nastaw o zakresie: <-100.0, 100.0>

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
                                    sample_time=self.SampleTime)

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

    def update(self):
        self.__heading_control()
        self.__direction_control()

        # wyliczenie nastaw
        thrA = self.__A_w * self.Power + self.__A_h
        thrB = self.__B_w * self.Power + self.__B_h
        thrC = self.__C_w * self.Power + self.__C_h
        thrD = self.__D_w * self.Power + self.__D_h

        # normalizacja do 100% mocy
        total = math.fabs(thrA) + math.fabs(thrB) + math.fabs(thrC) + math.fabs(thrD)
        norm_factor = 1.0
        if total > 100:
            norm_factor = 100.0/total

        # aktualizacja nastaw silnikow
        self.thrusterA = thrA * norm_factor
        self.thrusterB = thrB * norm_factor
        self.thrusterC = thrC * norm_factor
        self.thrusterD = thrD * norm_factor
