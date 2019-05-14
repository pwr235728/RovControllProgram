from PID import PID




class PidParams:
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0, Limit=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Limit = Limit


class RovPID:
    def __init__(self, outer_loop_params, inner_loop_params, sample_time):

        self.SetPoint = 0.0
        self.Output = 0.0
        self.SampleTime = sample_time

        self.positionPID = PID(outer_loop_params.Kp,
                               outer_loop_params.Ki,
                               outer_loop_params.Kd,
                               outer_loop_params.Limit)
        self.positionPID.setSampleTime(self.SampleTime)


        self.speedPID = PID(inner_loop_params.Kp,
                            inner_loop_params.Ki,
                            inner_loop_params.Kd,
                            inner_loop_params.Limit)
        self.speedPID.setSampleTime(self.SampleTime)

    def update(self, outer_loop_feadback, inner_loop_feadback):
        self.positionPID.SetPoint = self.SetPoint
        self.positionPID.update(outer_loop_feadback)

        self.speedPID.SetPoint = self.positionPID.output
        self.Output = self.speedPID.update(inner_loop_feadback)

        return self.Output
