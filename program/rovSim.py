class RovSim:

    P = 0.95
    Q = 1.0 - P

    def __init__(self, sample_time):
        self.sample_time = sample_time

        self.thrA = 0.0
        self.thrB = 0.0
        self.thrC = 0.0
        self.thrD = 0.0
        self.thrZL = 0.0
        self.thrZR = 0.0
        self.thrZB = 0.0

        self.heading = 0.0
        self.heading_speed = 0.0
        self.pitch = 0.0
        self.pitch_speed = 0.0
        self.depth = 0.0
        self.depth_speed = 0.0

        self.move_speed = 0.0


    def update(self):
        rot = self.thrB + self.thrD - self.thrA - self.thrC
        tr = self.thrB + self.thrA - self.thrD - self.thrC
        pitch = -self.thrZB + self.thrZL + self.thrZR
        depth = self.thrZB + self.thrZR + self.thrZL

        self.heading_speed = self.heading_speed * self.P + rot*self.Q
        self.heading = self.heading + self.heading_speed*self.sample_time

        self.pitch_speed= self.pitch_speed*self.P + pitch*self.Q
        self.pitch = self.pitch + self.pitch_speed * self.sample_time

        self.depth_speed = self.depth_speed*0.99 + depth*0.01
        self.depth = self.depth +self.depth_speed*self.sample_time

        self.move_speed = self.move_speed*self.P + tr*self.Q
