class RovSim:

    P = 0.99
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

    def set_engines(self, dict):
        self.thrusters = dict


    def update(self):
        rot = self.thrusters["fl"] + self.thrusters["br"] - self.thrusters["fr"] - self.thrusters["bl"]
        tr = self.thrusters["fl"] + self.thrusters["fr"] - self.thrusters["br"] - self.thrusters["bl"]

        rot = rot*100
        tr = tr*100

        pitch = -self.thrusters["vb"] + self.thrusters["vl"]+ self.thrusters["vr"]
        depth = self.thrusters["vb"] + self.thrusters["vr"] + self.thrusters["vl"]

        pitch = pitch*100
        depth = depth*100


        self.heading_speed = self.heading_speed  + rot*self.sample_time
        self.heading = self.heading + self.heading_speed*self.sample_time

        self.pitch_speed= self.pitch_speed + pitch*self.sample_time
        self.pitch = self.pitch + self.pitch_speed * self.sample_time

        self.depth_speed = self.depth_speed + depth*self.sample_time
        self.depth = self.depth +self.depth_speed*self.sample_time

        self.move_speed = self.move_speed*self.P + tr*self.sample_time
