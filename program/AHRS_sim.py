
class ahrs_sim():
    yaw = 0
    pitch = 0
    roll = 0
    free_acc = [0, 0, 0]
    rate_of_turn = [0, 0, 0]

    def get_data(self):
        """
        To call from oudstide the function
        :return: {'lineA_x':0,'lineA_y':0,'lineA_z':0, 'angularA_x':0,'angularA_y':0,'angularA_z':0, 'yaw':0,'pitch':0,'roll':0}
        """
        data = {}
        data['lineA_x'] = (self.free_acc[0])
        data['lineA_y'] = (self.free_acc[1])
        data['lineA_z'] = (self.free_acc[2])

        data['roll'] = (self.roll)
        data['pitch'] = (self.pitch)
        data['yaw'] = (self.yaw)

        data['angularA_x'] = (self.rate_of_turn[0])
        data['angularA_y'] = (self.rate_of_turn[1])
        data['angularA_z'] = (self.rate_of_turn[2])

        return data

    def run(self):
        while True:
            pass