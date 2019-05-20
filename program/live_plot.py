import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import time
import datetime
style.use("fivethirtyeight")
"""
fig = plt.figure()
ax1 = fig.add_subplot(1, 2, 1)
def animate(i):
    graph_data = open('samplefile.txt', 'r').read()
    lines = graph_data.split('\n')
    xs = []
    ys = []

    for line in lines:
        if len(line) > 1:
            x, y = line.split(',')
            xs.append(x)
            ys.append(y)

    ax1.clear()
    ax1.plot(xs, ys)


ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.show() """

class ahrs_logger:

    HEADING = []
    YAW_SPEED = []

    PITCH = []
    PITCH_SPEED = []

    DEPTH = []
    DEPTH_SPEED = []
    TIME = []

    length = 500

    __last_time = time.time()
    __current_time = time.time()

    def __log_heading(self, ahrs):
        self.HEADING.append(ahrs.HEADING)
        self.YAW_SPEED.append(ahrs.YAW_SPEED)

        if len(self.HEADING) > self.length:
            self.HEADING.pop(0)
            self.YAW_SPEED.pop(0)

    def __log_pitch(self, ahrs):
        self.PITCH.append(ahrs.PITCH)
        self.PITCH_SPEED.append(ahrs.PITCH_SPEED)

        if len(self.PITCH) > self.length:
            self.PITCH.pop(0)
            self.PITCH_SPEED.pop(0)

    def __log_depth(self, ahrs):
        self.DEPTH .append(ahrs.DEPTH )
        self.DEPTH_SPEED.append(ahrs.DEPTH_SPEED)

        if len(self.DEPTH ) > self.length:
            self.DEPTH .pop(0)
            self.DEPTH_SPEED.pop(0)


    def log(self, ahrs, bar02):

        self.__current_time = time.time()
        dt = self.__current_time - self.__last_time
        if dt > 0.02:

            self.__last_time = self.__current_time
            self.__log_heading(ahrs)
            self.__log_pitch(ahrs)
            self.__log_depth(bar02)


class Plotter:
    fig = plt.figure()

    heading_ax = fig.add_subplot(1, 3, 1)
    pitch_ax = fig.add_subplot(1, 3, 2)
    depth_ax = fig.add_subplot(1, 3, 3)

    def __plot_heading(self, ahrs_logger):
        self.heading_ax.clear()
        self.heading_ax.plot(ahrs_logger.HEADING, label='heading')
        self.heading_ax.plot(ahrs_logger.YAW_SPEED, label='heading speed')
        self.heading_ax.legend(loc='best')

    def __plot_pitch(self, ahrs_logger):
        self.pitch_ax.clear()
        self.pitch_ax.plot(ahrs_logger.PITCH, label='pitch')
        self.pitch_ax.plot(ahrs_logger.PITCH_SPEED, label='pitch speed')
        self.pitch_ax.legend(loc='best')

    def __plot_depth(self, ahrs_logger):
        self.depth_ax.clear()
        self.depth_ax.plot(ahrs_logger.DEPTH, label='depth')
        self.depth_ax.plot(ahrs_logger.DEPTH_SPEED, label='depth speed')
        self.depth_ax.legend(loc='best')

    def plot(self, ahrs_logger):
        self.__plot_heading(ahrs_logger)
        self.__plot_pitch(ahrs_logger)
        self.__plot_depth(ahrs_logger)

        self.fig.show()
        plt.pause(0.01)
