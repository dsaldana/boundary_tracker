import math

import matplotlib
import matplotlib.pyplot as plt
import numpy as np


class RobotDrawer(object):
    def __init__(self, ylim=[-2, 2], xlim=[-2, 2]):
        plt.ion()  # set plot to animated
        self.ax1 = plt.axes()

        plt.grid()
        plt.ylim(ylim)
        plt.xlim(xlim)

        self.grobots = None
        self.gpolygons = None
        plt.draw()

    def init_grobots(self, robots, rad=.51):
        grobots = []

        for (rx, ry, rth) in robots:
            rcircle = plt.plot(rx, ry, 'o', markersize=10)
            rline = plt.plot([rx, rx + rad * math.cos(rth)], [ry, ry + rad * math.sin(rth)])

            grobots.append((rcircle, rline))

        return grobots

    def init_polygons(self, polygons):
        gpolygons = []

        for points in polygons:
            np_points = np.array(points)
            gpol = plt.plot(np_points[:, 0], np_points[:, 1])
            gpolygons.append(gpol)

        return gpolygons

    def update_robots(self, robots, rad=.51):
        if self.grobots is None:
            self.grobots = self.init_grobots(robots)

        for (rcircle, rline), (rx, ry, rth) in zip(self.grobots, robots):
            rcircle[0].set_xdata([rx])
            rcircle[0].set_ydata([ry])
            rline[0].set_xdata([rx, rx + rad * math.cos(rth)])
            rline[0].set_ydata([ry, ry + rad * math.sin(rth)])
        plt.draw()

    def draw_polygons(self, polygons):
        if self.gpolygons is None:
            self.gpolygons = self.init_polygons(polygons)

        for gpol, points in zip(self.gpolygons, polygons):

            np_points = np.array(points)
            gpol[0].set_xdata(np_points[:, 0])
            gpol[0].set_ydata(np_points[:, 1])
        plt.draw()

###### TEST #############
# drawer = RobotDrawer()
#
# import numpy as np
# import time
# while(True):
#     r = np.random.random(3)
#     drawer.update_robots([r])
#     print r
#     time.sleep(1)
