#!/usr/bin/env python
from math import *
import numpy as np
from tf.transformations import euler_from_quaternion

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32

from vicon.msg import Subject
import time
from robot_drawer import RobotDrawer
import pickle
from shapely.geometry import Polygon, LinearRing, Point, LineString
from numpy import linalg as LA

goal = Point32()
goal.x = 0.0000001
goal.y = 0.0000001

# state
tracking = False
new_tracking_msg = False
# P Control constants for navigation
p_linear = .1
p_angular = .1

P_TRACKING = 0.70

pose = None

import matplotlib
import matplotlib.pyplot as plt

plt.ion()  # set plot to animated


def localization_callback(loc):
    global pose

    pose = loc


def run():
    rospy.init_node('boundary_tracker')

    print '--Scarab tracker starting--'


    ## Listen for the localization
    rospy.Subscriber('/vicon/Scarab_DCT_S45', Subject, localization_callback, queue_size=1)
    # Topic to control the robot velocity
    velPub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


    # initial time
    init_time = time.time()

    # Load boundaries
    boundaries = pickle.load(open("boundary.p", "rb"))

    log = []



    # Desired angular speed
    Omeg = .3
    # attarction
    Attrac = .6

    drawer = RobotDrawer()
    vel = Twist()


    # ####### Control Loop ###########
    while not rospy.is_shutdown():
        rospy.sleep(0.1)

        # Compute boundary time
        t = time.time()
        boundary_time = int(t - init_time) + 30

        if boundary_time == len(boundaries):
            pickle.dump(log, open("experiment%d.p" % t, "wb"))

            # Stop the robot
            vel.linear.x, vel.angular.z = 0., 0
            velPub.publish(vel)
            break

        boundary = np.array(boundaries[boundary_time])
        boundary[:, 0] -= 1
        boundary[:, 1] -= .7
        boundary[:] *= 1.5

        drawer.draw_polygons([boundary])

        if pose is None:
            print 'Error: no localization.1'
            continue


        ##### Robot pose
        rx, ry = pose.position.x, pose.position.y
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rth = euler_from_quaternion(quat)[2]

        drawer.update_robots([(rx, ry, rth)])

        #### Control
        clst = point_to_follow(Polygon(boundary), Point((rx, ry)), .2)

        # vector to approach to the desired point
        vr = np.array([clst[0] - rx, clst[1] - ry])

        # Total vector
        vtotal = 2 * vr


        # Feedback linearization
        vel.linear.x = vtotal[0] * cos(rth) + vtotal[1] * sin(rth)
        vel.angular.z = -vtotal[0] * sin(rth) / .1 + vtotal[1] * cos(rth) / .1

        ## LOG
        log.append((boundary_time, t, rx, ry, rth))

        # vel.linear.x, vel.angular.z = 0., 0
        # print "v=%f w=%f" % (vel.linear.x, vel.angular.z)
        print boundary_time

        velPub.publish(vel)


def point_to_follow(poly, point, dist):
    """point in the boundar, that is after
    """
    pol_ext = LinearRing(poly.exterior.coords)
    d = pol_ext.project(point) + dist
    p = pol_ext.interpolate(d )
    closest = list(p.coords)[0]
    return closest


def closest_on_polygon(poly, point):
    """Closest point in a polygon to an external point
    """
    pol_ext = LinearRing(poly.exterior.coords)
    d = pol_ext.project(point)
    p = pol_ext.interpolate(d)
    closest = list(p.coords)[0]
    return closest


def tanget_dir(point_in, points):
    p = Point(point_in).buffer(.0001)
    for p1, p2 in zip(points[:-1], points[1:]):

        l = LineString([p1, p2])
        if l.intersects(p):
            vec = np.array(p2) - np.array(p1)
            return vec / LA.norm(vec)
    return None


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
