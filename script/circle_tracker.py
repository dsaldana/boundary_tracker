#!/usr/bin/env python
from math import *
import numpy as np
from tf.transformations import euler_from_quaternion

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32

from vicon.msg import Subject
from robot_drawer import RobotDrawer

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


    # Desired angular speed
    Omeg = .5
    # attarction
    Attrac = 1.5


    drawer = RobotDrawer()

    # ####### Control Loop ###########
    while not rospy.is_shutdown():
        rospy.sleep(0.1)

        if pose is None:
            print 'Error: no localization.1'
            continue

        theta = np.linspace(0, 2 * pi, 50)
        boundary = [(cos(th1), sin(th1)) for th1 in theta]

        drawer.draw_polygons([boundary])

        # Robot pose
        rx, ry = pose.position.x, pose.position.y
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rth = euler_from_quaternion(quat)[2]

        drawer.update_robots([(rx, ry, rth)])

        # Circle        
        map_theta = atan2(ry, rx)

        # Tangent
        vt = np.array([-sin(map_theta), cos(map_theta)])
        # Atractor to the unitary radious
        vr = np.array([cos(map_theta), sin(map_theta)]) - np.array([rx, ry])

        # Total vector
        vtotal = Attrac * vr + Omeg * vt

        vel = Twist()
        # Feedback linearization
        vel.linear.x = vtotal[0] * cos(rth) + vtotal[1] * sin(rth)
        vel.angular.z = -vtotal[0] * sin(rth) / .1 + vtotal[1] * cos(rth) / .1

        # The angle is in the contrary direction
        # vel.angular.z = .2 * (atan2(vtotal[1], vtotal[0]) - rth)
        # vel.angular.z =  0.2 * (map_theta - rth + pi)
        # vel.angular.z =  0.2 * (0- rth)
        # print degrees(rth), degrees(atan2(ry, rx)), vel.angular.z

        # vel.linear.x, vel.angular.z = 0., 0
        print "v=%f w=%f" % (vel.linear.x, vel.angular.z)

        velPub.publish(vel)


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
