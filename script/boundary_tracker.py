#!/usr/bin/env python
from math import *
import numpy as np
from tf.transformations import euler_from_quaternion

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32

from vicon.msg import Subject

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
    Omeg = .1
    # attarction
    Attrac = .1


    # ####### Control Loop ###########
    while not rospy.is_shutdown():
        rospy.sleep(0.1)

        if pose is None:
            print 'Error: no localization.'
            continue

        # Robot pose
        rx, ry = pose.position.x, pose.position.y
        quat = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        rth = euler_from_quaternion(quat)[0]


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


        # vel.angular.z = .2 * (atan2(vtotal[1], vtotal[0]) - rth)
        # vel.angular.z =  0.2 * (map_theta - rth + pi)
        # vel.angular.z =  0.2 * (0- rth)
        # print degrees(rth), degrees(atan2(ry, rx)), vel.angular.z

        print vtotal

        velPub.publish(vel)


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
