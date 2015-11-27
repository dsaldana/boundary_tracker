#!/usr/bin/env python
from math import *

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32

from vicon.msg import Subject



# state
tracking = False
new_tracking_msg = False
# P Control constants for navigation
p_linear = .1
p_angular = .1


P_TRACKING = 0.70


def localization_callback(loc):
    print 'loc=', (loc.position.x, loc.position.y, loc.orientation.z)



def run():
   
    rospy.init_node('boundary_tracker')

    print '--Scarab tracker starting--'



    rospy.Subscriber('/vicon/Scarab_DCT_S45', Subject, localization_callback, queue_size=1)

   

    # ####### Control Loop ###########
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
          


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
