#! /usr/bin/env python

import PyKDL as kdl
import numpy as np
import sys
import rospy
import shape_msgs.msg


if __name__ == "__main__":
    x_min = 0.0
    y_min = 0.0
    z_min = 0.0
    x_max = 0.0
    y_max = 0.0
    z_max = 0.0
    if len(sys.argv) == 7:
        (x_min, y_min, z_min) = (float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
        (x_max, y_max, z_max) = (float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6]))
    
    rospy.init_node("shape_guesser")
    pub = rospy.Publisher('shaper', geometry_msgs.msg.Pose, queue_size=1)
    msg = shape_msgs.msg.SolidPrimitive()
    msg.dimensions = [x_min, y_min, z_min, x_max, y_max, z_max];

    print msg
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
