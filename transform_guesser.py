#! /usr/bin/env python

import PyKDL as kdl
import numpy as np
import sys
import rospy
import geometry_msgs.msg


if __name__ == "__main__":
    x = 0.0
    y = 0.0
    z = 0.0
    R = 0.0
    P = 0.0
    Y = 0.0
    if len(sys.argv) == 4:
        z = 0.5
        (R, P, Y) = (float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
    elif len(sys.argv) == 7:
        (x, y, z) = (float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
        (R, P, Y) = (float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6]))
    f = kdl.Frame()
    print "Rotation by RPY(%f,%f,%f)"% (R, P, Y)
    f.M = kdl.Rotation.RPY(R, P, Y)
    print "x: %f\ny: %f\nz: %f\nw: %f\n"%f.M.GetQuaternion()

    rospy.init_node("transform_guesser")
    pub = rospy.Publisher('poser', geometry_msgs.msg.Pose, queue_size=1)
    msg = geometry_msgs.msg.Pose()
    msg.position.x = x
    msg.position.y = y
    msg.position.z = z
    (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w) = f.M.GetQuaternion()
    print msg
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
