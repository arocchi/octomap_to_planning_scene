#! /usr/bin/env python

import PyKDL as kdl
import numpy as np
import sys
import rospy
import geometry_msgs.msg


if __name__ == "__main__":
	f = kdl.Frame()
	print "Rotation by RPY(%f,%f,%f)"% (float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
	f.M = kdl.Rotation.RPY(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
	print "x: %f\ny: %f\nz: %f\nw: %f\n"%f.M.GetQuaternion()

	rospy.init_node("transform_guesser")
	pub = rospy.Publisher('poser', geometry_msgs.msg.Pose, queue_size=1)
	msg = geometry_msgs.msg.Pose()
	msg.position.z = 0.5
	(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w) = f.M.GetQuaternion()
	print msg
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		pub.publish(msg)
		rate.sleep()
