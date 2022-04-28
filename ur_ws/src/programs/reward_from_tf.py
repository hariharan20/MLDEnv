#!/usr/bin/env python
import rospy, tf2_ros, geometry_msgs.msg
from geometry_msgs.msg import PointStamped
import tf
import numpy as np

if __name__=="__main__":
	rospy.init_node("Reward_from_tf")
	l = tf.TransformListener()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		l.waitForTransform("tool0" , "/box_link" , rospy.Time(0) , rospy.Duration(4.0))
		pointstamped = PointStamped()
		pointstamped.header.frame_id = "tool0"
		pointstamped.header.stamp = rospy.Time(0)
		pointstamped.point.x = 0.0
		pointstamped.point.y = 0.0
		pointstamped.point.z = 0.0
		#rospy.loginfo(pointstamped)
		#rospy.loginfo(l.transformPoint("box_link" , pointstamped))
		p  =l.transformPoint("box_link" , pointstamped)
		a = np.array([p.point.x , p.point.y , p.point.z])
		rospy.loginfo(np.linalg.norm(a))
		
		rate.sleep()
