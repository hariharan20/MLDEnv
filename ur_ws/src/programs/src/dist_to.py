#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, JointState
import ros_numpy
import numpy as np
from sklearn.cluster import DBSCAN
import tf
from geometry_msgs.msg import PointStamped, Point
#import moveit_commander as mc
from sensor_msgs.msg import JointState
import tf2_msgs.msg
from std_msgs.msg import Float64

rospy.init_node("dist_to_goal")
listener = tf.TransformListener()
goal_dist_pub_2 = rospy.Publisher("Dist_to_Goal/robot2" , Float64 , queue_size =1)
goal_dist_pub_1 = rospy.Publisher("Dist_to_Goal/robot1" , Float64 , queue_size =1)
while not rospy.is_shutdown():
    listener.waitForTransform("/tool0" , "/box_link", rospy.Time(0) , rospy.Duration(4.0))
    pointstamped = PointStamped()
    pointstamped.header.frame_id = "tool0"
    pointstamped.header.stamp = rospy.Time(0)
    p = listener.transformPoint("box_link" , pointstamped)
    a = np.array([p.point.x , p.point.y , p.point.z])
    goal_dist_pub_1.publish(np.linalg.norm(a))
    
    listener.waitForTransform("/Atool0" , "/box2_link", rospy.Time(0) , rospy.Duration(4.0))
    pointstamped = PointStamped()
    pointstamped.header.frame_id = "Atool0"
    pointstamped.header.stamp = rospy.Time(0)
    p = listener.transformPoint("box2_link" , pointstamped)
    a = np.array([p.point.x , p.point.y , p.point.z])
    goal_dist_pub_2.publish(np.linalg.norm(a))
