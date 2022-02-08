#! /usr/bin/env python
''' MOVING PR2 BASE'''



import rospy 
from geometry_msgs.msg import Twist
rospy.init_node("pointfrommove")
vel_pub = rospy.Publisher("/robot3/base_controller/command" , Twist  , queue_size=10)

move = Twist()
rate = rospy.Rate(10)
while True:
    move.linear.x = -10
    vel_pub.publish(move)
    rate.sleep()
    rospy.loginfo("Updates")
