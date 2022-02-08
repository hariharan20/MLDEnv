#! /usr/bin/env python
'''PREDEFINED MOTION FOR PR2 1ST ROBOT AS AN OBSTACLE'''



import rospy
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint 
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import math
def rad(deg):
    rad = 0
    radi = deg/180
    radi = radi*math.pi
    return radi
move = Twist()
rospy.init_node("pr2_2_move")
pub_vel = rospy.Publisher("/robot4/base_controller/command" , Twist , queue_size= 10)
pub_arm = rospy.Publisher("/robot4/r_arm_controller/command" , JointTrajectory , queue_size=10)
pub_l_arm = rospy.Publisher("/robot4/l_arm_controller/command" , JointTrajectory , queue_size=10)
jta = JointTrajectory()
jta.header = Header()
jta.header.stamp = rospy.Time.now()
jta.joint_names = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint','r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']


jtl = JointTrajectory()
jtl.header = Header()
jtl.header.stamp = rospy.Time.now()
jtl.joint_names =  ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint','l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']



pointr = JointTrajectoryPoint()
pointr.positions = [ -0.09 , rad(-500) , rad(-180) , -0.6 , 0 , 0 , 0]
pointr.time_from_start = rospy.Duration(2)

pointl = JointTrajectoryPoint()
pointl.positions = [ 0.09, rad(-500) , rad(180) , -0.6 , 0 , 0 , 0]
pointl.time_from_start = rospy.Duration(3)


rate = rospy.Rate(10)
jta.points.append(pointr)
jtl.points.append(pointl)
x = 0



x = 0
while x < 3 :
    x = x+1
    pub_arm.publish(jta)
    #rospy.loginfo("position updated")
    rate.sleep()


x = 0
     
while x < 3 :
    x = x+1
    pub_l_arm.publish(jtl)
    #rospy.loginfo("position left updated")
    rate.sleep()

while True:
    x = 0
    while x < 15 : 
        x = x+1
        move.linear.x = 0.5
        pub_vel.publish(move)
        rospy.loginfo("vel_updated")
        rate.sleep()

    x = 0
    while x < 15 : 
        x = x+1
        move.linear.x = -0.5
        pub_vel.publish(move)
        rospy.loginfo("vel_updated")
        rate.sleep()
 