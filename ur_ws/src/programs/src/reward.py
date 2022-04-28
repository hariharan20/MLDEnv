#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy
from std_msgs.msg import Int32
import moveit_commander
import numpy as np
OBS_WEIGHT = 1
GOAL_WEIGHT = 1
def callback(data):
    data_np = ros_numpy.numpify(data)
    x = data_np['dist_5']
    MIN_DIST = np.amin(x)
    goal = np.zeros((3))
    pub = rospy.Publisher("/reward_value" , Int32, queue_size= 10)
    goal[0] = -0.315
    goal[1] = 0.792
    goal[2] = -0.005
    #robot = moveit_commander.RobotCommander()

    current_eef_position = group.get_current_pose()
    curr = np.zeros((3))
    curr[0] = current_eef_position.pose.position.x
    curr[1] = current_eef_position.pose.position.y
    curr[2] = current_eef_position.pose.position.z
    DIST_TO_GOAL = np.linalg.norm(curr - goal)
    #rospy.loginfo(DIST_TO_GOAL)
    reward = Int32()
    if(GOAL_WEIGHT * DIST_TO_GOAL < 0.2 and OBS_WEIGHT * MIN_DIST > 0.5 ):
        reward = 1
    else : 
        reward = 0
    rospy.loginfo(reward)
    pub.publish(reward)

if __name__=="__main__":
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
    rospy.init_node("Reward_Function")
    sub = rospy.Subscriber("/RL_States/Obs_centres_with_distances" , PointCloud2, callback)
    rospy.spin()
