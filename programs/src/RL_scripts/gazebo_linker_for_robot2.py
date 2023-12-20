#! /usr/bin/env python
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
sys.path.remove(ros_path)
from stable_baselines import PPO2
import stable_baselines
import gym
from gym.utils import seeding
from gym import spaces
sys.path.append(ros_path)
import rospy
from sensor_msgs.msg import PointCloud2, JointState
from std_msgs.msg import Int32 , Float64
import ros_numpy
from geometry_msgs.msg import PointStamped , Point
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelConfiguration , SetModelConfigurationRequest
import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
print("Import Successful")

class ur(gym.Env):
	def __init__(self):
		rospy.init_node("ur_gym_class_robot2")
		self.shoulder_pan_pub = rospy.Publisher('/robot2/Ashoulder_pan_joint_velocity_controller/command' , Float64 , queue_size = 1)
		self.shoulder_lift_pub = rospy.Publisher('/robot2/Ashoulder_lift_joint_velocity_controller/command', Float64  , queue_size = 1)
		self.elbow_pub = rospy.Publisher('/robot2/Aelbow_joint_velocity_controller/command' , Float64 , queue_size = 1)
		self.wrist_1_pub = rospy.Publisher('/robot2/Awrist_1_joint_velocity_controller/command' , Float64 , queue_size = 1)
		self.wrist_2_pub = rospy.Publisher('/robot2/wrist_2_joint_velocity_controller/command' , Float64 , queue_size = 1)
		self.wrist_3_pub = rospy.Publisher('/robot2/wrist_3_joint_velocity_controller/command' , Float64 , queue_size = 1)
		self.old_dist_to_goal = 10
		self.cumulated_episode_reward = 0
		self.episode_num = 0
		self.goal_weight = 1
		self.action_space = spaces.Box(-4 , 4 ,( 6,))
		self.observation_space = spaces.Box(-np.inf , np.inf , (90,))#Correction Required #Correct on 26May 2022
		self.step_count = 0
		#rospy.loginfo("Robot Gym Env Initialized")
		#rospy.spin()
	def seed(self, seed = None):
		self.np_random  , seed  = seeding.np_random(seed)
		#rospy.loginfo("Seed function")
		return [seed]
	
	def _stop_moving(self):
		self.shoulder_pan_pub.publish(0)
		self.shoulder_lift_pub.publish(0)
		self.elbow_pub.publish(0)
		self.wrist_1_pub.publish(0)
		self.wrist_2_pub.publish(0)
		self.wrist_3_pub.publish(0)
		#rospy.loginfo("Stopping Motion")
	
	def step(self, action):
		self.shoulder_pan_pub.publish(action[0])
		self.shoulder_lift_pub.publish(action[1])
		self.elbow_pub.publish(action[2])
		self.wrist_1_pub.publish(action[3])
		self.wrist_2_pub.publish(action[4])
		self.wrist_3_pub.publish(action[5])
		rospy.sleep(0.3)
		self._stop_moving()
		obs = self._get_obs()
		info = {}
		self.step_count = self.step_count +1 
		done = self._is_done()
		reward = self._compute_reward(obs , done)
		self.cumulated_episode_reward += reward
		#rospy.loginfo("Step Success")
		return obs , reward, done , info
		
	def reset(self):
		self._stop_moving()
		reset_joints = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
		reset_req = SetModelConfigurationRequest()
		reset_req.model_name= 'robot2'
		reset_req.urdf_param_name = 'robot_description'
		reset_req.joint_names=['Ashoulder_pan_joint', 'Ashoulder_lift_joint' , 'Aelbow_joint', 'Awrist_1_joint', 'Awrist_2_joint' , 'Awrist_3_joint']
		reset_req.joint_positions =[0.0 , 0.0 , 0.0 ,0.0 ,0.0 ,0.0 ]
		res = reset_joints(reset_req)
		self.step_count = 0
		#rospy.loginfo("Gazebo Reset Done for Robot2")
		self.episode_num += 1
		self.cumulated_episode_reward = 0
		#rospy.loginfo("Reward reset done for Robot2")
		rospy.sleep(2)
		obs = self._get_obs()
		#rospy.loginfo("Obs Received")
		return obs	
		
		
		
		
	
	def _get_obs(self):
		states_cnt = np.ones(90) * 10
		
		self.robot_states = rospy.wait_for_message("/robot2/joint_states" , JointState , timeout=5)
		for i in range(len(self.robot_states.position)):
			states_cnt[i] = self.robot_states.position[i]
		#data = rospy.wait_for_message("/RL_States/Nearest_Obstacles_States/robot2" , PointCloud2)
		#np_data = ros_numpy.numpify(data)
		data = rospy.wait_for_message("robot_as_obstacle/robot2" , PointCloud2 , timeout=5)
		np_data_1 = ros_numpy.numpify(data)
		#np_data = np.concatenate((np_data ,  np_data_1 ) , axis = 0)
		#np_data_1 = np_data_1.sort(order = "distances")
		np_data_1 = np.sort(np_data_1 , order="distances")
		self.points = np.zeros((np_data_1.shape[0] , 3))
		self.points[: , 0] = np_data_1['x']
		self.points[: , 1] = np_data_1['y']
		self.points[: , 2] = np_data_1['z']
		self.points = self.points.reshape((self.points.shape[0] * self.points.shape[1]))
		for i in range(len(self.points)):
			states_cnt[i + len(self.robot_states.position)] = self.points[i]
		#rospy.loginfo("Get Obs Success")
		return states_cnt
		
	def _is_done(self):
		self.data = rospy.wait_for_message("/Dist_to_Goal/robot2" , Float64 , timeout=5)
		done = False
		self.data = self.data.data
		if(self.data < 0.10):
			done = True
		if(self.step_count > 40):
			done = True
		return done
	
	def _compute_reward(self , obs , done ):
		#least_distance = rospy.wait_for_message("/Least_distance/robot2" , PointCloud2)
		distances = rospy.wait_for_message("robot_as_obstacle/robot2" , PointCloud2 , timeout=5)
		distances = ros_numpy.numpify(distances)
		#least_distance = ros_numpy.numpify(least_distance)
		#fore_arm = min(least_distance['distance'][0] , distances['distances'][0])
		#upper_arm = min(least_distance['distance'][1] , distances['distances'][1])
		#wrist_1 = min(least_distance['distance'][2] , distances['distances'][2])
		#tool = min(least_distance['distance'][3] , distances['distances'][3])
		fore_arm = distances['distances'][0]
		upper_arm = distances['distances'][1]
		wrist_1 = distances['distances'][2]
		tool = distances['distances'][3]
		obs_wsum = 0.25 * fore_arm + 0.25 * upper_arm  + 0.25  * wrist_1 + 0.25 * tool
		fore_arm_ok = False
		upper_arm_ok = False
		wrist_1_ok = False
		tool_ok = False
		#print("Fore_arm  : " , fore_arm)
		#print("Upper_arm : " , upper_arm)
		#print("Wrist_1 : " , wrist_1)
		#print(fore_arm)
		#print(upper_arm)
		#print(wrist_1)
		#print(tool)
		if (fore_arm > 0.40):
			fore_arm_ok = True
		if (upper_arm > 0.40):
			upper_arm_ok = True
		if (wrist_1 > 0.40):
			wrist_1_ok = True
		if(tool > 0.40):
			tool_ok = True
		obstacle_ok = fore_arm_ok and upper_arm_ok and wrist_1_ok and tool_ok
		#obstacle_or =  or upper_arm_ok or wrist_1_ok or tool_ok
		self.data = rospy.wait_for_message("/Dist_to_Goal/robot2" , Float64 , timeout=5)
		self.data = self.data.data
		self.dist_to_goal  = self.data
		self.dist_to_goal_weighted = self.dist_to_goal * self.goal_weight
		#print(self.dist_to_goal)
		self.goal_nearing = self.dist_to_goal < self.old_dist_to_goal
		##self.min_obs_dist_weighted = obstacle * self.obs_weight
		#print("dist to goal : " , self.dist_to_goal_weighted)
		#print("obstacle_ok : " , obstacle_ok)
		'''
		if(0.2 < self.dist_to_goal_weighted <= 0.5):
			self.reward = 50
		elif(0.05 < self.dist_to_goal_weighted <= 0.2):
			self.reward = 100
		elif( self.dist_to_goal_weighted <= 0.05):
			self.reward = 200
		else: 
			self.reward = 25
		if ( self.goal_nearing):
			self.reward  = self.reward + 50
		if(obstacle_ok ==False):
			self.reward = self.reward - 250
		'''
		'''
		self.reward =  obstacle_ok - self.dist_to_goal
		
		if self.dist_to_goal < 0.1:
			self.reward = 1
		'''
		#print(obs_wsum)
		#print(obstacle_ok)
		if ( obstacle_ok == False):
			self.reward  = 2 * obs_wsum - 5
			#print(obs_wsum)
		else : 
			self.reward = 0
		self.reward = self.reward - (self.dist_to_goal / 2)
		if (self.dist_to_goal < 0.15 ):
			self.reward = 2
		rew  = "Reward is = " + str(self.reward)
		#print(self.dist_to_goal)
		self.old_dist_to_goal = self.dist_to_goal
		#print(rew)
		#rospy.loginfo("Reward Generated")
		return self.reward
	
		
		
