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
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest
import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
print("Import Successful")

class ur(gym.Env):
	def __init__(self):
		rospy.init_node("ur_gym_class_2")
		self.shoulder_pan_pub = rospy.Publisher('/robot2/Ashoulder_pan_joint_velocity_controller/command' , Float64 , queue_size = 1)
		self.shoulder_lift_pub = rospy.Publisher('/robot2/Ashoulder_lift_joint_velocity_controller/command', Float64  , queue_size = 1)
		self.elbow_pub = rospy.Publisher('/robot2/Aelbow_joint_velocity_controller/command' , Float64 , queue_size = 1)
		self.wrist_1_pub = rospy.Publisher('/robot2/Awrist_1_joint_velocity_controller/command' , Float64 , queue_size = 1)
		self.wrist_2_pub = rospy.Publisher('/robot2/Awrist_2_joint_velocity_controller/command' , Float64 , queue_size = 1)
		self.wrist_3_pub = rospy.Publisher('/robot2/Awrist_3_joint_velocity_controller/command' , Float64 , queue_size = 1)
		
		#rospy.Subscriber("/robot2/joint_states", JointState, self.cb_js)
		rospy.Subscriber("/robot_as_obstacle/robot2" , PointCloud2 , self.cb_rao)
		rospy.Subscriber("/Dist_to_Goal/robot2" , Float64 , self.cb_dtg)
		self.cumulated_episode_reward = 0
		self.episode_num = 0
		self.goal_weight = 1
		self.action_space = spaces.Box(-4 , 4 , (6,))
		self.observation_space = spaces.Box(-np.inf , np.inf , (90,))
		self.step_count = 0
		
	
	def cb_rao(self , data):
		self.rao = data
		
	def cb_dtg(self, data):
		self.dtg = data
		
	def seed(self, seed = None):
		self.np_random , seed = seeding.np_random(seed)
		return [seed]
	def _stop_moving(self):
		self.shoulder_pan_pub.publish(0)
		self.shoulder_lift_pub.publish(0)
		self.elbow_pub.publish(0)
		self.wrist_1_pub.publish(0)
		self.wrist_2_pub.publish(0)
		self.wrist_3_pub.publish(0)
	
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
		self.step_count = self.step_count + 1
		done = self._is_done()
		reward = self._compute_reward(obs , done)
		self.cumulated_episode_reward += reward
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
		self.episode_num += 1
		self.cumulated_episode_reward = 0
		obs = self._get_obs()
		return obs
		
	def _get_obs(self):
		states_cnt = np.ones(90) * 10
		position = []
		req = rospy.ServiceProxy('/gazebo/get_joint_properties' , GetJointProperties)
		joint  = GetJointPropertiesRequest()
		joint.joint_name = "robot2::Aelbow_joint"
		rospy.wait_for_service('/gazebo/get_joint_properties')
		angle_0 = req(joint)
		angle_0 = angle_0.position[0]
		position.append(float(angle_0))
		joint.joint_name = "robot2::Ashoulder_pan_joint"
		rospy.wait_for_service('/gazebo/get_joint_properties')
		angle_0 = req(joint)
		angle_0 = angle_0.position[0]
		position.append(float(angle_0))
		joint.joint_name = "robot2::Ashoulder_lift_joint"
		rospy.wait_for_service('/gazebo/get_joint_properties')
		angle_0 = req(joint)
		angle_0 = angle_0.position[0]
		position.append(float(angle_0))
		joint.joint_name = "robot2::Awrist_1_joint"
		rospy.wait_for_service('/gazebo/get_joint_properties')
		angle_0 = req(joint)
		angle_0 = angle_0.position[0]
		position.append(float(angle_0))
		joint.joint_name = "robot2::Awrist_2_joint"
		rospy.wait_for_service('/gazebo/get_joint_properties')
		angle_0 = req(joint)
		angle_0 = angle_0.position[0]
		position.append(float(angle_0))
		joint.joint_name = "robot2::Awrist_3_joint"
		rospy.wait_for_service('/gazebo/get_joint_properties')
		angle_0 = req(joint)
		#print(angle_0)
		angle_0 = angle_0.position[0]
		position.append(float(angle_0))
		print(position)
		for i in range(6):
			states_cnt[i] = position[i]
		data = self.rao
		data = ros_numpy.numpify(data)
		data = np.sort(data , order = "distances")
		points = np.zeros((data.shape[0] , 3))
		points[: , 0] = data['x']
		points[: , 1] = data['y']
		points[: , 2] = data['z']
		points = points.reshape(points.shape[0] * points.shape[1])
		for i in range(len(points)):
			states_cnt[i+6] = points[i]
		return states_cnt
	
	def _is_done(self):
		done = False
		data   = self.dtg.data
		if(data < 0.10):
			done = True
		if(self.step_count > 40):
			done = True
		return done
	
	def _compute_reward(self, obs , done):
		distances = self.rao
		distances = ros_numpy.numpify(distances)
		fore_arm = distances['distances'][0]
		upper_arm = distances['distances'][1]
		wrist_1 = distances['distances'][2]
		tool = distances['distances'][3]
		obs_wsum = 0.25 * fore_arm + 0.25 * upper_arm  + 0.25  * wrist_1 + 0.25 * tool
		fore_arm_ok = False
		upper_arm_ok = False
		wrist_1_ok = False
		tool_ok = False
		if (fore_arm > 0.40):
			fore_arm_ok = True
		if (upper_arm > 0.40):
			upper_arm_ok = True
		if (wrist_1 > 0.20):
			wrist_1_ok = True
		if(tool > 0.20):
			tool_ok = True
		obstacle_ok = fore_arm_ok and wrist_1_ok and tool_ok
		data = self.dtg.data
		
		if ( obstacle_ok == False):
			reward  = 2 * obs_wsum - 5
		else : 
			reward = 0
		reward = reward - (2 * data)
		if (data < 0.20 ):
			reward = 2
		return reward
		
		
		
		
		
