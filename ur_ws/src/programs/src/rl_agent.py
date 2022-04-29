#! /usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64 , Int32
import numpy as np
from memory import PPOMemory
from networks import ActorNetwork, CriticNetwork
import tensorflow as tflow
import tensorflow_probability as tfp 
import ros_numpy
from tensorflow.keras.optimizers import Adam
import tensorflow.keras as keras
import tf


class HARI_RL():
	def __init__(self):
		self.obs_points = []
		self.shoulder_pan_pub = rospy.Publisher('/shoulder_pan_joint_velocity_controller/command' , Float64 , queue_size = 10)
		self.shoulder_lift_pub = rospy.Publisher('/shoulder_lift_joint_velocity_controller/command', Float64  , queue_size = 10)
		self.elbow_pub = rospy.Publisher('/elbow_joint_velocity_controller/command' , Float64 , queue_size = 10)
		self.wrist_1_pub = rospy.Publisher('/wrist_1_joint_velocity_controller/command' , Float64 , queue_size = 10)
		self.wrist_2_pub = rospy.Publisher('/wrist_2_joint_velocity_controller/command' , Float64 , queue_size = 10)
		self.wrist_3_pub = rospy.Publisher('/wrist_3_joint_velocity_controller/command' , Float64 , queue_size = 10)
		self.obs_sub = rospy.Subscriber("/RL_States/Nearest_Obstacles_States" , PointCloud2 , cb_obs)
		self.actions =[[-0.1 ,-0.1 ,-0.1],
				[-0.1, -0.1,  0.1],
				[-0.1, 0 , -0.1],
				[-0.1, 0  , 0.1 ]]
		n_actions = len(self.actions)
		self.actor = ActorNetwork(n_action)
		self.critic = CriticNetwork()
		self.actor.compile(optimizer =Adam(learning_rate = self.alpha))
		self.critic.compile(optimizer= Adam(learning_rate= self.alpha))
		self.goal_weight = 1
		self.obs_weight = 1
		
		
		
		
	def stop_moving(self):
		self.shoulder_pan_pub.publish(0)
		self.shoulder_lift_pub.publish(0)
		self.elbow_pub.publish(0)
		self.wrist_1_pub.publish(0)
		self.wrist_2_pub.publish(0)
		self.wrist_3_pub.publish(0)
	
	def cb_obs(self, data):
		np_data = ros_numpy.numpify(data)
		self.points = np.zeros((np_data.shape[0] , 3))
		self.points[ : , 0] = np_data['x']
		self.points[: , 1 ]  = np_data['y']
		self.points[: , 2] = np_data['z']
		self.points = self.points.reshape((self.points.shape[0] * self.points.shape[1]))
	
	def get_current_obs_states(self):
		return self.points
	
	def take_action(self , action_index):
		velocity  = 0 # To be edited for more control over robot's eef
		self.selected_action = self.actions[action_index]
		self.shoulder_pan_pub.publish(self.selected_action[0])
		self.shoulder_lift_pub.publish(self.selected_action[1])
		self.elbow_pub.publish(self.selected_action[2])
		self.wrist_1_pub.publish(velocity)
		self.wrist_2_pub.publish(velocity)
		self.wrist_3_pub.publish(velocity)	
	
	def save_model(self):
		rospy.loginfo("saving the trained model")
		self.actor.save('actor')
		self.critic.save('critic')
	
	def cb_obs_states_post_action(self, data):
		np_data = ros_numpy.numpify(data)
		#self.obs_points_post_action = np.zeros((np_data.shape[0] , 3))
		#self.obs_points_post_action[ : , 0] = np_data['x']
		#self.obs_points_post_action[: , 1 ]  = np_data['y']
		#self.obs_points_post_action[: , 2] = np_data['z']
		self.dist_data = np_data['dist_5']
		self.min_dist = np.amin(self.dist_data)
		#self.obs_points_post_action = self.obs_points_post_action.reshape((self.obs_points_post_action.shape[0] * self.obs_points_post_action.shape[1]))
	
	def get_reward(self):
		#need to check on Sub Callback function types
		self.obs_states_post_action_sub = rospy.Subscriber('/RL_States/Nearest_Obstaacles_States', PointCloud2 , cb_obs_states_post_action)
		l = tf.TransformListener()
		l.waitForTransform("tool0" , "/box_link" , rospy.Time(0) , rospy.Duration(4.0))
		pointstamped = Pointstamped()
		pointstamped.header.frame_id = "tool0"
		pointstamped.header.stamp = rospy.Time(0)
		pointstamped.point.x = 0.0
		pointstamped.point.y = 0.0
		pointstamped.point.z = 0.0
		p = l.tranformPoint("box_link" , pointstamped)
		a = np.array([p.point.x , p.point.y , p.point.z])
		self.dist_to_goal  = np.linalg.norm(a)
		self.dist_to_goal_weighted = self.dist_to_goal * self.goal_weight
		self.min_obs_dist_weighted = self.min_dist * self.obs_weight
		if( self.dist_to_goal_weighted < 0.2 and self.min_obs_dist_weighted > 0.5):
			self.reward = 1
		else : 
			self.reward = 0
		
		return self.reward














def callback(data):
	NUMBER_OF_ACTIONS = 10
	ALPHA =  0.0003
	np_data = ros_numpy.numpify(data)
	points = np.zeros((np_data.shape[0] , 3))
	points[ : , 0] = np_data['x']
	points[: , 1 ]  = np_data['y']
	points[: , 2] = np_data['z']
	points = points.reshape((points.shape[0] * points.shape[1]))
	other_agents_states = tflow.convert_to_tensor(points)
	actor = ActorNetwork(NUMBER_OF_ACTIONS)
	actor.compile(optimizer  = Adam(learning_rate=ALPHA)
	critic = CriticNetwork()
	critic.compile(optimizer = Adam(learning_rate=ALPHA)
	
	probs = actor(other_agent_states)
	dist = tfp.distribution.Categorical(probs)
	action = dist.sample()
	log_prob = dist.log_prob(action)
	value = critic(other_agent_states)
	
	action = action.numpy()[0]
	value = value.numpy()[0]
	log_prob = log_prob.numpy()[0]
	shoulder_pan_pub = rospy.Publisher('/shoulder_pan_joint_velocity_controller/command', Float64, queue_size = 10)
	shoulder_lift_pub = rospy.Publisher('/shoulder_lift_joint_velocity_controller/command', Float64, queue_size= 10)
	elbow_pub = rospy.Publisher('/elbow_joint_velocity_controller/command', Float64, queue_size = 10)
	wrist_1_pub = rospy.Publisher('/wrist_1_joint_velocity_controller/command' , Float64, queue_size=10)
	wrist_2_pub = rospy.Publisher('/wrist_2_joint_velocity_controller/command' , Float64 , queue_size= 10)
	wrist_3_pub = rospy.Publisher('/wrist_3_joint_velocity_controller/command' , Float64 , queue_size=10)
	
	rate = rospy.Rate(0.5)
	actions = [[-0.1 ,-0.1 ,-0.1],
			[-0.1, -0.1,  0.1],
			[-0.1, 0 , -0.1],
			[-0.1, 0  , 0.1 ]]
	#	selected_action = random.choice(actions)
	# action = np.array(selected_action)
	#selected_action = random.choice(actions)
	selected_action = actions[action]
	action = np.array(selected_action)
	velocity = Float64()
	velocity = 0
	rospy.loginfo(action)
	shoulder_pan_pub.publish(action[0])
	shoulder_lift_pub.publish(action[1])
	elbow_pub.publish(action[2])
	wrist_1_pub.publish(velocity)
	wrist_2_pub.publish(velocity)
	wrist_3_pub.publish(velocity)
	rospy.sleep(2)
	l = tf.TransformListener()
	l.waitForTransform("tool0" , "/box_link" , rospy.Time(0) , rospy.Duration(4.0))
	pointstamped = PointStamped()
	pointstamped.header.frame_id = "tool0"
	pointstamped.header.stamp = rospy.Time(0)
	pointstamped.point.x = 0.0
	pointstamped.point.y = 0.0
	pointstamped.point.z = 0.0
	p = l.transformPoint("box_link" , pointstamped)
	goal_dist = np.linalg.norm(np.array[p.point.x , p.point.y ,p.point.z])
	
	
	

if __name__=="__main__":
	rospy.init_node("RL_Agent")
	sub = rospy.Subscriber("/RL_States/Nearest_Obstacles_States" , PointCloud2 , callback)
	
