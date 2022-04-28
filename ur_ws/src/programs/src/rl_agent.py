#! /usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
import numpy as np
from memory import PPOMemory
from networks import ActorNetwork, CriticNetwork
import tensorflow as tflow
import tensorflow_probability as tfp 
import ros_numpy
from tensorflow.keras.optimizers import Adam






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
	rospy.init_node("talker" )
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
	
