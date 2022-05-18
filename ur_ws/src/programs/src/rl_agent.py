#! /usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2 , JointState
from std_msgs.msg import Float64 , Int32
import numpy as np
from memory import PPOMemory
from networks import ActorNetwork, CriticNetwork
import tensorflow as tf
import tensorflow_probability as tfp 
import ros_numpy
from tensorflow.keras.optimizers import Adam
import tensorflow.keras as keras
import tf as tframe
from memory import PPOMemory
from geometry_msgs.msg import PointStamped , Point
import matplotlib.pyplot as plt
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SetModelConfigurationRequest

class HARI_RL():
	def __init__(self):
		self.obs_points = []
		self.shoulder_pan_pub = rospy.Publisher('/shoulder_pan_joint_velocity_controller/command' , Float64 , queue_size = 10)
		self.shoulder_lift_pub = rospy.Publisher('/shoulder_lift_joint_velocity_controller/command', Float64  , queue_size = 10)
		self.elbow_pub = rospy.Publisher('/elbow_joint_velocity_controller/command' , Float64 , queue_size = 10)
		self.wrist_1_pub = rospy.Publisher('/wrist_1_joint_velocity_controller/command' , Float64 , queue_size = 10)
		self.wrist_2_pub = rospy.Publisher('/wrist_2_joint_velocity_controller/command' , Float64 , queue_size = 10)
		self.wrist_3_pub = rospy.Publisher('/wrist_3_joint_velocity_controller/command' , Float64 , queue_size = 10)
		#self.obs_sub = rospy.Subscriber("/RL_States/Nearest_Obstacles_States" , PointCloud2 , self.cb_obs)
		#self.actions =[[-0.05 ,-0.05 ,-0.05],
		#		[-0.05, -0.05,  0.05],
		#		[-0.05, 0 , -0.05],
		#		[-0.05, 0  , 0.05 ]]
		action_for_one_joint = np.linspace(-0.5  , 0.5 ,10)
		self.actions = np.array(np.meshgrid(action_for_one_joint ,action_for_one_joint , action_for_one_joint)).T.reshape(-1 , 3)
		n_actions = len(self.actions)
		self.actor = ActorNetwork(n_actions)
		self.critic = CriticNetwork()
		self.alpha =0.5
		self.actor.compile(optimizer =Adam(learning_rate = self.alpha))
		self.critic.compile(optimizer= Adam(learning_rate= self.alpha))
		print("Compiled Both Actor and Critic")
		self.goal_weight = 1
		self.obs_weight = 1
		self.batch_size = 64
		self.memory  = PPOMemory(self.batch_size)
		self.n_epochs = 10	
		self.gamma = 0.99
		self.gae_lambda = 0.95
		self.policy_clip = 0.2	
		self.robot_position = []
		self.robot_velocity = []
		self.l = tframe.TransformListener()
		self.old_dist_to_goal = 2
	def cb_get_states(self , data):
		#rospy.loginfo(data.position)
		self.robot_position  = np.array(data.position)#[1]
		self.robot_velocity = np.array(data.velocity)#[1]
	def get_states(self):
		#rospy.wait_for
		#rospy.Subscriber("joint_states" , JointState , self.cb_get_states)
		self.robot_states  = rospy.wait_for_message("joint_states" , JointState )
		self.robot_position = tf.expand_dims(self.robot_states.position , 0)
		print("updated the robot joint positions")
	def stop_moving(self):
		self.shoulder_pan_pub.publish(0)
		self.shoulder_lift_pub.publish(0)
		self.elbow_pub.publish(0)
		self.wrist_1_pub.publish(0)
		self.wrist_2_pub.publish(0)
		self.wrist_3_pub.publish(0)
	
	def cb_obs(self):
		data  = rospy.wait_for_message("/RL_States/Nearest_Obstacles_States" , PointCloud2)
		np_data = ros_numpy.numpify(data)
		self.points = np.zeros((np_data.shape[0] , 3))
		self.points[ : , 0] = np_data['x']
		self.points[: , 1 ]  = np_data['y']
		self.points[: , 2] = np_data['z']
		self.points = tf.expand_dims(self.points , 0)
		print(np_data.shape[0])
		print("got the obs_points  and updated")
		#self.points = self.points.reshape((self.points.shape[0] * self.points.shape[1]))
	
	def take_action(self , action_index):
		velocity  = 0 # To be edited for more control over robot's eef
		self.selected_action = self.actions[action_index]
		self.shoulder_pan_pub.publish(self.selected_action[0])
		self.shoulder_lift_pub.publish(self.selected_action[1])
		self.elbow_pub.publish(self.selected_action[2])
		self.wrist_1_pub.publish(0)
		self.wrist_2_pub.publish(0)
		self.wrist_3_pub.publish(0)	
		print("Action Executed")
		print( self.selected_action)
	def save_model(self):
		rospy.loginfo("saving the trained model")
		self.actor.save('actor')
		self.critic.save('critic')
	
	def states_post_action(self):
		data = rospy.wait_for_message("/RL_States/Nearest_Obstacles_States" , PointCloud2)
		self.stop_moving()
		np_data = ros_numpy.numpify(data)
		#self.obs_points_post_action = np.zeros((np_data.shape[0] , 3))
		#self.obs_points_post_action[ : , 0] = np_data['x']
		#self.obs_points_post_action[: , 1 ]  = np_data['y']
		#self.obs_points_post_action[: , 2] = np_data['z']
		self.dist_data = np_data['distances']
		self.min_dist = np.amin(self.dist_data)
		#self.obs_points_post_action = self.obs_points_post_action.reshape((self.obs_points_post_action.shape[0] * self.obs_points_post_action.shape[1]))
	
	def get_reward(self):
		
		least_distance = rospy.wait_for_message("Least_distance" , PointCloud2)
		least_distance = ros_numpy.numpify(least_distance)
		fore_arm = least_distance['distance'][0]
		upper_arm = least_distance['distance'][1]
		wrist_1 = least_distance['distance'][2]
		tool = least_distance['distance'][3]
		fore_arm_ok = False
		upper_arm_ok = False
		wrist_1_ok = False
		tool_ok = False
		if (fore_arm < 0.5 and fore_arm > 0.25):
			fore_arm_ok = True
		if (upper_arm < 0.5 and upper_arm > 0.25):
			upper_arm_ok = True
		if (wrist_1 < 0.5 and wrist_1 > 0.25):
			wrist_1_ok = True
		if(tool_ok < 0.5 and tool_ok > 0.25):
			tool_ok = True
		obstacle_ok = fore_arm_ok and upper_arm_ok and wrist_1_ok and tool_ok
		
		if (obstacle_ok == True ):
			obstacle = 1
		else:
			obstacle = 0.5
		self.l.waitForTransform("tool0" , "/box_link" , rospy.Time(0) , rospy.Duration(4.0))
		pointstamped = PointStamped()
		pointstamped.header.frame_id = "tool0"
		pointstamped.header.stamp = rospy.Time(0)
		pointstamped.point.x = 0.0
		pointstamped.point.y = 0.0
		pointstamped.point.z = 0.0
		p = self.l.transformPoint("box_link" , pointstamped)
		a = np.array([p.point.x , p.point.y , p.point.z])
		self.dist_to_goal  = np.linalg.norm(a)
		self.dist_to_goal_weighted = self.dist_to_goal * self.goal_weight
		#print(self.dist_to_goal)
		self.goal_nearing = self.dist_to_goal < self.old_dist_to_goal
		self.min_obs_dist_weighted = obstacle * self.obs_weight
		if( self.goal_nearing or self.min_obs_dist_weighted > 0.5):
			self.reward = 1
		else : 
			self.reward = 0
		rew  = "Reward is = " + str(self.reward)
		self.old_dist_to_goal = self.dist_to_goal
		print(rew)
		return self.reward
	
	def on_shutdown(self):
		self.stop_moving()
		
		
	def store_transition(self , state , action , probs ,vals, reward , done):
		self.memory.store_memory(state , action , probs , vals , reward , done)	
	
	def cb_robot_position(self, data):
		self.position =  data.position
	
	def choose_action(self, obs_state , self_state):
		a = []
		a.append(obs_state)
		a.append(self_state)
		probs = self.actor(a)
		dist  = tfp.distributions.Categorical(probs)
		action = dist.sample()
		log_prob = dist.log_prob(action)
		value = self.critic(a)
		action = action.numpy()[0]
		value = value.numpy()[0]
		log_prob = log_prob.numpy()[0]
		print("Got the Action and Value")
		return action , log_prob , value
    	def learn(self):
        	for _ in range(self.n_epochs):
            		obs_state_arr , self_state_arr, action_arr, old_prob_arr, vals_arr,\
                		reward_arr, dones_arr, batches = \
                		self.memory.generate_batches()
			values = vals_arr
            		advantage = np.zeros(len(reward_arr), dtype=np.float32)

            		for t in range(len(reward_arr)-1):
                		discount = 1
                		a_t = 0
                		for k in range(t, len(reward_arr)-1):
                    			a_t += discount*(reward_arr[k] + self.gamma*values[k+1] * (1-int(dones_arr[k])) - values[k])
                    			discount *= self.gamma*self.gae_lambda
                		advantage[t] = a_t
                	#print("Batches = ")
                	print(_)
			#print(batches)
            		for batch in range(10):
                		with tf.GradientTape(persistent=True) as tape:
                    			obs_states = tf.convert_to_tensor(obs_state_arr[batch].numpy())
                    			self_states = tf.convert_to_tensor(np.array(self_state_arr[batch]))
                    			a = []
                    			a.append(obs_states)
                    			a.append(self_states)
                    			old_probs = tf.convert_to_tensor(np.array(old_prob_arr[batch]))
                    			actions = tf.convert_to_tensor(np.array(action_arr[batch]))
                    			probs = self.actor(a)
                    			dist = tfp.distributions.Categorical(probs)
                    			new_probs = dist.log_prob(actions)
					print(actions)
                    			critic_value = self.critic(a)

                    			critic_value = tf.squeeze(critic_value, 1)

                    			prob_ratio = tf.math.exp(new_probs - old_probs)
                    			weighted_probs = advantage[batch] * prob_ratio
                    			clipped_probs = tf.clip_by_value(prob_ratio,
                                                     				1-self.policy_clip,
                                                     				1+self.policy_clip)
                    			weighted_clipped_probs = clipped_probs * advantage[batch]
                    			actor_loss = -tf.math.minimum(weighted_probs,
                                                  			weighted_clipped_probs)
                    			actor_loss = tf.math.reduce_mean(actor_loss)

                    			returns = advantage[batch] + values[batch]
                    			critic_loss = keras.losses.MSE(critic_value, returns)

                		actor_params = self.actor.trainable_variables
                		actor_grads = tape.gradient(actor_loss, actor_params)
                		critic_params = self.critic.trainable_variables
                		critic_grads = tape.gradient(critic_loss, critic_params)
                		self.actor.optimizer.apply_gradients(
                        		zip(actor_grads, actor_params))
                		self.critic.optimizer.apply_gradients(
                        		zip(critic_grads, critic_params))

        	self.memory.clear_memory()
        	
def start():
	plt.ion()
	rospy.init_node("RL_Agent")
	rl_obj = HARI_RL()
	rl_obj.stop_moving()
	rl_obj.get_states()
	rl_obj.cb_obs()
	score = 0
	n_steps = 0
	best_score = -1
	game_index = 0
	score_history = []
	index = []
	while game_index  < 20:
		
		score  = []
		game_index = game_index + 1
		index.append(game_index)
		rospy.loginfo("About to start in 20 Senconds")
		#rospy.sleep(10)
		done = False
		i = 0
		while not done:
			if i == 10 : 
				done = True
			i = i+1
			rl_obj.get_states()
			rl_obj.cb_obs()
			action , prob , val = rl_obj.choose_action(rl_obj.points , rl_obj.robot_position)
			rl_obj.take_action(action)
			#rospy.sleep(2)
			#rl_obj.states_post_action()
			rl_obj.get_reward()
			score.append(rl_obj.reward)
			rl_obj.memory.store_memory(rl_obj.points , rl_obj.robot_position , action  , prob, val , rl_obj.reward , done)
			n_steps  = n_steps + 1
			#score  = score + rl_obj.reward
			#rl_obj.learn()
			#rl_obj.save_models()
		score_history.append(np.mean(score))
		rl_obj.stop_moving()
		rl_obj.learn()
		#save_trial = rl_obj.actor(rl_obj.points , rl_obj.robot_position)
		a = []
		a.append(rl_obj.points)
		a.append(rl_obj.robot_position)
		if ( game_index == 1):
			rl_obj.actor._set_inputs(a)
		rl_obj.actor.save_weights('./actor_checkpoints/my_checkpoint')
		#rl_obj.actor.save('actor' , save_format ='tf')
		save_trial_critic = rl_obj.critic(a)
		if ( game_index == 1):
			rl_obj.critic._set_inputs(a)
		rl_obj.critic.save_weights('./critic_checkpoints/my_checkpoint')
		#rl_obj.critic.save('critic')
		#print(rl_obj.actor(rl_obj.memory.obs_states[4] , rl_obj.memory.self_states[4]))
		rospy.loginfo("Training Completed, Please Reset the scene, You GOT 2 Minutes")
		reset_joints = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
		reset_req = SetModelConfigurationRequest()
		reset_req.model_name= 'robot'
		reset_req.urdf_param_name = 'robot_description'
		reset_req.joint_names=['shoulder_pan_joint', 'shoulder_lift_joint' , 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint' , 'wrist_3_joint']
		reset_req.joint_positions =[0.0 , 0.0 , 0.0 ,0.0 ,0.0 ,0.0 ]
		res = reset_joints(reset_req)
		
		#rospy.wait_for_service('/gazebo/reset_world')
		#reset_simulation = rospy.ServiceProxy('/gazebo/reset_world' ,Empty)
		#reset_simulation()
		#rospy.sleep(30)
		plt.plot(index , score_history)
		plt.draw()
		plt.pause(0.01)
		plt.clf()
if __name__=="__main__":
	start()
