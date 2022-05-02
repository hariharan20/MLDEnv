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
from memory import PPOMemory

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
		self.batch_size = 64
		self.memory  = PPOMemory(self.batch_size)
		self.n_epochs = 10
		
		
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
	def on_shutdown(self):
		self.stop_moving()
		
		
	def store_transition(self , state , action , probs ,vals, reward , done):
		self.memory.store_memory(state , action , probs , vals , reward , done)	
		
    	def learn(self):
        	for _ in range(self.n_epochs):
            		state_arr, action_arr, old_prob_arr, vals_arr,\
                		reward_arr, dones_arr, batches = \
                		self.memory.generate_batches()

            		values = vals_arr
            		advantage = np.zeros(len(reward_arr), dtype=np.float32)

            		for t in range(len(reward_arr)-1):
                		discount = 1
                		a_t = 0
                	for k in range(t, len(reward_arr)-1):
                    		a_t += discount*(reward_arr[k] + self.gamma*values[k+1] * (
                        		1-int(dones_arr[k])) - values[k])
                    		discount *= self.gamma*self.gae_lambda
                	advantage[t] = a_t

            		for batch in batches:
                		with tf.GradientTape(persistent=True) as tape:
                    			states = tf.convert_to_tensor(state_arr[batch])
                    			old_probs = tf.convert_to_tensor(old_prob_arr[batch])
                    			actions = tf.convert_to_tensor(action_arr[batch])

                    			probs = self.actor(states)
                    			dist = tfp.distributions.Categorical(probs)
                    			new_probs = dist.log_prob(actions)

                    			critic_value = self.critic(states)

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
                    # critic_loss = tf.math.reduce_mean(tf.math.pow(
                    #                                  returns-critic_value, 2))
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
	rospy.init_node("RL_Agent")
	object_with_no_name = HARI_RL()
	






if __name__=="__main__":
	start()
	
