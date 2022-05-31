#! /usr/bin/env python
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import tensorflow as tf
from tensorflow.keras.layers import RNN , LSTMCell
from stable_baselines.common.policies import ActorCriticPolicy
from stable_baselines.common.vec_env import DummyVecEnv
import numpy as np
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
class cp(ActorCriticPolicy):
	def __init__(self, sess , ob_space , ac_space , n_env , n_steps , n_batch , reuse=True , **kwargs):
		super(cp , self).__init__(sess, ob_space, ac_space, n_env, n_steps, n_batch, reuse=reuse, scale=False)
		with tf.variable_scope("model" , reuse = reuse):
			activ = tf.nn.relu
			extracted_features = self.custom_lstm(self.processed_obs , **kwargs)
			extracted_features = tf.layers.flatten(extracted_features)
			
			pi_h = extracted_features
			for i, layer_size in enumerate([128 , 128 , 128]):
				pi_h = activ(tf.layers.dense(pi_h , layer_size , name='pi_fc' + str(i)))
			pi_latent = pi_h
			vf_h = extracted_features
			for j , layer_size in enumerate([32 , 32]):
				vf_h = activ(tf.layers.dense(vf_h , layer_size , name = 'vf_fc' + str(j)))
			value_fn = tf.layers.dense(vf_h , 1 , name ="vf")
			vf_latent = vf_h
			
			
			self._proba_distribution , self._policy , self.q_value = self.pdtype.proba_distribution_from_latent(pi_latent , vf_latent , init_scale= 0.01)
		self._value_fn = value_fn
		self._setup_init()		
		
	def step(self, obs, state=None, mask=None, deterministic=False):
		if deterministic:
			action, value, neglogp = self.sess.run([self.deterministic_action, self.value_flat, self.neglogp],{self.obs_ph: obs})
		else:
			action, value, neglogp = self.sess.run([self.action, self.value_flat, self.neglogp],{self.obs_ph: obs})
		print("stepped")
		return action, value, self.initial_state, neglogp

	def proba_step(self, obs , state=None , mask = None):
		
		return self.sess.run(self.policy_proba, {self.obs_ph: obs})

	def value(self, obs , state=None , mask = None):
		
		return self.sess.run(self.value_flat , {self.obs_ph:obs})

	def custom_lstm(self , state_vec , **kwargs):
		#print(state_vec)
		state_vec = tf.expand_dims(state_vec , 0)
		activ = tf.nn.relu
		state_vec = tf. transpose(state_vec)
		state_vec = tf.squeeze(state_vec)
		robot_states = tf.gather(state_vec , indices = list(range(0,6)))
		obs_states = tf.gather(state_vec , indices = list(range(6 , 90)))
		#state_vec = tf.reshape(state_vec , (-1 , 90))
		#robot_states = state_vec[0:6 , :]
		#robot_states = tf.slice(state_vec , begin=[0] , size=[6])   Trial 1
		#obs_states = state_vec[6:90 , :]
		#obs_states = tf.slice(state_vec , begin=[6] , size=[84])  Trial 1
		obs_states = tf.reshape(obs_states , (3 , -1))
		rnn_layer = RNN(LSTMCell(64))
		obs_states = tf.expand_dims(obs_states , 0)
		rnn_output = rnn_layer(obs_states)
		robot_states = tf.expand_dims(robot_states , 0)
		return tf.concat([robot_states , rnn_output] , 1)
