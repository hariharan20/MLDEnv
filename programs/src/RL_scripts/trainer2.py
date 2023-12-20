#! /usr/bin/env python
from gl2 import ur
from ahn_policy_good import cp
from stable_baselines import PPO2
from stable_baselines.common.vec_env import DummyVecEnv, SubprocVecEnv
env = ur()
'''
env = DummyVecEnv([lambda: env])
model = PPO2.load("rb2" , env=env , verbose=1, tensorboard_log="./rb")
model.learn(total_timesteps=70000 , tb_log_name="PPO_on_robot_2" , reset_num_timesteps=False)
model.save("rb2_1")
print("model Saved")
'''
model = PPO2(cp, env ,  verbose=1 , tensorboard_log="./rb")

model.learn(total_timesteps=100000 , tb_log_name="PPO_on_robot_2")
model.save("rb2_final")
print("PPO Model Saved")

