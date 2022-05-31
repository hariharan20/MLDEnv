#! /usr/bin/env python
from gazebo_linker import ur
from ahn_policy import cp
from stable_baselines import PPO2

env = ur()
model = PPO2(cp, env ,  verbose=1 , tensorboard_log="./ppo_gazebo")

model.learn(total_timesteps=250000)
model.save("ppo2_ur_gym")


