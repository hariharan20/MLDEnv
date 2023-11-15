# The Repo Contains scripts for training Online Replanning of Collaborative robotic arms in dynamic environment
Steps to work with this repository
```
mkdir -p ros_ws/src
cd ros_ws/src
git clone https://github.com/sariahmghames/Learning_replanning.git
```
The Repo Contains three parts:
1) Robot and Simulation
2) PointNet inference
3) Reinforcement Learning

## [1] Robot and Simulation
The  multi_robot package and the universal_robot repository contains the required simulation stack for two identical manipulators

Before launching the simulation, there are few additional gazebo models used in the project, which are inside the gazebo_models folder, place them inside """"

To start with the simulation :
```
roslaunch multi_robot main.launch
```
This launches the simulation and all required controllers.

## [2] PointNet inference
The Project uses pointnet for semantic segmentation of objects in the simulation. 
refer: https://github.com/charlesq34/pointnet for information on training the model with specific dataset.
Once the training is done, place the model checkpoint inside `pointnet/src/pointnet/log`

Before launching the inference script, launch the sampler script, 
```
rosrun programs pcldata.py
```
Also change the respective model location inside ` batch_inference.py ` inside `pointnet/src/pointnet/sem_seg/ `

The `batch_inference.py` is an implementation of pointnet inference with ROS.
```
rosrun pointnet batch_inference.py
```
## [3] Reinforcement Learning 
The Project has an implementation of PPO (Proximal Policy Optimization) algorithm used for manipulators.
From here, there will be executables for two robots separately launched.

The first step will be to get the states for the RL Agent.
```
rosrun programs state_pro.py
rosrun programs state_pro2.py
```
Once the states are ready for generation, launch the training module implemented with OpenAI gym interface. 
```
cd programs/src/RL_scripts
python trainer.py
python trainer2.py
```
