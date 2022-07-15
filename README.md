# Learning_replanning
This is the repo for the online replanning of collaborative-robotic arms in dynamic environments

Virtual Environment 1:
-------------------------------------
To run the simulator:
roslaunch multi_robot main.launch

To run the point cloud, run:
rosrun pointnet batch_inference.py

To detect the adjacent obstacle robot:
rosrun programs robot_as_obstacle.py

To get distance from each robot end-effector to its corresponding goal:
rosrun programs dist_to.py


Virtual Environment 2:
--------------------------
1. Ensure Gym and tensorflow are installed
2. To run RL algorithm for one robot:
python trainer.py 
3. To run RL algo for the other robot (in another terminal)
python trainer2.py
