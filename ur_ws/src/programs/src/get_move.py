#!/usr/bin/env python
''' CODE FOR MOVING UR10 THROUGH A WAYPOINTS
'''
from scipy.interpolate import Rbf
import scipy
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32MultiArray, String
import numpy as np
from mpl_toolkits.mplot3d import axes3d
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose
'''''

xg = -0.315
yg = 0.792
zg = -0.005
goal = np.array([xg , yg  , zg])
goal = np.array([goal])
fig  = plt.figure(figsize=(10,6))
ax = axes3d.Axes3D(fig)
for l in range(len(goal)):
    xi = 0.792
    yi = 0.756
    zi = 0.09
    X = np.array([xi, goal[l][0]])
    Y = np.array([yi, goal[l][1]])
    Z = np.array([zi , goal[l][2]])
    ax.scatter3D(X, Y,Z, c='r')
    rbfi = scipy.interpolate.Rbf(X, Y, Z, function='cubic')
    X_new = np.linspace(xi, goal[l][0], 20)
    Y_new = np.linspace(yi, goal[l][1], 20)
    Z_new = rbfi(X_new, Y_new)    
    ax.plot(X_new, Y_new, Z_new)
    Xp = X_new.flatten()
    Yp = Y_new.flatten()
    Zp = Z_new.flatten()
    traj1 = np.zeros((len(Xp), 3))
    for i in range(len(Xp)):
        traj1[i] = np.array([Xp[i], Yp[i], Zp[i]])
    t1_r, t1_c = traj1.shape


print traj1
ax.set_xlabel('X[mm]', fontsize = 15)
ax.set_ylabel('Y[mm]', fontsize = 15)
ax.set_zlabel('Z[mm]', fontsize = 15)
ax.xaxis._axinfo['label']['space_factor'] = 5.0
ax.yaxis._axinfo['label']['space_factor'] = 5.0
ax.zaxis._axinfo['label']['space_factor'] = 5.0
    # ax.set_title("End-Effector random demo based cubic spline trajectory generation for Strawberry Picking")
plt.show()


'''''



def all_close(goal, actual, tolerance):

  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class ProUoL(object):
  def __init__(self):
    super(ProUoL, self).__init__()

   
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    planning_frame = group.get_planning_frame() 
    eef_link = group.get_end_effector_link()
    group_names = robot.get_group_names()

   
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def execute_plan(self, plan):
    
    group = self.group

    group.execute(plan, wait=True)


  def plan_cartesian_path(self, scale=1):

    group = self.group



    xg = -0.315
    yg = 0.792
    zg = -0.005
    goal = np.array([xg , yg  , zg])
    goal = np.array([goal])
    fig  = plt.figure(figsize=(10,6))
    ax = axes3d.Axes3D(fig)
    for l in range(len(goal)):
        xi = 0.792
        yi = 0.756
        zi = 0.09
        X = np.array([xi, goal[l][0]])
        Y = np.array([yi, goal[l][1]])
        Z = np.array([zi , goal[l][2]])
        ax.scatter3D(X, Y,Z, c='r')
        rbfi = scipy.interpolate.Rbf(X, Y, Z, function='cubic')
        X_new = np.linspace(xi, goal[l][0], 20)
        Y_new = np.linspace(yi, goal[l][1], 20)
        Z_new = rbfi(X_new, Y_new)    
        ax.plot(X_new, Y_new, Z_new)
        Xp = X_new.flatten()
        Yp = Y_new.flatten()
        Zp = Z_new.flatten()
        traj = Pose()
        trajpts = []
        for i in range(len(Xp)):
            traj.position.x = Xp[i]
            traj.position.y = Yp[i]
            traj.position.z = Zp[i]
            trajpts.append(copy.deepcopy(traj))
    ax.set_xlabel('X[mm]', fontsize = 15)
    ax.set_ylabel('Y[mm]', fontsize = 15)
    ax.set_zlabel('Z[mm]', fontsize = 15)
    ax.xaxis._axinfo['label']['space_factor'] = 5.0
    ax.yaxis._axinfo['label']['space_factor'] = 5.0
    ax.zaxis._axinfo['label']['space_factor'] = 5.0
    #plt.show()


   
    (plan, fraction) = group.compute_cartesian_path(
                                       trajpts,   
                                       0.01,       
                                       0.0)         
    return plan, fraction


  def go_to_pose_goal(self):

    group = self.group

    pose_goal = geometry_msgs.msg.Pose()
    
    pose_goal.position.x = 0.792
    pose_goal.position.y = 0.756
    pose_goal.position.z = 0.09
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()


    
    return True


def main():
    try:
        doer = ProUoL()
        doer.go_to_pose_goal()
        plan, fraction = doer.plan_cartesian_path()
        doer.execute_plan(plan)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()