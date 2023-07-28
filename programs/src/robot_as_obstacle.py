#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
import ros_numpy
import numpy 
import tf2_msgs.msg 
import numpy as np
rospy.init_node("robot_as_obstacle")
l = tf.TransformListener()
p = PointStamped()
x = []
y = []
z = [] 
b = PointStamped()
df = 100 #(Diving Factor) Default 4 for getting higher resolution of obstacle increase this factor
pub = rospy.Publisher("robot_as_obstacle/robot1" , PointCloud2 , queue_size=1)
pub_1 = rospy.Publisher("robot_as_obstacle/robot2" , PointCloud2 , queue_size=1)



# dis_calc makes sphere around each joint of the robot and returns points with minimum distance from each joint
def dis_calc( l  , listp  , robot): #robot keyword denotes FRAME WRT
	joint_location = []
	if robot == "Robot1":
		joint_transform = PointStamped()
		joint_transform.header.frame_id = "forearm_link"
		temp_point = l.transformPoint("base_link" , joint_transform)
		joint_location.append([temp_point.point.x , temp_point.point.y , temp_point.point.z])
		joint_transform.header.frame_id = "upper_arm_link"
		temp_point = l.transformPoint("base_link" , joint_transform)
		joint_location.append([temp_point.point.x , temp_point.point.y , temp_point.point.z])
		joint_transform.header.frame_id = "wrist_1_link"
		temp_point = l.transformPoint("base_link" , joint_transform)
		joint_location.append([temp_point.point.x , temp_point.point.y , temp_point.point.z])
		joint_transform.header.frame_id = "tool0"
		temp_point = l.transformPoint("base_link" , joint_transform)
		joint_location.append([temp_point.point.x , temp_point.point.y , temp_point.point.z])
	
	
	elif robot == "Robot2":
		joint_transform = PointStamped()
		joint_transform.header.frame_id = "Aforearm_link"
		temp_point = l.transformPoint("Abase_link" , joint_transform)
		joint_location.append([temp_point.point.x , temp_point.point.y , temp_point.point.z])
		joint_transform.header.frame_id = "Aupper_arm_link"
		temp_point = l.transformPoint("Abase_link" , joint_transform)
		joint_location.append([temp_point.point.x , temp_point.point.y , temp_point.point.z])
		joint_transform.header.frame_id = "Awrist_1_link"
		temp_point = l.transformPoint("Abase_link" , joint_transform)
		joint_location.append([temp_point.point.x , temp_point.point.y , temp_point.point.z])
		joint_transform.header.frame_id = "Atool0"
		temp_point = l.transformPoint("Abase_link" , joint_transform)
		joint_location.append([temp_point.point.x , temp_point.point.y , temp_point.point.z])
	
	
	
	nearest_points_wrt_robots = np.zeros((len(joint_location),   4))
	for j in range(len(joint_location)):
		dist = []
		for i in range(listp.shape[0]):
			dist.append(np.linalg.norm(listp[i] - joint_location[j]))
		dist = np.array(dist)
		point = listp[np.where(dist==np.min(dist))]
		point = point[0]
		nearest_points_wrt_robots[j , 0] = point[ 0]
		nearest_points_wrt_robots[j , 1] = point[ 1]
		nearest_points_wrt_robots[j , 2] = point[ 2]
		nearest_points_wrt_robots[j , 3] = np.min(dist)
	return nearest_points_wrt_robots

while not rospy.is_shutdown():
	x = []
	y = []
	z = []
	distances = []
	joint_positions = []
	l.waitForTransform("/base_link" , "Abase_link" , rospy.Time.now() , rospy.Duration(4.0))
	p.header.frame_id = "/Aforearm_link"
	a = l.transformPoint("/Aupper_arm_link" , p)
	for _ in range(df-1):
		b.point.x = a.point.x*(_+1)/df
		b.point.y = a.point.y*(_+1)/df
		b.point.z = a.point.z*(_+1)/df
		b.header.frame_id = a.header.frame_id
		distance = np.linalg.norm([b.point.x , b.point.y , b.point.z])
		temp = l.transformPoint("/base_link" , b)
		x.append(temp.point.x)
		y.append(temp.point.y)
		z.append(temp.point.z)
		distances.append(distance)
	p.header.frame_id = "/Awrist_1_link"
	a = l.transformPoint("/Aforearm_link" , p)
	for _ in range(df-1):
		b.point.x = a.point.x*(_+1)/df
		b.point.y = a.point.y*(_+1)/df
		b.point.z = a.point.z*(_+1)/df
		b.header.frame_id = a.header.frame_id
		distance = np.linalg.norm([b.point.x , b.point.y , b.point.z])
		temp = l.transformPoint("/base_link" , b)
		x.append(temp.point.x)
		y.append(temp.point.y)
		z.append(temp.point.z)
		distances.append(distance)
	points = [x , y , z]
	points = np.array(points)
	points = np.transpose(points)
	points = dis_calc(l , points , "Robot1")
	points_ = np.zeros(len(points) , dtype = [('x' , np.float32) , ('y' , np.float32) , ('z' , np.float32) , ('distances' , np.float32)])
	points_['x'] = points[: , 0]
	points_['y'] = points[: , 1]
	points_['z'] = points[: , 2]
	points_['distances'] = points[: , 3]
	#print(points_)
	points_msg = ros_numpy.msgify(PointCloud2 , points_)
	points_msg.header.frame_id = "base_link"
	points_msg.header.stamp = rospy.Time(0)
	pub.publish(points_msg)
	#print(points.shape)	
	
	x = []
	y = []
	z = []
	distances = []
	#l.waitForTransform("/base_link" , "Abase_link" , rospy.Time.now() , rospy.Duration(4.0))
	p.header.frame_id = "/forearm_link"
	a = l.transformPoint("/upper_arm_link" , p)
	for _ in range(df -1):
		b.point.x = a.point.x*(_+1)/df
		b.point.y = a.point.y*(_+1)/df
		b.point.z = a.point.z*(_+1)/df
		b.header.frame_id = a.header.frame_id
		distance = np.linalg.norm([b.point.x , b.point.y , b.point.z])
		temp = l.transformPoint("/Abase_link" , b)
		x.append(temp.point.x)
		y.append(temp.point.y)
		z.append(temp.point.z)
		distances.append(distance)
	p.header.frame_id = "/wrist_1_link"
	a = l.transformPoint("/forearm_link" , p)
	for _ in range(df-1):
		b.point.x = a.point.x*(_+1)/df
		b.point.y = a.point.y*(_+1)/df
		b.point.z = a.point.z*(_+1)/df
		b.header.frame_id = a.header.frame_id
		distance = np.linalg.norm([b.point.x , b.point.y , b.point.z])
		temp = l.transformPoint("/Abase_link" , b)
		x.append(temp.point.x)
		y.append(temp.point.y)
		z.append(temp.point.z)
		distances.append(distance)
	points = [x , y , z]
	points = np.array(points)
	points = np.transpose(points)
	points = dis_calc(l  , points , "Robot2")
	points_ = np.zeros(len(points) , dtype = [('x' , np.float32) , ('y' , np.float32) , ('z' , np.float32) , ('distances' , np.float32)])
	points_['x'] = points[: , 0]
	points_['y'] = points[: , 1]
	points_['z'] = points[: , 2]
	points_['distances'] = points[:  , 3]
	
	points_msg = ros_numpy.msgify(PointCloud2 , points_)
	points_msg.header.frame_id = "Abase_link"
	points_msg.header.stamp = rospy.Time(0)
	pub_1.publish(points_msg)
	#print(points.shape)
