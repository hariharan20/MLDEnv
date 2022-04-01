#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, JointState
from sklearn.cluster import DBSCAN
import ros_numpy
import numpy as np
from sklearn.cluster import DBSCAN
import tf
from geometry_msgs.msg import PointStamped, Point
import moveit_commander as mc
from sensor_msgs.msg import JointState

def dis_calc(listp , centre=[0 ,0, 0]):
	centre = np.array(centre)
	dist = []
	for i in range(listp.shape[0]):
		dist.append(np.linalg.norm(centre - listp[i]))
	dist = np.array(dist)
    	return np.max(dist) , listp[np.where(dist==np.max(dist))]




def callback(data):
    np_data = ros_numpy.numpify(data)
    #Intensity of PointCloud and Classes are related for viewing by a factor of 50
    #Values of I :
    # 50  = Tables
    # 100 = UR10
    # 150 = Obstacles
    obs_points = np_data[np.where(np_data['I']==150)]
    points = np.zeros((obs_points.shape[0], 3))
    
    points[:,0] = obs_points['x']
    points[:,1] = obs_points['y']
    points[:,2] = obs_points['z']
    #rospy.loginfo(points.shape)
    clustering = DBSCAN(eps = 0.2).fit(points)
    obs_points['I'] = clustering.labels_ + 1
    #_, cluster_counts = np.unique(clustering.labels_)
    #rospy.loginfo(np.unique(clustering.labels_))
    obs_points_pc2 = PointCloud2()
    obs_points_pc2 = ros_numpy.msgify(PointCloud2 , obs_points)
    obs_points_pc2.header.frame_id = 'camera_link'
    NUMBER_CLUSTERS = len(np.unique(clustering.labels_))
    centres = np.zeros((NUMBER_CLUSTERS , 3)) #centres in camera frame
    centres_in_base_frame = np.zeros((NUMBER_CLUSTERS , 3))
    centres_in_wrist_frame = np.zeros((NUMBER_CLUSTERS , 3))
    centres_in_forearm_frame = np.zeros((NUMBER_CLUSTERS , 3))
    centres_in_upperarm_frame = np.zeros((NUMBER_CLUSTERS , 3))
    #centres_in_base_frame = np.zeros((NUMBER_CLUSTERS , 3))
    #centres_in_base_frame = np.zeros((NUMBER_CLUSTERS , 3))
    distances_of_obs = []
    points_near_to_joints  = np.zeros((NUMBER_CLUSTERS,3 , 4))


    for i in range(NUMBER_CLUSTERS):
        
        same_class_points_np = obs_points[np.where(obs_points['I'] == i)]
        same_class_points = np.zeros((same_class_points_np.shape[0] , 3))
        same_class_points[:,0] = same_class_points_np['x']
        same_class_points[:,1] = same_class_points_np['y']
        same_class_points[:,2] = same_class_points_np['z']
	listener = tf.TransformListener()
	listener.waitForTransform("/camera_link" , "/forearm_link", rospy.Time(0) , rospy.Duration( 4.0))
	laser_point = PointStamped()
	laser_point.point.x = 0
	laser_point.point.y = 0
	laser_point.point.z = 0

	laser_point.header.frame_id = "/forearm_link"
	p =  listener.transformPoint("/camera_link" , laser_point)
	joint_origin_in_camera_frame = [p.point.x , p.point.y , p.point.z]
	_ , point_with_min_dist = dis_calc(same_class_points, centre = joint_origin_in_camera_frame)
	min_dist_point = PointStamped()
	#rospy.loginfo(point_with_min_dist)
	min_dist_point.point.x = point_with_min_dist[0 , 0]
	min_dist_point.point.y = point_with_min_dist[0  , 1]
	min_dist_point.point.z = point_with_min_dist[0 , 2]
	min_dist_point.header.frame_id = "/camera_link"
	p = listener.transformPoint("world" ,min_dist_point)
	points_near_to_joints[i ,0 , 0] = p.point.x
	points_near_to_joints[i, 1, 0 ] = p.point.y
	points_near_to_joints[i ,2 , 0] = p.point.z 
	rospy.loginfo(points_near_to_joints[i , : , 0])	

	
	laser_point.header.frame_id = "/wrist_2_link"
	listener.waitForTransform("/camera_link" , "/wrist_2_link", rospy.Time(0) , rospy.Duration( 4.0))
	p =  listener.transformPoint("/camera_link" , laser_point)
	joint_origin_in_camera_frame = [p.point.x , p.point.y , p.point.z]
	_ , point_with_min_dist = dis_calc(same_class_points, centre = joint_origin_in_camera_frame)
	min_dist_point = PointStamped()
	min_dist_point.point.x = point_with_min_dist[0 , 0]
	min_dist_point.point.y = point_with_min_dist[0, 1]
	min_dist_point.point.z = point_with_min_dist[0 ,2]
	min_dist_point.header.frame_id = "/camera_link"
	p = listener.transformPoint("world" ,min_dist_point)
	points_near_to_joints[i ,0 , 1] = p.point.x
	points_near_to_joints[i, 1,  1] = p.point.y
	points_near_to_joints[i ,2 , 1] = p.point.z
	rospy.loginfo(points_near_to_joints[i , : , 1])	
	
	laser_point.header.frame_id = "/wrist_1_link"
	listener.waitForTransform("/camera_link" , "/wrist_1_link", rospy.Time(0) , rospy.Duration( 4.0))
	p =  listener.transformPoint("/camera_link" , laser_point)
	joint_origin_in_camera_frame = [p.point.x , p.point.y , p.point.z]
	_ , point_with_min_dist = dis_calc(same_class_points, centre = joint_origin_in_camera_frame )
	min_dist_point = PointStamped()
	min_dist_point.point.x = point_with_min_dist[0 ,0]
	min_dist_point.point.y = point_with_min_dist[0 , 1]
	min_dist_point.point.z = point_with_min_dist[0 ,2]
	min_dist_point.header.frame_id = "/camera_link"
	p = listener.transformPoint("world" ,min_dist_point)
	points_near_to_joints[i ,0 , 2] = p.point.x
	points_near_to_joints[i, 1,  2] = p.point.y
	points_near_to_joints[i ,2 , 2] = p.point.z
	rospy.loginfo(points_near_to_joints[i , : , 2])	

	laser_point.header.frame_id = "/tool0"
	listener.waitForTransform("/camera_link" , "/tool0", rospy.Time(0) , rospy.Duration( 4.0))
	p =  listener.transformPoint("/camera_link" , laser_point)
	joint_origin_in_camera_frame = [p.point.x , p.point.y , p.point.z]
	_ , point_with_min_dist = dis_calc(same_class_points, centre = joint_origin_in_camera_frame)
	min_dist_point = PointStamped()
	min_dist_point.point.x = point_with_min_dist[0, 0]
	min_dist_point.point.y = point_with_min_dist[0 ,1]
	min_dist_point.point.z = point_with_min_dist[0 ,2]
	min_dist_point.header.frame_id = "/camera_link"
	p = listener.transformPoint("world" ,min_dist_point)
	points_near_to_joints[i ,0 , 3] = p.point.x
	points_near_to_joints[i, 1,  3] = p.point.y
	points_near_to_joints[i ,2 , 3] = p.point.z
	rospy.loginfo(points_near_to_joints[i , : , 3])
	
    group_name = "manipulator"
    group = mc.MoveGroupCommander(group_name)
    ur10_joint_states = JointState()
    ur10_joint_states.position = group.get_current_joint_values()
    pub_ur10 = rospy.Publisher("/RL_States/Robot_Joint_States" , JointState, queue_size=10)
    pub_ur10.publish(ur10_joint_states) 



if __name__=="__main__":
    rospy.init_node("clusterer")
    sub = rospy.Subscriber("/predicted_topic", PointCloud2, callback)

    rospy.spin()
