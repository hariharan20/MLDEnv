#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, JointState
from sklearn.cluster import DBSCAN
import ros_numpy
import numpy as np
from sklearn.cluster import DBSCAN
import tf
from geometry_msgs.msg import PointStamped, Point
#import moveit_commander as mc
from sensor_msgs.msg import JointState
import tf2_msgs.msg




def dis_calc(listp , centre):
	centre = np.array(centre)
	dist = []
	for i in range(listp.shape[0]):
		dist.append(np.linalg.norm(centre - listp[i]))
	dist = np.array(dist)
        return np.min(dist) , listp[np.where(dist==np.min(dist))]

def callback(data):
    np_data = ros_numpy.numpify(data)
    #Intensity of PointCloud and Classes are related for viewing by a factor of 50
    #Values of I :
    # 50  = Tables
    # 100 = UR10
    # 150 = Obstacles
    obs_points = np_data[np.where(np_data['I']==200)]
    #rospy.loginfo(obs_points.shape[0])
    points = np.zeros((obs_points.shape[0], 3))
    #rospy.loginfo("data_recieved") 
    points[:,0] = obs_points['x']
    points[:,1] = obs_points['y']
    points[:,2] = obs_points['z']
    #rospy.loginfo(points.shape)
    clustering = DBSCAN(eps = 0.2).fit(points)
    obs_points['I'] = clustering.labels_
    #rospy.loginfo(obs_points['I'][5])
    cluster_pub = rospy.Publisher("Clustered_Obstacles" , PointCloud2 , queue_size  = 10)
    cluster_pub_msg  = ros_numpy.msgify(PointCloud2 , obs_points)
    cluster_pub_msg.header.frame_id = "camera_link"
    cluster_pub.publish(cluster_pub_msg)
    #_, cluster_counts = np.unique(clustering.labels_)
    #rospy.loginfo(np.unique(clustering.labels_))
    obs_points_pc2 = PointCloud2()
    obs_points_pc2 = ros_numpy.msgify(PointCloud2 , obs_points)
    obs_points_pc2.header.frame_id = 'camera_link'
    NUMBER_CLUSTERS = len(np.unique(clustering.labels_))
    #rospy.loginfo(NUMBER_CLUSTERS)
    centres = np.zeros((NUMBER_CLUSTERS , 3)) #centres in camera frame
    centres_in_base_frame = np.zeros((NUMBER_CLUSTERS , 3))
    centres_in_wrist_frame = np.zeros((NUMBER_CLUSTERS , 3))
    centres_in_forearm_frame = np.zeros((NUMBER_CLUSTERS , 3))
    centres_in_upperarm_frame = np.zeros((NUMBER_CLUSTERS , 3))
    #centres_in_base_frame = np.zeros((NUMBER_CLUSTERS , 3))
    #centres_in_base_frame = np.zeros((NUMBER_CLUSTERS , 3))
    distances_of_obs = []
    NUMBER_POINTS = NUMBER_CLUSTERS *4
    points_near_to_joints  = np.zeros((NUMBER_CLUSTERS , 4 , 4))
    points_x = []
    points_y =[]
    points_z = []
    intensity = []
    rospy.loginfo("Data processing Initialized")
    #timer = rospy.Time.now() + rospy.Duration(4)
    #rospy.sleep(0.1)
    success = False		
    #timer = rospy.Time.now()
    #listener = listee()
    #tf_data = rospy.wait_for_message("/tf", tf2_msgs.msg.TFMessage)
    timer = rospy.Time.now()
    print(timer)
    global listener
    listener.waitForTransform("/camera_link" , "/tool0", timer , rospy.Duration(4.0))
    	
    least_distance = np.zeros(4 , dtype = [('distance' , np.float64)])
    for i in range(NUMBER_CLUSTERS):
        #rospy.loginfo(i) 
        same_class_points_np = obs_points[np.where(obs_points['I'] == i)]
	same_class_points = np.zeros((same_class_points_np.shape[0] , 3))
	same_class_points[:,0] = same_class_points_np['x']
	same_class_points[:,1] = same_class_points_np['y']
	same_class_points[:,2] = same_class_points_np['z']
	laser_point = PointStamped()
	laser_point.point.x = 0
	laser_point.point.y = 0
	laser_point.point.z = 0
	#rospy.loginfo(same_class_points)
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
	points_near_to_joints[i , 3 , 0] = _
	if(i == 0):
		least_distance['distance'][0] = _
	else :
		if(least_distance['distance'][0]  > _ ):
			least_distance['distance'][0] = _
	points_x.append(p.point.x)
	points_y.append(p.point.y)
	points_z.append(p.point.z)
	intensity.append((10 * i) + 1)  
	#rospy.loginfo(points_near_to_joints[i , : , 0])	
	
	laser_point.header.frame_id = "/upper_arm_link"
	#listener.waitForTransform("/camera_link" , "/wrist_2_link", rospy.Time.now() , rospy.Duration( 4.0))
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
	points_near_to_joints[i , 3 , 1] = _
	if(i == 0):
		least_distance['distance'][1] = _
	else :
		if(least_distance['distance'][1]  > _ ):
			least_distance['distance'][1] = _
	points_x.append(p.point.x)
	points_y.append(p.point.y)
	points_z.append(p.point.z)
	intensity.append((10*i) + 2)
	#rospy.loginfo(points_near_to_joints[i , : , 1])	
	laser_point.header.frame_id = "/wrist_1_link"	
	#listener.waitForTransform("/camera_link" , "/wrist_1_link", rospy.Time.now() , rospy.Duration( 4.0))
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
	points_near_to_joints[i, 3 ,2] = _
	if(i == 0):
		least_distance['distance'][2] = _
	else :
		if(least_distance['distance'][2]  > _ ):
			least_distance['distance'][2] = _
	points_x.append(p.point.x)
	points_y.append(p.point.y)
	points_z.append(p.point.z)
	intensity.append((10 * i ) + 3)
	#rospy.loginfo(points_near_to_joints[i , : , 2])			

	laser_point.header.frame_id = "/tool0"
	#listener.waitForTransform("/camera_link" , "/tool0", rospy.Time.now() , rospy.Duration( 4.0))
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
	points_near_to_joints[i ,3 , 3] = _
	if(i == 0):
		least_distance['distance'][3] = _
	else :
		if(least_distance['distance'][3]  > _ ):
			least_distance['distance'][3] = _
	points_x.append(p.point.x)
	points_y.append(p.point.y)
	points_z.append(p.point.z)
	intensity.append((10 * i ) + 4)
	#rospy.loginfo(points_near_to_joints[i , : , 3])
    nearest_obs  = np.zeros((NUMBER_POINTS , 5))
    rospy.loginfo(least_distance)
    nearest_obs  = np.zeros(NUMBER_POINTS , dtype=[('x' , np.float32), ('y', np.float32), ('z' , np.float32), ('intensity', np.float32) , ('distances' , np.float32)])
    nearest_obs['x'] = points_near_to_joints[: , 0  ,:].reshape(NUMBER_POINTS)
    nearest_obs['y'] = points_near_to_joints[: , 1 , :].reshape(NUMBER_POINTS)
    nearest_obs['z'] = points_near_to_joints[: , 2 , :].reshape(NUMBER_POINTS)
    nearest_obs['intensity'] = np.array(intensity)
    nearest_obs['distances'] = points_near_to_joints[: , 3 , :].reshape(NUMBER_POINTS)
    least_distance_msg = ros_numpy.msgify(PointCloud2 , least_distance)
    least_distance_msg.header.frame_id = "world"
    least_distance_pub = rospy.Publisher("/Least_distance" , PointCloud2 , queue_size = 10)
    least_distance_pub.publish(least_distance_msg)
    nearest_obs_msg = PointCloud2()
    #rospy.loginfo(nearest_obs['x'].shape)
    nearest_obs_msg = ros_numpy.msgify(PointCloud2, nearest_obs)
    nearest_obs_msg.header.frame_id = 'world'
    pub_nearest_obs = rospy.Publisher("/RL_States/Nearest_Obstacles_States" , PointCloud2 , queue_size=10)
    pub_nearest_obs.publish(nearest_obs_msg)
    #group_name = "manipulator"
    #group = mc.MoveGroupCommander(group_name)
    #ur10_joint_states = JointState()
    #ur10_joint_states.position = group.get_current_joint_values()
    #pub_ur10 = rospy.Publisher("/RL_States/Robot_Joint_States" , JointState, queue_size=10)
    #pub_ur10.publish(ur10_joint_states) 



def start():
    rospy.init_node("clusterer")
    rospy.loginfo("Node Started")
    global listener 
    listener = tf.TransformListener()
    sub = rospy.Subscriber("/predicted_topic", PointCloud2, callback)
    rospy.loginfo("Subscriber_started")
    
if __name__ == "__main__":
	
	start()
	rospy.spin()
