#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from sklearn.cluster import DBSCAN
import ros_numpy
import numpy as np
from sklearn.cluster import DBSCAN
import tf
from geometry_msgs.msg import PointStamped, Point
def dis_calc(centre , listp):
    centre_array = np.full(listp.shape , centre)
    return np.amax(np.linalg.norm(centre_array - listp))




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
    distances_from_robot = np.zeros((NUMBER_CLUSTERS, 4))
    for i in range(NUMBER_CLUSTERS):
        
        same_class_points_np = obs_points[np.where(obs_points['I'] == i)]
        same_class_points = np.zeros((same_class_points_np.shape[0] , 3))
        same_class_points[:,0] = same_class_points_np['x']
        same_class_points[:,1] = same_class_points_np['y']
        same_class_points[:,2] = same_class_points_np['z']
        centres[i, 0] = np.average(same_class_points[:,0])
        centres[i, 1] = np.average(same_class_points[:,1])
        centres[i, 2] = np.average(same_class_points[:,2])
        #rospy.loginfo(centres[i])
        distances_of_obs.append(dis_calc(centres[i] , same_class_points))


        listener = tf.TransformListener()
        listener.waitForTransform("/camera_link", "/world", rospy.Time(0),rospy.Duration(4.0))
        laser_point=PointStamped()
        laser_point.header.frame_id = "camera_link"
        laser_point.header.stamp =rospy.Time(0)
        laser_point.point.x=centres[i, 0]
        laser_point.point.y=centres[i, 1]
        laser_point.point.z=centres[i, 2]
        p=listener.transformPoint("/world",laser_point)
        centres_in_base_frame[i , 0] = p.point.x
        centres_in_base_frame[i , 1] = p.point.y
        centres_in_base_frame[i , 2] = p.point.z
        distances_from_robot[i, 0] = np.linalg.norm(centres_in_base_frame[i])



        listener.waitForTransform("/camera_link", "/wrist_1_link", rospy.Time(0),rospy.Duration(4.0))
        laser_point=PointStamped()
        laser_point.header.frame_id = "camera_link"
        laser_point.header.stamp =rospy.Time(0)
        laser_point.point.x=centres[i, 0]
        laser_point.point.y=centres[i, 1]
        laser_point.point.z=centres[i, 2]
        p=listener.transformPoint("/wrist_1_link",laser_point)
        centres_in_wrist_frame[i , 0] = p.point.x
        centres_in_wrist_frame[i , 1] = p.point.y
        centres_in_wrist_frame[i , 2] = p.point.z
        rospy.loginfo(centres_in_wrist_frame[i])
        distances_from_robot[i, 1] = np.linalg.norm(centres_in_wrist_frame[i])




        listener.waitForTransform("/camera_link", "/forearm_link", rospy.Time(0),rospy.Duration(4.0))
        laser_point=PointStamped()
        laser_point.header.frame_id = "camera_link"
        laser_point.header.stamp =rospy.Time(0)
        laser_point.point.x=centres[i, 0]
        laser_point.point.y=centres[i, 1]
        laser_point.point.z=centres[i, 2]
        p=listener.transformPoint("/forearm_link",laser_point)
        centres_in_forearm_frame[i , 0] = p.point.x
        centres_in_forearm_frame[i , 1] = p.point.y
        centres_in_forearm_frame[i , 2] = p.point.z
        distances_from_robot[i, 2] = np.linalg.norm(centres_in_forearm_frame[i])




        listener.waitForTransform("/camera_link", "/upper_arm_link", rospy.Time(0),rospy.Duration(4.0))
        laser_point=PointStamped()
        laser_point.header.frame_id = "camera_link"
        laser_point.header.stamp =rospy.Time(0)
        laser_point.point.x=centres[i, 0]
        laser_point.point.y=centres[i, 1]
        laser_point.point.z=centres[i, 2]
        p=listener.transformPoint("/upper_arm_link",laser_point)
        centres_in_upperarm_frame[i , 0] = p.point.x
        centres_in_upperarm_frame[i , 1] = p.point.y
        centres_in_upperarm_frame[i , 2] = p.point.z
        distances_from_robot[i, 3] = np.linalg.norm(centres_in_upperarm_frame[i])


        rospy.loginfo(distances_from_robot[i])
        #rospy.loginfo(p)
        #rospy.loginfo(dis_calc(centres[i] , same_class_points))
        #rospy.loginfo(centres[i]) 
    dist = np.array(distances_of_obs)
    obs_centres = np.zeros((NUMBER_CLUSTERS , 4))
    obs_centres = np.zeros(NUMBER_CLUSTERS, dtype=[('x', np.float32),('y', np.float32),('z', np.float32), ('dist', np.float32)])
    obs_centres['x'] = centres_in_base_frame[:,0]
    obs_centres['y'] = centres_in_base_frame[:,1]
    obs_centres['z'] = centres_in_base_frame[:,2]
    obs_centres['dist'] = dist
    #print(sampled.shape)
    obs_centres_msg = PointCloud2()
    #pcpub.header.frame_id = "lala"
    obs_centres_msg = ros_numpy.msgify(PointCloud2 , obs_centres)
    obs_centres_msg.header.frame_id = 'world'

    pub_obs = rospy.Publisher("Centres", PointCloud2, queue_size=10)
    #xp.header.frame_id = "laka_laka"
    pub_obs.publish(obs_centres_msg)

    pub = rospy.Publisher("obstacles", PointCloud2, queue_size=10)
    #xp.header.frame_id = "laka_laka"
    pub.publish(obs_points_pc2)
    #rospy.loginfo("pointcloud published")



if __name__=="__main__":
    rospy.init_node("clusterer")
    sub = rospy.Subscriber("/predicted_topic", PointCloud2, callback)

    rospy.spin()
