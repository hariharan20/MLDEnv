#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from sklearn.cluster import DBSCAN
import ros_numpy
import numpy as np
from sklearn.cluster import DBSCAN

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
    rospy.loginfo(points.shape)
    clustering = DBSCAN(eps = 0.2).fit(points)
    obs_points['I'] = clustering.labels_
    #_, cluster_counts = np.unique(clustering.labels_)
    rospy.loginfo(np.unique(clustering.labels_))
    obs_points_pc2 = PointCloud2()
    obs_points_pc2 = ros_numpy.msgify(PointCloud2 , obs_points)
    obs_points_pc2.header.frame_id = 'lala'
    NUMBER_CLUSTERS = len(np.unique(clustering.labels_))
    centres = np.zeros((NUMBER_CLUSTERS , 3))
    distances_of_obs = []
    for i in range(NUMBER_CLUSTERS):
        
        same_class_points_np = obs_points[np.where(obs_points['I'] == i)]
        same_class_points = np.zeros((same_class_points_np.shape[0] , 3))
        same_class_points[:,0] = same_class_points_np['x']
        same_class_points[:,1] = same_class_points_np['y']
        same_class_points[:,2] = same_class_points_np['z']
        centres[i] = np.average(same_class_points)
        distances_of_obs.append(dis_calc(centres[i] , same_class_points))
        rospy.loginfo(dis_calc(centres[i] , same_class_points))
        #rospy.loginfo(centres[i]) 



    pub = rospy.Publisher("obstacles", PointCloud2, queue_size=10)
    #xp.header.frame_id = "laka_laka"
    pub.publish(obs_points_pc2)
    rospy.loginfo("pointcloud published")



if __name__=="__main__":
    rospy.init_node("clusterer")
    sub = rospy.Subscriber("/predicted_topic", PointCloud2, callback)

    rospy.spin()