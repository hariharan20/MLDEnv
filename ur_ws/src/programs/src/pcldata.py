#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import pcl
from sklearn.cluster import DBSCAN
import ros_numpy
import numpy as np


def callback(data):
    pc = ros_numpy.numpify(data)
    points = np.zeros((pc.shape[0], 3))
    points[:,0] = pc['x']
    points[:,1] = pc['y']
    points[:,2] = pc['z']
    
    p = pcl.PointCloud(np.array(points, dtype = np.float32))
    print(p)
    c = DBSCAN(eps = 0.11, min_samples= 4).fit(points)
    
    xp = PointCloud2()
    
    
    for i in range(len(c.labels_) ):
        if(True):
            datae = points[i].tostring()
            xp.data.__add__(datae)

    pub = rospy.Publisher("filteredddd", PointCloud2, queue_size=10)
    xp.header.frame_id = "laka_laka"
    pub.publish(xp)

    #clustering =  #pub = rospy.Publisher("clustered", PointCloud2, queue_size= 1)


if __name__=="__main__":
    rospy.init_node("clusterer", anonymous=True)
    sub = rospy.Subscriber("pcl_filtere", PointCloud2, callback)
    
    rospy.spin()
