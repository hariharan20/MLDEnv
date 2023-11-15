#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import pcl
from sklearn.cluster import DBSCAN
import ros_numpy
import numpy as np
from random import sample
import os
from geometry_msgs.msg import Point
SAMPLE_NUMBER = 18000
def callback(data):
	pc = ros_numpy.numpify(data)
	points = np.zeros((307200, 4))# 307200 is camera specific 640 * 480 and should be varied when using different camera
    #pc = sample(pc , 2000)
    #pcpub = PointCloud2()
	points[: , 0] = np.reshape( pc['x'], (307200  ))
	points[:  ,1] = np.reshape(pc['y'] , (307200  ))
	points[: , 2] = np.reshape(pc['z'], (307200 ) )
	points[: , 3] = np.reshape(pc['rgb'], (307200 ) )
    #pcpub = ros_numpy.msgify(PointCloud2, pc)
    #print(points.shape)
    #p = pcl.PointCloud(np.array(points, dtype = np.float32))
    #print(p)
    #rospy.loginfo(pc['rgb'].shape)
	points = points[~np.isnan(points).any(axis=1)]
	points = np.unique(points, axis =0)
	if(points.shape[0] != 0):
		array_pcd = sample(points , SAMPLE_NUMBER)
		array_pcd = np.array(array_pcd)
		xyz_min = np.amin(array_pcd, axis=0)[0:3]
		array_pcd[:,0:3] -= xyz_min
		pub_min = rospy.Publisher("MinPoint" , Point, queue_size=10)
		min_point = Point()
		min_point.x = xyz_min[0]
		min_point.y = xyz_min[1]
		min_point.z = xyz_min[2]
		pub_min.publish(min_point)
    	#trial to automate without npy file generation using ros
    	#trial to automate without npy file generation using ros
		array_pcd_msg = np.zeros((SAMPLE_NUMBER , 4))
		array_pcd_msg = np.zeros(SAMPLE_NUMBER , dtype=[('x' , np.float32) ,('y' , np.float32), ('z' , np.float32), ('rgb' ,np.float32)])
		array_pcd_msg['x'] = array_pcd[:, 0]
		array_pcd_msg['y'] = array_pcd[:, 1]
		array_pcd_msg['z'] = array_pcd[:, 2]
		msg_array_pcd = PointCloud2()
		msg_array_pcd = ros_numpy.msgify(PointCloud2 , array_pcd_msg)
		msg_array_pcd.header.frame_id  = "camera_link"   
		pub_array_pcd = rospy.Publisher("array_pcd" , PointCloud2 , queue_size=10)
		pub_array_pcd.publish(msg_array_pcd)
		sampled = np.zeros((SAMPLE_NUMBER , 4))
		sampled = np.zeros(SAMPLE_NUMBER, dtype=[('x', np.float32),('y', np.float32),('z', np.float32), ('rgb', np.float32)])
		sampled['x'] = array_pcd[ :,0] + min_point.x
		sampled['y'] = array_pcd[ :,1] + min_point.y
		sampled['z'] = array_pcd[:,2] + min_point.z
		sampled['rgb'] = np.ones(SAMPLE_NUMBER) * 0.1
    	#print(sampled.shape)
		sampled_msg = PointCloud2()
    	#pcpub.header.frame_id = "lala"
		sampled_msg = ros_numpy.msgify(PointCloud2 , sampled)
		sampled_msg.header.frame_id = 'camera_link'
		pub = rospy.Publisher("sampled", PointCloud2, queue_size=10)
    	#xp.header.frame_id = "laka_laka"
		pub.publish(sampled_msg)
		pub_min.publish(min_point)

    	#clustering =  #pub = rospy.Publisher("clustered", PointCloud2, queue_size= 1)


if __name__=="__main__":
    rospy.init_node("pcl_sampler", anonymous=True)
    sub = rospy.Subscriber("/camera/depth/points", PointCloud2, callback)
    
    rospy.spin()
