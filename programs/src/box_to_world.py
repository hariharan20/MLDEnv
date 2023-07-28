#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler as qe



if __name__ == '__main__':
    rospy.init_node('tf_of_box')
    #tfb = FixedTFBroadcaster()
    pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
	
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        box_transform = geometry_msgs.msg.TransformStamped()
        box_transform.header.frame_id = "world"
        box_transform.header.stamp = rospy.Time.now()
        box_transform.child_frame_id= "box_link"
        box_transform.transform.translation.x = -0.152
        box_transform.transform.translation.y = 0.82
        box_transform.transform.translation.z = 0.056
        box_rotation = qe(0,0,3.14)
        box_transform.transform.rotation.x = box_rotation[0]
        box_transform.transform.rotation.y = box_rotation[1]
        box_transform.transform.rotation.z = box_rotation[2]
        box_transform.transform.rotation.w = box_rotation[3]
        box_tfm = tf2_msgs.msg.TFMessage([box_transform])
        pub_tf.publish(box_tfm)
		#rospy.loginfo("TF Published")
