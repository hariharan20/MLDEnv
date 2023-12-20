/* CODE FOR SAVING PCD FILES OF SCENE CAPTURED from KINECT*/

#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>
void data_store(const sensor_msgs::PointCloud2&input)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(input ,  cloud);
    pcl::io::savePCDFileASCII("UR10_1.pcd"  , cloud);
    ROS_INFO("Saved data, chill you have 10 seconds to stop the program");
}
main(int argc, char **argv)
{
    ros::init(argc , argv , "table_pcd");
    ROS_INFO("Started PCL write node");
    ros::NodeHandle nh;
    ros::Rate sleeper(0.02);
    ros::Subscriber table_sub = nh.subscribe("/camera/depth/points", 10 , data_store);
    ros::spin();
    sleeper.sleep();    
    
    return 0;
}