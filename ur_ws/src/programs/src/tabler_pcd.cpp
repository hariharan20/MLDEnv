/* LOADING A PCD FILE FOR DISPLAYING*/


#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>
#include<iostream>
using namespace std;
class SubAndPub{
public:
    SubAndPub(){
        pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output" , 1);
        sub = nh.subscribe("/camera/depth/points", 10 , &SubAndPub::callback , this);

    }
    void callback(const sensor_msgs::PointCloud2&input)
    {
        pcl::PointCloud<pcl::PointXYZ> input_c;
        pcl::fromROSMsg(input, input_c);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::io::loadPCDFile("table.pcd" , cloud);
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud , output);
        cout << input.data[1];



    }
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
};



main(int argc, char **argv)
{
    ros::init(argc , argv , "SubAndPub");
    ROS_INFO("Started PCL read node");
    SubAndPub PCD_R;
    ros::spin();


    return 0;
}