#ifndef VOXEL_H
#define VOXEL_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class Voxel_Filter
{
public:
    Voxel_Filter(const ros::NodeHandle &);

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2 &);

    ros::NodeHandle nh_;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher marker_pub;
};
#endif 
