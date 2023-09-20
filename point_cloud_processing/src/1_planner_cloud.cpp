#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/filters/voxel_grid.h>

int main(int argc, char **argv){
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::string package_path = ros::package::getPath("point_cloud_processing");
    std::string path = package_path+"/point_clouds/plane.pcd";
    cloud.push_back(pcl::PointXYZ(1.0,2.0,3.0));
    cloud.push_back(pcl::PointXYZ(4.0,5.0,6.0));
    cloud.push_back(pcl::PointXYZ(7.0,8.0,9.0));
    cloud.push_back(pcl::PointXYZ(10.0,11.0,12.0));
    cloud.push_back(pcl::PointXYZ(13.0,14.0,15.0));
    pcl::io::savePCDFileASCII(path,cloud);
    return 0;
}