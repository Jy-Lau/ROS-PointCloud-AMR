#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/filters/voxel_grid.h>

int main(int argc, char **argv){
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl::PCDReader cloud_reader;
    std::string package_path = ros::package::getPath("point_cloud_processing");
    std::string path = package_path+"/point_clouds/tb3_world.pcd";
    cloud_reader.read(path,*cloud);
	std::cout<<"Number of points "<<cloud->width * cloud->height<<std::endl;
    return 0;
}