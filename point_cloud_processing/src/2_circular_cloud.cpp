#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char **argv){
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    double radius =3.0;
    int num_points =50;
    double angular_step = 2.0*M_PI/num_points;
    
    for(int i=0;i<num_points;++i){
        pcl::PointXYZRGB point;
        double angle=i*angular_step;
        point.x=radius*std::cos(angle);
        point.y=radius*std::sin(angle);
        point.z=1.0;

        point.r=255*std::cos(angle);
        point.g=255*std::sin(angle);
        point.b=255*std::cos(angle * M_PI_2);
        cloud.push_back(point);
    }
    std::string package_path = ros::package::getPath("point_cloud_processing");
    std::string path = package_path+"/point_clouds/circular.pcd";
    pcl::io::savePCDFileASCII(path,cloud);
    return 0;
}