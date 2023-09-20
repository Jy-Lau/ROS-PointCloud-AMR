#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <filesystem>
#include <iostream>

int main(int argc, char **argv){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string package_path = ros::package::getPath("point_cloud_processing");
    std::string path = package_path+"/point_clouds/plane.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ>(path,*cloud);
    pcl::visualization::PCLVisualizer viewer("Point Cloud with Path");
    viewer.addPointCloud(cloud, "Ground Plane ");
    
    float ball_radius = 0.2;

    float min_x, min_y, max_x, max_y = 0;
    for(int i=0;i<cloud->points.size();++i){
        pcl::PointXYZ point = cloud->points[i];
        if(point.x<min_x)min_x=point.x;
        if(point.y<min_y)min_y=point.y;
        if(point.x>max_x)max_x=point.x;
        if(point.y>max_y)max_y=point.y;
    }
    float ball_x = min_x+1.2;
    float ball_y = min_y;
    int i=0;
    while(ball_y<=max_y){
        pcl::PointXYZ point;
        point.x=ball_x;
        point.y=ball_y;
        viewer.addSphere(point,0.1,0.0,1.0,0.0,"Sphere"+std::to_string(i++));
        ball_y+=ball_radius*1.5;
    }
    
    viewer.spin();
    return 0;
}