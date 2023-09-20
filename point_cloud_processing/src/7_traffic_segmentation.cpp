#include "voxel_filter.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree.h>


typedef pcl::PointXYZ PointT;

Voxel_Filter::Voxel_Filter(const ros::NodeHandle &nh): nh_(nh)
{
    pub = nh_.advertise<sensor_msgs::PointCloud2>("road_cloud", 10);
    marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    sub = nh_.subscribe("kitti/point_cloud", 10, &Voxel_Filter::pointCloudCallback,this);
}
void Voxel_Filter::pointCloudCallback(const sensor_msgs::PointCloud2& msg)
{   
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cropped_cloud (new pcl::PointCloud<PointT>) ;
    pcl::fromROSMsg(msg, *pcl_cloud);

    pcl::PassThrough<PointT> passing_x;
    pcl::PassThrough<PointT> passing_y;
    int radius = 15;
    // Along X Axis
    passing_x.setInputCloud(pcl_cloud);
    passing_x.setFilterFieldName("x");
    passing_x.setFilterLimits(-radius,radius);
    passing_x.filter(*cropped_cloud);

    // Along Y Axis
    passing_y.setInputCloud(cropped_cloud);
    passing_y.setFilterFieldName("y");
    passing_y.setFilterLimits(-radius,radius);
    passing_y.filter(*cropped_cloud);

    pcl::PointCloud<PointT>::Ptr voxel_cloud (new pcl::PointCloud<PointT>) ;
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(cropped_cloud);
    voxel_filter.setLeafSize(0.1 , 0.1, 0.1);
    voxel_filter.filter(*voxel_cloud);

    //Road Segmentation
    pcl::NormalEstimation<PointT,pcl::Normal> normals_estimator;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr road_normals  (new pcl::PointCloud<pcl::Normal>);
    pcl::ExtractIndices<pcl::Normal> normal_extractor;


    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> road_segmentor;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr  coefficients (new pcl::ModelCoefficients);
    pcl::ExtractIndices<PointT> indices_extractor;
    pcl::PointCloud<PointT>::Ptr road_cloud  (new pcl::PointCloud<PointT>);

    //Normal Extraction
    normals_estimator.setSearchMethod(tree);
    normals_estimator.setInputCloud(voxel_cloud);
    normals_estimator.setKSearch(30);
    normals_estimator.compute(*road_normals);

    // Parameters for segmentation
    road_segmentor.setOptimizeCoefficients(true);
	road_segmentor.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	road_segmentor.setMethodType(pcl::SAC_RANSAC);
	road_segmentor.setNormalDistanceWeight(0.5);    //algorithm gives equal importance to cylinder geometric shape, and alignment of normal surface (perpendicular)
	road_segmentor.setMaxIterations(100); //increasing the number of iterations can improve accuracy but may also increase computation time
	road_segmentor.setDistanceThreshold(0.4);
    road_segmentor.setInputCloud(voxel_cloud);
    road_segmentor.setInputNormals(road_normals);
    road_segmentor.segment(*inliers,*coefficients);

    //Indices extractor from voxel cloud
    indices_extractor.setInputCloud(voxel_cloud);
    indices_extractor.setIndices(inliers);
    indices_extractor.setNegative(true); //will get what the segmentor has just segmented from the inliers (a plane) if set to false
    indices_extractor.filter(*road_cloud);

    //Remove normals
    // normal_extractor.setNegative (true);
    // normal_extractor.setInputCloud (road_cloud);
    // normal_extractor.setIndices (inliers);
    // normal_extractor.filter (*road_normals);

    //Traffic clustering
    pcl::PointCloud<PointT>::Ptr segmented_cluster (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr all_clusters (new pcl::PointCloud<PointT>);
    tree->setInputCloud (road_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    
    struct BBox
    {
      float x_min;
      float x_max;
      float y_min;
      float y_max;
      float z_min;
      float z_max;
      double r = 1.0;
      double g = 0.0;
      double b = 0.0;
    };

    //Eucludian based clustering
    ec.setClusterTolerance (0.25);
    ec.setMinClusterSize(600);
    ec.setMaxClusterSize(2000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(road_cloud);
    ec.extract(cluster_indices);
    std::vector<BBox> bboxes;

    size_t min_reasonable_size = 610;
    size_t max_reasonable_size = 1900;
    int num_reasonable_clusters = 0;
    for(size_t i=0;i<cluster_indices.size(); ++i){
        if (cluster_indices[i].indices.size() > min_reasonable_size && cluster_indices[i].indices.size() < max_reasonable_size)
        {
            pcl::PointCloud<PointT>::Ptr reasonable_cluster (new pcl::PointCloud<PointT>);
            pcl::ExtractIndices<PointT> extract;
            pcl::IndicesPtr indices(new std::vector<int>(cluster_indices[i].indices.begin(), cluster_indices[i].indices.end()));
            extract.setInputCloud (road_cloud);
            extract.setIndices(indices);
            extract.setNegative (false);
            extract.filter (*reasonable_cluster);
            all_clusters->operator+=(*reasonable_cluster);
            num_reasonable_clusters++;

            Eigen::Vector4f min_pt, max_pt;
            pcl::getMinMax3D<PointT>(*reasonable_cluster, min_pt, max_pt);

            pcl::PointXYZ center((min_pt[0] + max_pt[0]) / 2.0, (min_pt[1] + max_pt[1]) / 2.0, (min_pt[2] + max_pt[2]) / 2.0);
            BBox bbox;
            bbox.x_min = min_pt[0];
            bbox.y_min = min_pt[1];
            bbox.z_min = min_pt[2];
            bbox.x_max = max_pt[0];
            bbox.y_max = max_pt[1];
            bbox.z_max = max_pt[2];

            bboxes.push_back(bbox);
        }
    }
    //==================================== Drawing Boxes  ====================================

    visualization_msgs::MarkerArray marker_array;

    int id = 0;
    // Create a marker for each bounding box
    for (const auto& bbox : bboxes)
    {
        // Create the marker for the top square
        visualization_msgs::Marker top_square_marker;
        top_square_marker.header = msg.header;
        top_square_marker.ns = "bounding_boxes";
        top_square_marker.id = id++;
        top_square_marker.type = visualization_msgs::Marker::LINE_STRIP;
        top_square_marker.action = visualization_msgs::Marker::ADD;
        top_square_marker.pose.orientation.w = 1.0;
        top_square_marker.scale.x = 0.06;
        top_square_marker.color.r = bbox.r;
        top_square_marker.color.g = bbox.g;
        top_square_marker.color.b = bbox.b;
        top_square_marker.color.a = 1.0;

        // Add the points to the top square marker
        geometry_msgs::Point p1, p2, p3, p4;
        p1.x = bbox.x_max; p1.y = bbox.y_max; p1.z = bbox.z_max;
        p2.x = bbox.x_min; p2.y = bbox.y_max; p2.z = bbox.z_max;
        p3.x = bbox.x_min; p3.y = bbox.y_min; p3.z = bbox.z_max;
        p4.x = bbox.x_max; p4.y = bbox.y_min; p4.z = bbox.z_max;
        top_square_marker.points.push_back(p1);
        top_square_marker.points.push_back(p2);
        top_square_marker.points.push_back(p3);
        top_square_marker.points.push_back(p4);
        top_square_marker.points.push_back(p1);

        // Add the top square marker to the array
        marker_array.markers.push_back(top_square_marker);

        // Create the marker for the bottom square
        visualization_msgs::Marker bottom_square_marker;
        bottom_square_marker.header = msg.header;
        bottom_square_marker.ns = "bounding_boxes";
        bottom_square_marker.id = id++;
        bottom_square_marker.type = visualization_msgs::Marker::LINE_STRIP;
        bottom_square_marker.action = visualization_msgs::Marker::ADD;
        bottom_square_marker.pose.orientation.w = 1.0;
        bottom_square_marker.scale.x = 0.04;
        bottom_square_marker.color.r = bbox.r;
        bottom_square_marker.color.g = bbox.g;
        bottom_square_marker.color.b = bbox.b;
        bottom_square_marker.color.a = 1.0;

        // Add the points to the bottom square marker
        geometry_msgs::Point p5, p6, p7, p8;
        p5.x = bbox.x_max; p5.y = bbox.y_max; p5.z = bbox.z_min;
        p6.x = bbox.x_min; p6.y = bbox.y_max; p6.z = bbox.z_min;
        p7.x = bbox.x_min; p7.y = bbox.y_min; p7.z = bbox.z_min;
        p8.x = bbox.x_max; p8.y = bbox.y_min; p8.z = bbox.z_min;

        bottom_square_marker.points.push_back(p5);
        bottom_square_marker.points.push_back(p6);
        bottom_square_marker.points.push_back(p7);
        bottom_square_marker.points.push_back(p8);
        bottom_square_marker.points.push_back(p5); // connect the last point to the first point to close the square

        // Add the bottom square marker to the marker array
        marker_array.markers.push_back(bottom_square_marker);


        // Create the marker for the lines connecting the top and bottom squares
        visualization_msgs::Marker connecting_lines_marker;
        connecting_lines_marker.header = msg.header;
        connecting_lines_marker.ns = "bounding_boxes";
        connecting_lines_marker.id = id++;
        connecting_lines_marker.type = visualization_msgs::Marker::LINE_LIST;
        connecting_lines_marker.action = visualization_msgs::Marker::ADD;
        connecting_lines_marker.pose.orientation.w = 1.0;
        connecting_lines_marker.scale.x = 0.04;
        connecting_lines_marker.color.r = 0.0;
        connecting_lines_marker.color.g = 1.0;
        connecting_lines_marker.color.b = 0.0;
        connecting_lines_marker.color.a = 1.0;

        // Add the points to the connecting lines marker
        connecting_lines_marker.points.push_back(p1);
        connecting_lines_marker.points.push_back(p5);

        connecting_lines_marker.points.push_back(p2);
        connecting_lines_marker.points.push_back(p6);

        connecting_lines_marker.points.push_back(p3);
        connecting_lines_marker.points.push_back(p7);

        connecting_lines_marker.points.push_back(p4);
        connecting_lines_marker.points.push_back(p8);

        // Add the connecting lines marker to the marker array
        marker_array.markers.push_back(connecting_lines_marker);


        // Create a marker for the corners
        visualization_msgs::Marker corner_marker;
        corner_marker.header = msg.header;
        corner_marker.ns = "bounding_boxes";
        corner_marker.id = id++;
        corner_marker.type = visualization_msgs::Marker::SPHERE;
        corner_marker.action = visualization_msgs::Marker::ADD;
        corner_marker.pose.orientation.w = 1.0;
        corner_marker.scale.x = 0.4;
        corner_marker.scale.y = 0.4;
        corner_marker.scale.z = 0.4;
        corner_marker.color.r = bbox.r;
        corner_marker.color.g = 0.2;
        corner_marker.color.b = 0.5;
        corner_marker.color.a = 0.64;

        // Create a sphere for each corner and add it to the marker array

        corner_marker.pose.position = p1;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p2;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p3;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p4;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p5;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p6;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p7;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        corner_marker.pose.position = p8;
        corner_marker.id = id++;
        marker_array.markers.push_back(corner_marker);

        marker_pub.publish(marker_array);
    }
    all_clusters->header=pcl_cloud->header;
    pub.publish(all_clusters);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "road_node");
    ros::NodeHandle nh;
    Voxel_Filter voxel(nh);
    ros::spin();
    return 0;
}