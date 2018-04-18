//
// Created by mohammad on 1/14/18.
//

#ifndef PROJECT_POINTCLOUD_PROCESS_H
#define PROJECT_POINTCLOUD_PROCESS_H


#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include "std_msgs/Byte.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include <string>


typedef pcl::PointXYZ PointT;


class pointcloud_process {

public:
    struct angle_3d
    {
        double roll;
        double pitch;
        double yaw;
    };
    struct point_2d
    {
        double x;
        double y;
    };
    struct point_3d
    {
        double x;
        double y;
        double z;
    };
    struct line_3d
    {
        pointcloud_process::point_3d point;
        pointcloud_process::point_3d vector;
    };
    struct plane_3d
    {
        pointcloud_process::point_3d point;
        pointcloud_process::point_3d vector;
    };
    enum end_condition { open, close, obstacle, safe};
    struct decision_distance
    {
        double  distance;
        end_condition decision;
    };
    pointcloud_process();
    pcl::PointCloud<PointT>::Ptr convert_msgs_to_pcl(const sensor_msgs::PointCloud2ConstPtr& input);
    pcl::PointCloud<PointT>::Ptr range_filter(pcl::PointCloud<PointT>::Ptr cloud, float min_range, float max_range, std::string axis);
    pcl::PointCloud<pcl::Normal>::Ptr estimate_normals(pcl::PointCloud<PointT>::Ptr cloud_filtered);
    pointcloud_process::angle_3d estimate_angular_offset(pcl::ModelCoefficients& coefficients_cylinder);
    sensor_msgs::PointCloud2 convert_pcl_to_msgs(pcl::PointCloud<PointT>::Ptr cloud);
    pointcloud_process::point_2d estimate_cylinder_center_average(pcl::PointCloud<PointT>::Ptr cloud);
    pointcloud_process::point_2d estimate_cylinder_center_at_Z(pointcloud_process::line_3d line, double Z);
    bool complete_circle(pcl::PointCloud<PointT>::Ptr cloud, pointcloud_process::point_2d center);
    int safeguard_decision_maker(pcl::PointCloud<PointT>::Ptr cloud, pointcloud_process::point_2d center, unsigned long size);
    pointcloud_process::point_3d estimate_center_at_zero(pointcloud_process::line_3d line);
    int open_closed_pipe_decision_maker(unsigned long cloud_size, unsigned long inlier_size);
    double find_last_cloud(pcl::PointCloud<PointT>::Ptr cloud);
    double find_first_cloud(pcl::PointCloud<PointT>::Ptr cloud);
    double find_average_cloud(pcl::PointCloud<PointT>::Ptr cloud);
    pointcloud_process::decision_distance decision_distance_estimatie(double decision_ratio, double min_distance, unsigned long cloud_size, unsigned long inlier_size, pcl::PointCloud<PointT>::Ptr obstacle_cloud, pcl::PointCloud<PointT>::Ptr cylinder_cloud);
//    double scan_for_eop_backwards(pcl::PointCloud<PointT>::Ptr cylinder, double max_distance, pointcloud_process::line_3d line);
//    double scan_for_eop_forward(pcl::PointCloud<PointT>::Ptr cylinder, double max_distance, pointcloud_process::line_3d line);
    bool floor_decision(int cloud_size, int threshold);


private:
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::PCLPointCloud2* cloud_temp;

};


#endif //PROJECT_POINTCLOUD_PROCESS_H
