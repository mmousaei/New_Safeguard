#include <ros/ros.h>
// PCL specific includes
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
#include <math.h>
#include "safeguard_internal_process.h"
#include "robot_config/Forward_Safeguard_Output.h"
#include <sensor_msgs/TimeReference.h>



typedef pcl::PointXYZ PointT;
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

ros::Publisher pointcloud1;
ros::Publisher pointcloud2;
ros::Publisher processed_info;
robot_config::Forward_Safeguard_Output output_topic;
static pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
pointcloud_process my_process;
int cloud_cnt = 0;
bool cloud_out = false;
float z_min = 1;
float z_max = 2.5;
float x_min = -1;
float x_max = 1;
float y_min = -1;
float y_max = 0;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    if(!cloud_out)
    {
        cloud_cnt++;

        //// Convert sensor_msgs to pcl::Pointcloud
        cloud = my_process.convert_msgs_to_pcl(input);

        std::cerr << "Cloud count is: " << cloud_cnt << std::endl;
        cloud_out = true;
    }

}

int main (int argc, char** argv)
{
    //// Initialize ROS

    ros::init (argc, argv, "pcl_subscriber");
    ros::NodeHandle nh;

    //// Create a ROS subscriber for the input point cloud

    ros::Subscriber sub = nh.subscribe ("/sick_visionary_t_driver/points", 5, cloud_cb);

    //// Create a ROS publisher for the output point cloud

    pointcloud1 = nh.advertise<sensor_msgs::PointCloud2> ("safeguard/filtered/points", 1);
    pointcloud2 = nh.advertise<sensor_msgs::PointCloud2> ("safeguard/obstacle/points", 1);
    processed_info = nh.advertise<robot_config::Forward_Safeguard_Output> ("safeguard/info", 1);

    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    sensor_msgs::PointCloud2 output1;

    int main_cnt = 0;


    while (ros::ok())
    {

        ros::spinOnce();

        sleep(5);
        if(cloud_out)
        {

            cloud_filtered = my_process.range_filter(cloud, z_min, z_max, "z");
            cloud_filtered = my_process.range_filter(cloud_filtered, x_min, x_max, "x");
            cloud_filtered = my_process.range_filter(cloud_filtered, y_min, y_max, "y");

//// Convert pcl::Pointcloud to sensor_msgs
            main_cnt++;
            std::cerr << "Main count is: " << main_cnt << std::endl;
//        std::cerr << "The size of the cloud_filtered is: " << cloud_filtered->size() << std::endl;
            output1 = my_process.convert_pcl_to_msgs(cloud_filtered);
            output1.header.frame_id = "/camera";
            pointcloud1.publish(output1);
            cloud_out = false;

            robot_config::Forward_Safeguard_Output output_topic;

            output_topic.distance = my_process.floor_decision(cloud_filtered->size(), 20);
            processed_info.publish(output_topic);
        }

    }

    //// Spin
    ros::spin ();
}