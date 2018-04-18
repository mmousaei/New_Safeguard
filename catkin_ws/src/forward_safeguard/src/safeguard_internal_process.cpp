//
// Created by mohammad on 1/14/18.
//

#include "safeguard_internal_process.h"




pointcloud_process::pointcloud_process() {

}



pcl::PointCloud<PointT>::Ptr pointcloud_process::convert_msgs_to_pcl(const sensor_msgs::PointCloud2ConstPtr &input) {

    // Convert to PCL data type
    cloud_temp = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_temp);
    pcl_conversions::toPCL(*input, *cloud_temp);
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::fromPCLPointCloud2(*cloud_temp, *cloud);


    return cloud;
}

pcl::PointCloud<PointT>::Ptr pointcloud_process::range_filter(pcl::PointCloud<PointT>::Ptr cloud, float min_range, float max_range, std::string axis) {


    pass.setInputCloud (cloud);
    pass.setFilterFieldName (axis);
    // pass.setFilterLimits (.9, 1.5);
    // pass.setFilterLimits (1, 1.25); // for close-end

    pass.setFilterLimits (min_range, max_range);
    // pass.setFilterLimits (2, 2.2); // big pipe

    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    pass.filter (*cloud_filtered);


    return cloud_filtered;
}

pcl::PointCloud<pcl::Normal>::Ptr pointcloud_process::estimate_normals(pcl::PointCloud<PointT>::Ptr cloud_filtered) {

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    return cloud_normals;
}

pointcloud_process::angle_3d pointcloud_process::estimate_angular_offset(pcl::ModelCoefficients &coefficients_cylinder) {

    angle_3d angle;

    double cos_temp;

    double x = coefficients_cylinder.values[3];
    double y = coefficients_cylinder.values[4];
    double z = coefficients_cylinder.values[5];

    if(z < 0)
    {
        z = -z;
        x = -x;
        y = -y;
    }
    cos_temp = z*100 / sqrt(x*x*10000.0 + y*y*10000.0 + z*z*10000.0);
    angle.roll = (acos(cos_temp)*180.0/M_PI);
//    if(coefficients_cylinder.values[5] < 0)
//        angle.pitch = 180 - angle.pitch;
//    if(angle.pitch > 90)
//    {
//        angle.pitch -= 180;
//        angle.pitch = fabs(angle.pitch);
//    }
    cos_temp = y*100 / sqrt(x*x*10000.0 + y*y*10000.0 + z*z*10000.0);
    angle.yaw = (acos(cos_temp)*180.0/M_PI);
//    if(coefficients_cylinder.values[5] < 0)
//        angle.yaw = 180 - angle.yaw;
    cos_temp = x*100 / sqrt(x*x*10000.0 + y*y*10000.0 + z*z*10000.0);
    angle.pitch = (acos(cos_temp)*180.0/M_PI);
//    if(coefficients_cylinder.values[5] < 0)
//        angle.roll = 180 - angle.roll;



    return angle;
}

sensor_msgs::PointCloud2 pointcloud_process::convert_pcl_to_msgs(pcl::PointCloud<PointT>::Ptr cloud) {

    // Convert to ROS data type

    cloud_temp = new pcl::PCLPointCloud2;
    sensor_msgs::PointCloud2 output;
    // pcl::PCLPointCloud2* cloud_temp2 = new pcl::PCLPointCloud2;
//   pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_temp);
    pcl::toPCLPointCloud2(*cloud , *cloud_temp);
    pcl_conversions::fromPCL(*cloud_temp, output);
    // pcl::toROSMsg(output, cloud_cylinder);
    // Publish the data



    return output;
}

pointcloud_process::point_2d pointcloud_process::estimate_cylinder_center_average(pcl::PointCloud<PointT>::Ptr cloud) {

    point_2d center;
    double radius = .32;
    double dist;
    bool notSafe = false;
    double sum = 0;
    double xcenter = 0;
    double ycenter = 0;
    std_msgs::Int8 temp1;
//    temp1.data = 0;
    for(int i = 0; i < cloud->points.size(); i++)
    {
        xcenter += cloud->points[i].x;
        ycenter += cloud->points[i].y;
    }

    xcenter /= cloud->points.size();
    ycenter /= cloud->points.size();

    center.x = xcenter;
    center.y = ycenter;

    return center;
}

int pointcloud_process::safeguard_decision_maker(pcl::PointCloud<PointT>::Ptr cloud, pointcloud_process::point_2d center, unsigned long size) {

    double radius = .4;
    double dist;
    bool notSafe = false;
    double sum = 0;
    double xcenter = 0;
    double ycenter = 0;
    std_msgs::Int8 temp1;
    int temp = 0;
    int notSafeCount = 0;
    for(int i = 0; i < cloud->points.size(); i++)
    {
        dist = ( (cloud->points[i].x - center.x) * (cloud->points[i].x - center.x) ) + ( (cloud->points[i].y - center.y) * (cloud->points[i].y - center.y) );

//        std::cerr<<"Distance of obstacle is: "<< dist << std::endl;

        sum += dist;

        if ( dist < radius*radius )
        {
            // notSafe = true;
            notSafeCount++;
//            temp1.data = 1;
            // std::cerr << "The distance squared of the point to the origin is " << dist << " meters and the radius squared is " << radius*radius <<std::endl;
            // distance.data = dist;
        }
    }
    if ((notSafeCount) > size/200.0)
    {
        notSafe = true;
        temp = 1;
    }

    return temp;
}


pointcloud_process::point_2d pointcloud_process::estimate_cylinder_center_at_Z(pointcloud_process::line_3d line, double Z) {
    double t;
    pointcloud_process::point_2d center;

//    t = -(line.vector.x*(line.point.x + center_at_zero.x) + line.vector.y*(line.point.y + center_at_zero.y) +line.vector.z*(line.point.z+Z+center_at_zero.z)) / (line.vector.x*line.vector.x + line.vector.y*line.vector.y + line.vector.z*line.vector.z);
//    t = -(line.vector.x*(line.point.x) + line.vector.y*(line.point.y) +line.vector.z*(line.point.z+Z)) / (line.vector.x*line.vector.x + line.vector.y*line.vector.y + line.vector.z*line.vector.z);
    t = (Z - line.point.z) / line.vector.z;
    center.x = line.vector.x*t + line.point.x;
    center.y = line.vector.y*t + line.point.y;
//    center.z = line.vector.z*t + line.point.z;

    return center;
}



pointcloud_process::point_3d pointcloud_process::estimate_center_at_zero(pointcloud_process::line_3d line) {

    double t;
    pointcloud_process::point_3d center;

    t = -(line.vector.x*line.point.x + line.vector.y*line.point.y +line.vector.z*line.point.z) / (line.vector.x*line.vector.x + line.vector.y*line.vector.y + line.vector.z*line.vector.z);

    center.x = line.vector.x*t + line.point.x;
    center.y = line.vector.y*t + line.point.y;
    center.z = line.vector.z*t + line.point.z;

    return center;
}

int pointcloud_process::open_closed_pipe_decision_maker(unsigned long cloud_size, unsigned long inlier_size)
{

    std::cerr<<"The ratio is : " << (double) inlier_size/cloud_size << std::endl;

    int decision;
    decision = ((double) inlier_size) > ((double) (0.9 * cloud_size)) ? 0 : 1;

    return decision;
}


double pointcloud_process::find_last_cloud(pcl::PointCloud<PointT>::Ptr cloud) {
    if (cloud->points.size() > 3)
        return 5;
    double distance = 0;
    for (int i = 0; i < cloud->points.size(); i++) {
        if (cloud->points[i].z > distance) {
            distance = cloud->points[i].z;
        }
    }
    return distance;
}

double pointcloud_process::find_first_cloud(pcl::PointCloud<PointT>::Ptr cloud) {
    if (cloud->points.size() == 0)
        return 0;
    double distance = 100000;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        if( cloud->points[i].z < distance)
        {
            distance = cloud->points[i].z;
        }
    }
    return distance;
}

double pointcloud_process::find_average_cloud(pcl::PointCloud<PointT>::Ptr cloud) {
    if (cloud->points.size() == 0)
        return 0;
    double distance = 0;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        distance += cloud->points[i].z;
    }
    distance /= cloud->points.size();
    return distance;
}

pointcloud_process::decision_distance
pointcloud_process::decision_distance_estimatie(double decision_ratio, double min_distance,
                                                unsigned long cloud_size, unsigned long inlier_size, pcl::PointCloud<PointT>::Ptr obstacle_cloud, pcl::PointCloud<PointT>::Ptr cylinder_cloud)
{
    pointcloud_process::decision_distance output;
    double ratio = (double) inlier_size/cloud_size;
    double max_cylinder = pointcloud_process::find_last_cloud(cylinder_cloud);
    double max_obstacle = pointcloud_process::find_last_cloud(obstacle_cloud);


    std::cerr<<"The ratio is : " << ratio << std::endl;
    if(cylinder_cloud->empty())
    {
        output.decision = open;
        output.distance = 0;
    }
    if(obstacle_cloud->size() > 10)
    {
        if(ratio < decision_ratio)
        {
            if(fabs(max_cylinder-max_obstacle) < min_distance)
            {
                output.decision = close;
                double ave_obs = find_average_cloud(obstacle_cloud);
                double min_aveobs_maxcyl = ave_obs > max_cylinder ? max_cylinder : ave_obs;
                output.distance = min_aveobs_maxcyl;
//                std::cerr<<"min_aveobs_maxcyllllllllllllllllllllllllllllllllllllllllllllllllllllllll is: " << output.distance <<std::endl;
            }
            else
            {
                output.decision = open;
                double min_cyl_obs;
                min_cyl_obs = max_cylinder > max_obstacle ? max_obstacle : max_cylinder;
                output.distance = min_cyl_obs;
            }
        }
        else
        {
            output.decision = obstacle;
            output.distance = find_average_cloud(obstacle_cloud);
        }
    }
    else
    {
        if(max_cylinder < 0.7)
        {
            output.distance = open;
            output.distance = max_cylinder;
        }
        else
        {
            output.decision = safe;
            output.distance = 3.0;
        }
    }

    return output;
}

bool pointcloud_process::complete_circle(pcl::PointCloud<PointT>::Ptr cloud, pointcloud_process::point_2d center) {

    bool output;
//    std::vector<int> count(8,0);
    int count[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    if(!cloud->size())
    {
        std::cerr<<"HEREEEEEEEEEEEEEEEEeee"<<std::endl;
        return 0;
    }
    for(int i = 0; i < cloud->size(); i++)
    {
        double x = cloud->points[i].x - center.x;
        double y = cloud->points[i].y - center.y;
        if( x > 0 && y > 0)
        {
            if(x > y)
            {
                count[0]++;
            }
            else
            {
                count[1]++;
            }
        }
        else if(x > 0 && y < 0)
        {
            if(std::fabs(x) > std::fabs(y))
            {
                count[2]++;
            }
            else
            {
                count[3]++;
            }
        }
        else if(x < 0 && y > 0)
        {
            if(std::fabs(x) > std::fabs(y))
            {
                count[4]++;
            }
            else
            {
                count[5]++;
            }
        }
        else if(x < 0 && y < 0)
        {
            if(std::fabs(x) > std::fabs(y))
            {
                count[6]++;
            }
            else
            {
                count[7]++;
            }
        }
    }



    output = 0;
    for(int i = 0; i < 8; i++)
    {
        if(count[i] > 0)
        {
            count[8]++;
        }
    }
    if(count[8] > 7) output = true; else output = false;


    return output;
}

bool pointcloud_process::floor_decision(int cloud_size, int threshold) {

    if(cloud_size > threshold)
    {
        return true;
    }
    else
    {
        return false;
    }

}


//double pointcloud_process::scan_for_eop_backwards(pcl::PointCloud<PointT>::Ptr cylinder, double max_distance, pointcloud_process::line_3d line) {
//
//    if(cylinder->empty())
//        return max_distance;
//    int max_cnt = 200;
//    bool eop = 0;
//    pcl::PointCloud<PointT>::Ptr temp (new pcl::PointCloud<PointT>);
//    double max_thresh = max_distance+0.04;
//    double min_thresh = max_distance;
//    pointcloud_process::point_2d center;
//    int cnt = 0;
//    while(!eop)
//    {
//        cnt++;
//        if(cnt > max_cnt) break;
//        temp = range_filter(cylinder, min_thresh, max_thresh);
//        center = estimate_cylinder_center_at_Z(line, max_thresh);
//        eop = complete_circle(temp, center);
//
//        std::cerr<<"In the while loop of scan for eop, the cylinder size is: " << cnt << " and the filtered size: " << temp->size() << std::endl;
//        if(max_thresh > 0.5)
//        {
//            max_thresh -= 0.02;
//            min_thresh -= 0.02;
//        }
//        else
//        {
//            eop = 1;
//        }
//    }
//
//    return max_thresh;
//}

//double pointcloud_process::scan_for_eop_forward(pcl::PointCloud<PointT>::Ptr cylinder, double max_distance,
//                                                pointcloud_process::line_3d line) {
//    if(cylinder->empty())
//        return max_distance;
//    int max_cnt = 200;
//    bool eop = 1;
//    pcl::PointCloud<PointT>::Ptr temp (new pcl::PointCloud<PointT>);
//    double max_thresh = max_distance-0.04;
//    double min_thresh = max_distance-0.08;
//    pointcloud_process::point_2d center;
//    int cnt = 0;
//    while(eop)
//    {
//        cnt++;
//        if(cnt > max_cnt) break;
//        temp = range_filter(cylinder, min_thresh, max_thresh);
//        center = estimate_cylinder_center_at_Z(line, max_thresh);
//        eop = complete_circle(temp, center);
//
//        std::cerr<<"In the second while loop of scan for eop, the cylinder size is: " << cnt << " and the filtered size: " << temp->size() << std::endl;
//        if(max_thresh < 3)
//        {
//            max_thresh += 0.02;
//            min_thresh += 0.02;
//        }
//        else
//        {
//            return 3;
//        }
//    }
//
//    return max_thresh;
//}






