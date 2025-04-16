
#include "pointcloud_to_laserscan/remove_pointcloud.hpp"
#include <cmath>
#include <pcl/common/common.h>
#include <limits>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace NANA
{
 void RemovePointcloud::LivoxLidarSubCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
    god_frame_ = this->declare_parameter("god_frame", "");
    //pcl::PointCloud<pcl::PointXYZ>::Ptr livox_lidar_pointcloud_(new pcl::PointCloud<pcl::PointXYZ>);
    //ROS点云(msg) transform to pcl's format & store in object:livox_lidar_pointcloud_
    pcl::fromROSMsg(*msg, *livox_lidar_pointcloud_);
    }

void RemovePointcloud::ComparePointclouds(pcl::PointCloud<pcl::PointXYZ>::Ptr livox_lidar_pointcloud_)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cough_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &cloud_b : cloud_b->points)
    {
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        if (kdTree_.nearestKSearch(cloud_b, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            float z_diff = cloud_b.z - map_cloud_->points[pointIdxNKNSearch[0]].z;
            if (z_diff >= dis_diff_) 
            {   
                result_cloud->points.push_back(pcl::PointXYZ(cloud_b.x, cloud_b.y, cloud_b.z));
                cough_cloud->points.push_back(pcl::PointXYZ(cloud_b.x, cloud_b.y, cloud_b.z));
            }
            else if(z_diff >= 0.05)
            {
                cough_cloud->points.push_back(pcl::PointXYZ(cloud_b.x, cloud_b.y, cloud_b.z));
            }
        }
    }
    senosor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*livox_lidar_pointcloud_, output);
    output.header.stamp = this->now();
    processed_point_cloud_pub_->publish(output);
    pcl::VoxelGrid<pcl::PointXYZ> ker;
    ker.setInputCloud(cough_cloud);
    ker.setLeafSize(0.01f, 0.01f, 0.01f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr f_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ker.filter(*f_cloud);
}

//PassThrough Filter
double RemovePointcloud::calculateZAngleThreshold()
{
    return tan(45.0 * M_PI / 180.0);
}

void RemovePointcloud::applyPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &f_pointcloud,
                                              double min_z, double max_z){
pcl::PassThrough<pcl::PointXYZ> pass;
pass.setInputCloud(f_pointcloud);
pass.setFilterFieldName("z");
pass.setFilterLimits(min_z,max_z);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pass.filter(*cloud_filtered);
*f_pointcloud = *cloud_filtered;
}

//Bilateral  Filter
void RemovePointcloud::applyBilateralFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &f_pointcloud,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud_with_intensity)
{  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_intensity(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*input_cloud_with_intensity, *cloud_without_intensity);
    pcl::FastBilateralFilter<pcl::PointXYZ> bilateral_filter;
    bilateral_filter.setInputCloud(f_pointcloud);
    bilateral_filter.setSigmaS(3);
    bilateral_filter.setSigmaR(0.05);
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud_without_intensity;
    bilateral_filter.filter(filtered_cloud_without_intensity);
}

void RemovePointcloud::filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &f_pointcloud)
{
    double z_threshold = calculateZAngleThreshold();
    applyPassThroughFilter(f_pointcloud,z_threshold, std::numeric_limits<double>::max());
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_with_intensity(new pcl::PointCloud<pcl::PointXYZ>);
    applyBilateralFilter(f_pointcloud, input_cloud_with_intensity);
    *f_pointcloud = *input_cloud_with_intensity;
}
} 

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(NANA::RemovePointcloud);