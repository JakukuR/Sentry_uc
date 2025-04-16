#ifndef REMOVEPOINTCLOUD_H
#define REMOVEPOINTCLOUD_H

#include <iostream>
#include <chrono>
#include <queue>
#include <memory>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/fast_bilateral.h>

namespace NANA
{
const int ACC_POINT_NUM = 3;
class RemovePointcloud : public rclcpp::Node{
public:
    REMOVEPOINTCLOUD_H
    explicit RemovePointcloud(const rclcpp::NodeOptions &options);
    ~RemovePointcloud() override;
    void removeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &livox_lidar_pointcloud_);
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cough_point_cloud_pub_;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr removed_cloud_pub_;
    std::string god_frame_;
    std::string filename_ = "";
    void LivoxLidarSubCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void ComparePointclouds(pcl::PointCloud<pcl::PointXYZ>::Ptr livox_lidar_pointcloud_);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr livox_lidar_pointcloud_;
    double dis_diff_ = 0.0;
    double calculateZAngleThreshold();
    void applyPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &f_pointcloud,
                                double min_z, double max_z);
    void applyBilateralFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &f_pointcloud,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud_with_intensity);
    //cilent
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client_;
};
}

#endif // REMOVEPOINTCLOUD_H