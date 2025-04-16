#ifndef ICP_NODE_HPP_
#define ICP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace ICP{
  class ICPNode : public rclcpp::Node{

  public:
  ICPNode();
  private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2::SharedPtr sub_;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr f_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  }
}
#endif ICP_NODE_HPP_




