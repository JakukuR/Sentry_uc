#include "pointcloud_to_laserscan/ICP_pointcloud.hpp"

namespace ICP {
    ICP::ICPNode() : Node("icp_node")
    {
    // Initialize the ICP algorithm with a maximum of 10 iterations
    icp_.setMaxIterations(15);
    icp_.setEuclideanFitnessEpsilon(0.01); // Set a small threshold for convergence

    // Create a subscriber to receive point cloud data
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_cloud", 10, std::bind(&ICPNode::cloudCallback, this, std::placeholders::_1));

    // Create a publisher to send the aligned point cloud
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_cloud", 10);
    }

    void ICPNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
    icp_frame_ = this->declare_parameter("icp_frame", "");
    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr f_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *f_pointcloud);

    // Perform ICP
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    icp_.setInputSource(f_pointcloud);
    icp_.align(*aligned_cloud);

    if (icp_.hasConverged())
    {
    RCLCPP_INFO(this->get_logger(), "ICP has converged, score is %f", icp_.getFitnessScore());
    }
    else
    {
    RCLCPP_INFO(this->get_logger(), "ICP did not converge.");
    }

    // Convert the aligned point cloud back to a ROS message
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*aligned_cloud, output_msg);

    // Publish the aligned point cloud
    pub_->publish(output_msg);
    }

    int main(int argc, char **argv)
    {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ICPNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ICP::ICPPointcloud);


