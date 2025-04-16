#include <memory>
#include "nav2_msgs/action/navigate_to_pose.hpp"  
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp" 
#include "rclcpp_action/rclcpp_action.hpp" 
// #include "rm_serial_driver/pose.hpp"
#include <std_msgs/msg/int8.hpp>
namespace rm_serial_driver
{
using NavigationAction = nav2_msgs::action::NavigateToPose;

class NavToPoseClient : public rclcpp::Node {
 public:
  using NavigationActionClient = rclcpp_action::Client<NavigationAction>;  // 定义导航动作客户端类型
  using NavigationActionGoalHandle =
      rclcpp_action::ClientGoalHandle<NavigationAction>;  // 定义导航动作目标句柄类型

   NavToPoseClient(const rclcpp::NodeOptions & options);

  void GoalPoseSubCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

  double pose_x;
  double pose_y;
  std_msgs::msg::Int8 status;
  std_msgs::msg::Int8 ssta;

  void sendGoal() ;
private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub;
  NavigationActionClient::SharedPtr action_client_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr nav_status;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr is_goal;

};
}