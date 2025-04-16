#include <memory>
#include "decision_msgs/msg/target_point.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


using NavigationAction = nav2_msgs::action::NavigateToPose;
decision_msgs::msg::TargetPoint target_point;

class NavToPoseClient : public rclcpp::Node {
 public:
  using NavigationActionClient = rclcpp_action::Client<NavigationAction>;  // 定义导航动作客户端类型
  using NavigationActionGoalHandle =
      rclcpp_action::ClientGoalHandle<NavigationAction>;  // 定义导航动作目标句柄类型

  NavToPoseClient() : Node("nav_to_pose_client") 
  {
    // 创建导航动作客户端
    action_client_ = rclcpp_action::create_client<NavigationAction>(
        this, "navigate_to_pose");
    location_point=this->create_subscription<decision_msgs::msg::TargetPoint>("nav_point",10,std::bind(&NavToPoseClient::nav_point_callback, this, std::placeholders::_1));
  }

  void nav_point_callback(const decision_msgs::msg::TargetPoint & msg)
  {
    target_point=msg;
  }

  void sendGoal() {
    // 等待导航动作服务器上线，等待时间为5秒
    while (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_INFO(get_logger(), "等待Action服务上线。");
    }
    // 设置导航目标点
    target_point.point_x=0.0;
    target_point.point_y=0.0;
    auto goal_msg = NavigationAction::Goal();
    goal_msg.pose.header.frame_id = "map";  // 设置目标点的坐标系为地图坐标系
    goal_msg.pose.pose.position.x = target_point.point_x;  // 设置目标点的x坐标为target_point.x
    goal_msg.pose.pose.position.y = target_point.point_y;  // 设置目标点的y坐标为target_point.y

    auto send_goal_options =
        rclcpp_action::Client<NavigationAction>::SendGoalOptions();
    // 设置请求目标结果回调函数
    send_goal_options.goal_response_callback =
        [this](NavigationActionGoalHandle::SharedPtr goal_handle) {
          if (goal_handle) {
            RCLCPP_INFO(get_logger(), "目标点已被服务器接收");
          }
        };
    // 设置移动过程反馈回调函数
    send_goal_options.feedback_callback =
        [this](
            NavigationActionGoalHandle::SharedPtr goal_handle,
            const std::shared_ptr<const NavigationAction::Feedback> feedback) {
          (void)goal_handle;  // 假装调用，避免 warning: unused
          RCLCPP_INFO(this->get_logger(), "反馈剩余距离:%f",
                      feedback->distance_remaining);
        };
    // 设置执行结果回调函数
    send_goal_options.result_callback =
        [this](const NavigationActionGoalHandle::WrappedResult& result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "处理成功！");
          }
        };
    // 发送导航目标点
    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  NavigationActionClient::SharedPtr action_client_;
  rclcpp::Subscription<decision_msgs::msg::TargetPoint>::SharedPtr location_point;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavToPoseClient>();
  node->sendGoal();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}